#!/usr/bin/env python3
"""
CB1 Stepper Control — Generate trajectory CSV & send to STM32 via USB CDC
===================================================================

Usage:
  python cb1_stepper_control.py generate       # Generate trajectory CSV
  python cb1_stepper_control.py send <file>     # Send CSV commands to STM32
  python cb1_stepper_control.py all             # Generate + send in one go
  python cb1_stepper_control.py test            # Quick connectivity test

Protocol (from main_pingpong.c / main.c):
  Move  : 0xAA 'M' [M0:2B][M1:2B]...[M5:2B] 0x0A   (15 bytes, big-endian int16)
  Estop : 0xAA 'E' 0x0A
  Status: 0xAA 'S' 0x0A   → Response: 0xAA 'S' [status] [M0..M5 steps_done] 0x0A

CSV format:
  M0,M1,M2,M3,M4,M5     (header)
  <int16>,<int16>,...   (6 columns, signed step counts per 20ms segment)
"""

import serial
import struct
import time
import csv
import os
import sys
import random
import argparse

# ────────────────────────────────────────────────────────────────────────
# Protocol constants (match STM32 firmware)
# ────────────────────────────────────────────────────────────────────────
SOF        = 0xAA
EOL        = 0x0A
CMD_MOVE   = ord('M')
CMD_ESTOP  = ord('E')
CMD_STATUS = ord('S')

# ────────────────────────────────────────────────────────────────────────
# Trajectory constants
# ────────────────────────────────────────────────────────────────────────
NUM_AXES       = 6          # 6 stepper motors
TOTAL_TARGET   = 10000      # Total pulses per axis
MAX_SEGMENT    = 3000       # Max pulses in any single segment per axis
SEGMENT_TIME_S = 0.020      # 20 ms per segment (matches DDA tick: 4000 × 5µs)

DEFAULT_CSV    = "trajectory.csv"
DEFAULT_PORT   = "/dev/ttyACM0"
DEFAULT_BAUD   = 115200


# ════════════════════════════════════════════════════════════════════════
# SECTION 1 — GENERATE TRAJECTORY → CSV
# ════════════════════════════════════════════════════════════════════════

def generate_trajectory_csv(output_path: str):
    """
    Generate a CSV with simulated step counts for 6 axes.

    Each axis independently ramps up to TOTAL_TARGET (10000) pulses.
    Each segment row is at most MAX_SEGMENT (3000) pulses per axis,
    matching the format expected by main_pingpong.c.

    A simple acceleration / deceleration profile is used so the
    pulse distribution looks like a natural trapezoidal move.
    """
    # ── Build a per-axis step plan ──────────────────────────────────
    # We split each axis's remaining steps into chunks of
    # decreasing size (simulating accel/decel profile).
    # The first few segments are large, last ones taper off.
    all_rows = []

    # Track remaining steps per axis
    remaining = [TOTAL_TARGET] * NUM_AXES

    # For a more realistic motion profile, we use a "pace" factor
    # that starts high and decreases as we approach the target.
    segment_index = 0

    print(f"[GENERATE] Creating trajectory: {NUM_AXES} axes × ~{TOTAL_TARGET} pulses each")
    print(f"[GENERATE] Max {MAX_SEGMENT} pulses per segment, {SEGMENT_TIME_S*1000:.0f} ms per segment\n")

    while any(r > 0 for r in remaining):
        row = [0] * NUM_AXES
        # Progress factor: 1.0 at start → 0.0 at end
        # Fraction of work done across all axes
        done_fraction = 1.0 - (sum(remaining) / (NUM_AXES * TOTAL_TARGET))
        
        # Use a sine-based profile to create smooth accel/decel
        # "Pace" = multiplier for how aggressive the segment is:
        #   high at start (acceleration), lower in middle (cruise),
        #   tapers at end (deceleration).
        # We use a cosine curve: cos(π * t) goes from 1 → -1
        # mapped to range [0.3 .. 1.0]
        import math
        pace = 0.7 * (1.0 - math.cos(math.pi * done_fraction * 2.0))
        pace = max(0.3, min(1.0, pace))  # clamp

        for axis in range(NUM_AXES):
            if remaining[axis] <= 0:
                continue

            # Segment size: between MAX_SEGMENT * 0.2 to MAX_SEGMENT, scaled by pace
            max_chunk = int(MAX_SEGMENT * pace)
            if max_chunk < 50:
                max_chunk = min(remaining[axis], 50)  # last tiny chunks

            max_chunk = min(max_chunk, remaining[axis])
            min_chunk = max(1, int(max_chunk * 0.3))
            if min_chunk > max_chunk:
                chunk = max_chunk
            else:
                chunk = random.randint(min_chunk, max_chunk)
            remaining[axis] -= chunk
            row[axis] = chunk

        all_rows.append(row)

        # Safety: prevent infinite loop
        segment_index += 1
        if segment_index > 10000:
            print("[ERROR] Hit segment limit (10000), something went wrong!")
            break

    # ── Write CSV ───────────────────────────────────────────────────
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['M0', 'M1', 'M2', 'M3', 'M4', 'M5'])  # header
        for row in all_rows:
            writer.writerow(row)

    # ── Summary ─────────────────────────────────────────────────────
    totals = [sum(row[i] for row in all_rows) for i in range(NUM_AXES)]
    max_seg = max(len(row) for row in all_rows)

    print(f"[GENERATE] ✓ Wrote {len(all_rows)} segments to '{output_path}'")
    print(f"[GENERATE]   Duration: {len(all_rows) * SEGMENT_TIME_S:.3f} s")
    print(f"[GENERATE]   Total steps per axis:")
    for i in range(NUM_AXES):
        print(f"     M{i}: {totals[i]:>6} steps  (max segment: {max(row[i] for row in all_rows):>5})")

    return all_rows


# ════════════════════════════════════════════════════════════════════════
# SECTION 2 — READ CSV
# ════════════════════════════════════════════════════════════════════════

def read_csv(filepath: str):
    """Read CSV and return list of segments (each is list of 6 ints)."""
    segments = []
    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        # Skip header if present
        first_row = next(reader, None)
        if first_row is None:
            return segments

        # Check if first row is a header (starts with 'M')
        if first_row[0].startswith('M') or first_row[0].startswith('m'):
            pass  # header skipped
        else:
            # First row is actual data
            try:
                row = [int(x.strip()) for x in first_row[:NUM_AXES]]
                segments.append(row)
            except ValueError:
                pass

        # Read remaining rows
        for line in reader:
            if line and len(line) >= NUM_AXES:
                try:
                    row = [int(x.strip()) for x in line[:NUM_AXES]]
                    segments.append(row)
                except ValueError:
                    print(f"[WARN] Skipping invalid row: {line}")

    print(f"[INFO] Read {len(segments)} segments from '{filepath}'")
    return segments


# ════════════════════════════════════════════════════════════════════════
# SECTION 3 — USB CDC COMMUNICATION WITH STM32
# ════════════════════════════════════════════════════════════════════════

class StepperController:
    """
    Controls the STM32 stepper driver via USB CDC (Virtual COM Port).

    Implements the protocol defined in main_pingpong.c / main.c:
      - Move  : AA 'M' + 6×int16 (big-endian) + 0A
      - Estop : AA 'E' + 0A
      - Status: AA 'S' + 0A
    """

    def __init__(self, port: str = DEFAULT_PORT, baudrate: int = DEFAULT_BAUD,
                 timeout: float = 1.0):
        self.port = port
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        # Give STM32 time to enumerate after opening the port
        time.sleep(2)
        print(f"[SERIAL] Connected to STM32 on {port} @ {baudrate} baud")

    # ── low-level send ──────────────────────────────────────────────

    def _send_packet(self, data: bytes):
        """Write raw bytes to serial."""
        self.ser.write(data)

    def _read_byte(self) -> int | None:
        """Read a single byte, or None on timeout."""
        b = self.ser.read(1)
        return b[0] if b else None

    # ── high-level commands ─────────────────────────────────────────

    def send_move(self, steps: list[int]) -> str | None:
        """
        Send a 'M' move command with 6 step values.

        Args:
            steps: List of 6 signed int16 values (one per axis).
                   Positive = forward, Negative = reverse.

        Returns:
            'K'  if command was queued (ack)
            'N'  if queue was full (nack)
            None on timeout or error
        """
        if len(steps) != NUM_AXES:
            raise ValueError(f"Expected {NUM_AXES} steps, got {len(steps)}")

        # Build packet: 0xAA 'M' [M0][M1][M2][M3][M4][M5] 0x0A
        packet = bytes([SOF, CMD_MOVE])
        for s in steps:
            packet += struct.pack('>h', int(s))  # big-endian int16
        packet += bytes([EOL])

        self.ser.write(packet)
        resp = self._read_byte()

        if resp is None:
            return None
        return chr(resp)

    def send_move_async(self, steps: list[int]):
        """Send a 'M' move command asynchronously without blocking to read."""
        if len(steps) != NUM_AXES:
            raise ValueError(f"Expected {NUM_AXES} steps, got {len(steps)}")
        packet = bytes([SOF, CMD_MOVE])
        for s in steps:
            packet += struct.pack('>h', int(s))
        packet += bytes([EOL])
        self.ser.write(packet)
        self.ser.flush()

    def send_estop(self):
        """Send emergency stop command."""
        self._send_packet(bytes([SOF, CMD_ESTOP, EOL]))
        print("[ESTOP] Emergency stop sent to STM32")

    def get_status(self) -> dict | None:
        """
        Query STM32 status.

        Returns dict with:
            'running': True if simulation is running, False otherwise
        Or None if no valid response.
        """
        self._send_packet(bytes([SOF, CMD_STATUS, EOL]))

        # Read response (30 bytes max)
        resp = self.ser.read(30)
        if (len(resp) >= 3 and resp[0] == SOF and resp[1] == CMD_STATUS):
            return {
                'running': resp[2] == 0x01,
            }
        return None

    def wait_idle(self, poll_interval: float = 0.02, timeout: float = 60.0):
        """
        Poll until STM32 reports motion stopped.

        Args:
            poll_interval: Time between status polls (seconds)
            timeout: Max time to wait (seconds). 0 = no timeout

        Returns True when idle, False if timeout.
        """
        start = time.time()
        while True:
            status = self.get_status()
            if status and not status['running']:
                elapsed = time.time() - start
                print(f"[IDLE] Motion completed after {elapsed:.2f}s")
                return True

            if timeout > 0 and (time.time() - start) > timeout:
                print(f"[WARN] Timeout waiting for idle ({timeout}s)")
                return False

            time.sleep(poll_interval)

    # ── cleanup ─────────────────────────────────────────────────────

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[SERIAL] Port closed")


# ════════════════════════════════════════════════════════════════════════
# SECTION 4 — SEND TRAJECTORY TO STM32
# ════════════════════════════════════════════════════════════════════════

def send_trajectory(port: str, csv_path: str, baudrate: int = DEFAULT_BAUD):
    """
    Read CSV segments and stream them to STM32 using a sliding window flow control
    (CMD_QUEUE_SIZE = 4, queue limit = 3).
    """
    if not os.path.exists(csv_path):
        print(f"[ERROR] CSV file not found: '{csv_path}'")
        return False

    segments = read_csv(csv_path)
    if not segments:
        print("[ERROR] No valid segments found in CSV")
        return False

    try:
        ctrl = StepperController(port, baudrate)
    except serial.SerialException as e:
        print(f"[ERROR] Failed to connect: {e}")
        return False

    result = False

    try:
        total = len(segments)
        sent_idx = 0
        in_flight = 0
        CMD_QUEUE_LIMIT = 3

        print(f"\n[SEND] Streaming {total} segments to STM32...")
        print(f"[SEND] Window limit: {CMD_QUEUE_LIMIT}, flow control: 'K'(Ack) / 'N'(Nack) / 'D'(Done)\n")

        start_time = time.time()
        ctrl.ser.reset_input_buffer()

        while sent_idx < total or in_flight > 0:
            # 1. Read serial response
            if ctrl.ser.in_waiting > 0:
                rx_data = ctrl.ser.read(ctrl.ser.in_waiting)
                for byte in rx_data:
                    char = chr(byte)
                    if char == 'D':
                        in_flight = max(0, in_flight - 1)
                        print(f"  [Done] Segment completed. In-flight: {in_flight}")
                    elif char == 'K':
                        print(f"  [Ack] STM32 queued segment.")
                    elif char == 'N':
                        print(f"  [Nack] STM32 queue full, retrying...")
                        sent_idx -= 1
                        in_flight = max(0, in_flight - 1)
                        time.sleep(0.005)

            # 2. Send next move if buffer not full
            if sent_idx < total and in_flight < CMD_QUEUE_LIMIT:
                steps = segments[sent_idx]
                ctrl.send_move_async(steps)
                in_flight += 1
                sent_idx += 1
                if sent_idx % 100 == 0 or sent_idx == total:
                    elapsed = time.time() - start_time
                    print(f"Progress: {sent_idx}/{total} ({sent_idx/total*100:.0f}%) | In-flight: {in_flight}")

            time.sleep(0.001)

        elapsed = time.time() - start_time
        print(f"\n[SEND] ✓ Sent {sent_idx}/{total} segments in {elapsed:.2f}s")
        result = True

    except KeyboardInterrupt:
        print("\n\n[!] Interrupted by user")
        ctrl.send_estop()

    except serial.SerialException as e:
        print(f"[ERROR] Serial error: {e}")

    finally:
        ctrl.close()

    return result


# ════════════════════════════════════════════════════════════════════════
# SECTION 5 — TEST CONNECTION
# ════════════════════════════════════════════════════════════════════════

def test_connection(port: str, baudrate: int = DEFAULT_BAUD):
    """Quick test: send a single move command and check response."""
    print(f"[TEST] Testing connection to STM32 on {port}...")

    try:
        ctrl = StepperController(port, baudrate)
    except serial.SerialException as e:
        print(f"[TEST] ✗ Failed to open {port}: {e}")
        print(f"  → Check cable, permissions (try: sudo chmod 666 {port})")
        return False

    try:
        # Send a simple move
        test_steps = [1000, 500, 2000, 1500, 800, 300]
        print(f"[TEST] Sending move: {test_steps}")
        resp = ctrl.send_move(test_steps)
        print(f"[TEST] Response: {resp}  "
              f"({'✓ ACK' if resp == 'K' else '✗ unexpected'})")

        # Query status
        status = ctrl.get_status()
        print(f"[TEST] Status: {status}")

        print(f"\n[TEST] ✓ STM32 is alive and responding!")
        return True

    finally:
        ctrl.close()


# ════════════════════════════════════════════════════════════════════════
# MAIN
# ════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="CB1 Stepper Control — Trajectory generator & USB sender for STM32",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s generate                         # Generate trajectory.csv
  %(prog)s send trajectory.csv              # Send existing CSV to STM32
  %(prog)s all -c mytraj.csv                # Generate + send in one go
  %(prog)s test -p /dev/ttyACM0             # Quick connectivity test
  %(prog)s generate -c ramp_up.csv          # Custom CSV filename
        """
    )

    parser.add_argument(
        'action',
        nargs='?',
        choices=['generate', 'send', 'all', 'test'],
        default='all',
        help="Action: 'generate' CSV, 'send' to STM32, 'all' = both, 'test' connection"
    )
    parser.add_argument(
        '--csv', '-c',
        default=DEFAULT_CSV,
        help=f"CSV file path (default: {DEFAULT_CSV})"
    )
    parser.add_argument(
        '--port', '-p',
        default=DEFAULT_PORT,
        help=f"Serial port for STM32 (default: {DEFAULT_PORT})"
    )
    parser.add_argument(
        '--baud', '-b',
        type=int,
        default=DEFAULT_BAUD,
        help=f"Serial baud rate (default: {DEFAULT_BAUD})"
    )

    args = parser.parse_args()

    print("╔═══════════════════════════════════════════╗")
    print("║   CB1 — Stepper Motor Control for STM32  ║")
    print("╚═══════════════════════════════════════════╝")
    print()

    if args.action == 'generate':
        generate_trajectory_csv(args.csv)

    elif args.action == 'send':
        send_trajectory(args.port, args.csv, args.baud)

    elif args.action == 'all':
        generate_trajectory_csv(args.csv)
        print()
        send_trajectory(args.port, args.csv, args.baud)

    elif args.action == 'test':
        test_connection(args.port, args.baud)


if __name__ == '__main__':
    main()
