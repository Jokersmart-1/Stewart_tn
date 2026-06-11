# 04 — Giao Tiếp USB với Raspberry Pi

## 📡 Tổng Quan

STM32F446ZE hoạt động như **USB CDC Device** (Virtual COM Port).  
Raspberry Pi kết nối qua cáp USB → STM32 nhận lệnh điểm đến cho 6 trục.

```
Raspberry Pi                    STM32F446ZE
──────────                      ───────────
/dev/ttyACM0  ←── USB CDC ───→  PA11 (D−)
                                PA12 (D+)
                                USB OTG FS (Device Only)
```

---

## 🔌 Phần Cứng

| Tín hiệu | Pin STM32 | Ghi chú |
|----------|-----------|---------|
| USB D− | **PA11** | USB_OTG_FS_DM |
| USB D+ | **PA12** | USB_OTG_FS_DP |
| Mode | Device Only | STM32 là slave, Raspi là host |

**Cấu hình trong IOC:**
```
PA11.Signal = USB_OTG_FS_DM  (Device_Only mode)
PA12.Signal = USB_OTG_FS_DP  (Device_Only mode)
USB_OTG_FS.VirtualMode = Device_Only
```

---

## 📦 Giao Thức Lệnh (Command Protocol)

### Gói Lệnh — Raspberry Pi → STM32

```
┌──────┬──────────────────────────────────┬──────┐
│ SOF  │           PAYLOAD                │ EOL  │
│ 0xAA │ CMD [1] + DATA [N bytes]         │ 0x0A │
└──────┴──────────────────────────────────┴──────┘
```

### Lệnh Di Chuyển (CMD = 0x4D = 'M')

```
Byte:  0      1      2-3    4-5    6-7    8-9    10-11  12-13  14
       SOF    CMD    M0     M1     M2     M3     M4     M5     EOL
       0xAA   'M'   int16  int16  int16  int16  int16  int16  '\n'
```

- Mỗi `int16` = số bước (signed) cho 1 trục
- Dấu (+) = chiều forward, dấu (−) = chiều reverse
- Tổng packet: **15 bytes**

**Ví dụ:** Di chuyển M0=+1000, M1=+2000, M2=0, M3=+500, M4=-300, M5=+800

```
AA 4D 03E8 07D0 0000 01F4 FF D4 0320 0A
```

### Lệnh Dừng Khẩn Cấp (CMD = 0x45 = 'E')

```
AA  45  0A
```
→ Ngay lập tức tắt TIM2, reset tất cả trục

### Lệnh Truy Vấn Trạng Thái (CMD = 0x53 = 'S')

```
AA  53  0A
```

**STM32 trả lời:**
```
AA 53 [STATUS_BYTE] [STEPS_DONE × 6 × int32] 0A
```
- `STATUS_BYTE`: `0x01` = đang chạy, `0x00` = dừng/idle

---

## 🛠️ Cài Đặt USB CDC trên STM32CubeIDE

### Bước 1: Thêm USB Middleware trong CubeMX

```
Middleware → USB_DEVICE → Class = CDC (Virtual Com Port)
```

### Bước 2: Cấu Hình VCP Speed

Trong `usbd_cdc_if.c`:
```c
#define APP_RX_DATA_SIZE  64
#define APP_TX_DATA_SIZE  64
```

### Bước 3: Nhận Dữ Liệu

```c
// usbd_cdc_if.c — CDC_Receive_FS callback
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len) {
    // Buf chứa dữ liệu nhận được
    // Len = số byte
    
    CDC_Parse_Command(Buf, *Len);  // Gọi parser của mình
    
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return USBD_OK;
}
```

### Bước 4: Parse Lệnh

```c
void CDC_Parse_Command(uint8_t *buf, uint32_t len) {
    if (buf[0] != 0xAA) return;  // Kiểm tra SOF
    
    switch (buf[1]) {
        case 'M': {  // Move command
            if (len < 15) return;
            int16_t steps[6];
            for (int i = 0; i < 6; i++) {
                steps[i] = (int16_t)((buf[2 + i*2] << 8) | buf[3 + i*2]);
            }
            // Gọi DDA với target steps
            uint32_t abs_steps[6];
            for (int i = 0; i < 6; i++) {
                axes[i].dir = (steps[i] < 0) ? 1 : 0;
                abs_steps[i] = (steps[i] < 0) ? -steps[i] : steps[i];
                DDA_SetDirection(i, axes[i].dir);
            }
            DDA_SetTarget(abs_steps);
            DDA_Start();
            break;
        }
        case 'E': {  // Emergency Stop
            TIM2->CR1 &= ~TIM_CR1_CEN;
            simulation_running = 0;
            break;
        }
        case 'S': {  // Status query
            CDC_Send_Status();
            break;
        }
    }
}
```

### Bước 5: Gửi Phản Hồi

```c
void CDC_Send_Status(void) {
    uint8_t tx_buf[30];
    tx_buf[0] = 0xAA;
    tx_buf[1] = 'S';
    tx_buf[2] = simulation_running ? 0x01 : 0x00;
    // ... append steps_done cho 6 trục
    CDC_Transmit_FS(tx_buf, sizeof(tx_buf));
}
```

---

## 🐍 Script Python trên Raspberry Pi

```python
#!/usr/bin/env python3
"""
stepper_controller.py — Gửi lệnh di chuyển đến STM32 qua USB CDC
"""

import serial
import struct
import time

class StepperController:
    SOF = 0xAA
    EOL = 0x0A
    
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Chờ STM32 enumerate
    
    def move(self, steps: list[int]):
        """
        Gửi lệnh di chuyển 6 trục
        steps: list of 6 int16 values (negative = reverse)
        """
        assert len(steps) == 6
        packet = bytes([self.SOF, ord('M')])
        for s in steps:
            packet += struct.pack('>h', s)  # big-endian int16
        packet += bytes([self.EOL])
        self.ser.write(packet)
    
    def stop(self):
        """Dừng khẩn cấp"""
        self.ser.write(bytes([self.SOF, ord('E'), self.EOL]))
    
    def get_status(self):
        """Truy vấn trạng thái"""
        self.ser.write(bytes([self.SOF, ord('S'), self.EOL]))
        resp = self.ser.read(30)
        if len(resp) > 2 and resp[0] == self.SOF:
            return {'running': resp[2] == 1}
        return None
    
    def close(self):
        self.ser.close()


# Ví dụ sử dụng
if __name__ == '__main__':
    ctrl = StepperController('/dev/ttyACM0')
    
    # Di chuyển: M0=+1000, M1=+2000, M2=0, M3=+500, M4=-300, M5=+800
    ctrl.move([1000, 2000, 0, 500, -300, 800])
    
    # Chờ hoàn thành
    while True:
        status = ctrl.get_status()
        if status and not status['running']:
            break
        time.sleep(0.01)
    
    print("Đã đến đích!")
    ctrl.close()
```

---

## 🔍 Debug USB

### Kiểm tra trên Raspberry Pi

```bash
# Xem thiết bị USB
lsusb
# Nên thấy: ID 0483:5740 STMicroelectronics STM32 Virtual COM Port

# Xem cổng serial
ls /dev/ttyACM*
# Hoặc
ls /dev/ttyUSB*

# Test kết nối bằng minicom
minicom -D /dev/ttyACM0 -b 115200

# Kiểm tra log kernel
dmesg | grep -i usb | tail -20
```

### Troubleshooting

| Vấn đề | Nguyên nhân | Giải pháp |
|--------|-------------|-----------|
| Không thấy ttyACM0 | USB CDC chưa init | Kiểm tra `MX_USB_DEVICE_Init()` được gọi |
| Mất kết nối | Clock 48MHz sai | Kiểm tra PLLSAI → 48MHz cho USB |
| Dữ liệu sai | Endian nhầm | Đảm bảo big-endian nhất quán cả 2 phía |

---

## ⏳ Trạng Thái Phát Triển

- [x] Hardware PA11/PA12 đã cấu hình trong CubeMX
- [ ] Enable USB_DEVICE middleware trong CubeMX
- [ ] Implement CDC_Receive callback
- [ ] Implement Command Parser
- [ ] Test trên Raspberry Pi
