# 🤖 6-Axis Stepper Motor Controller — STM32F446ZE

**Firmware cho bo mạch tùy chỉnh STM32F446ZETx**  
Điều khiển **6 động cơ bước (Step/Dir)** đồng bộ, nhận điểm đến từ Raspberry Pi qua USB Serial (PA11/PA12 — USB OTG FS).  
Nạp firmware qua **MicroSD card** (DFU/bootloader).

---

## 📋 Tổng Quan Hệ Thống

```
┌─────────────────┐     USB Serial      ┌────────────────────────┐
│  Raspberry Pi   │ ──────────────────► │  STM32F446ZE Motherboard│
│  (Host/Master)  │   PA11(D-) PA12(D+) │                        │
└─────────────────┘                     │  6x Step/Dir Output    │
                                        │  TIM2 DDA @ 200kHz     │
                                        │  SDIO MicroSD          │
                                        └────────────────────────┘
                                               │  │  │  │  │  │
                                            M0 M1 M2 M3 M4 M5
                                          (6 stepper motor drivers)
```

---

## 📁 Cấu Trúc Tài Liệu

| File | Nội dung |
|------|----------|
| [`01_hardware.md`](01_hardware.md) | Sơ đồ chân, phần cứng, motor drivers |
| [`02_clock_timer.md`](02_clock_timer.md) | Cấu hình clock 180MHz, TIM2 DDA 200kHz |
| [`03_dda_algorithm.md`](03_dda_algorithm.md) | Thuật toán DDA, bộ tích lũy bước, đồng bộ 6 trục |
| [`04_usb_communication.md`](04_usb_communication.md) | Giao tiếp USB OTG FS với Raspberry Pi |
| [`05_sdcard_firmware_update.md`](05_sdcard_firmware_update.md) | Nạp firmware qua MicroSD (SDIO) |
| [`06_development_roadmap.md`](06_development_roadmap.md) | Lộ trình phát triển, các giai đoạn |

---

## ✅ Trạng Thái Hiện Tại

| Giai đoạn | Mô tả | Trạng thái |
|-----------|-------|-----------|
| **Phase 1** | Cấu hình phần cứng, GPIO, Clock | ✅ Hoàn thành |
| **Phase 2** | TIM2 DDA — test bắn xung STEP1, STEP2 | ✅ Hoàn thành |
| **Phase 3** | Mở rộng DDA cho 6 trục đồng thời | 🔄 Đang phát triển |
| **Phase 4** | USB Serial — nhận lệnh từ Raspberry Pi | ⏳ Chưa bắt đầu |
| **Phase 5** | SDIO — nạp firmware qua MicroSD | ⏳ Chưa bắt đầu |
| **Phase 6** | Tích hợp & kiểm tra toàn hệ thống | ⏳ Chưa bắt đầu |

---

## 🔧 Thông Số Kỹ Thuật

| Thông số | Giá trị |
|----------|---------|
| **MCU** | STM32F446ZETx (LQFP144) |
| **Clock** | 180 MHz (HSE 12MHz × PLL) |
| **Timer DDA** | TIM2 @ 200 kHz interrupt |
| **Số trục** | 6 (STEP0–STEP5) |
| **Giao tiếp Raspi** | USB OTG FS (PA11/PA12) |
| **Nạp firmware** | SDIO MicroSD (4-bit wide bus) |
| **Step/Dir Output** | GPIO Push-Pull |

---

## 🚀 Bắt Đầu Nhanh

1. **Clone / mở project** trong STM32CubeIDE
2. **Build** project (`Ctrl+B`)
3. **Nạp firmware** qua MicroSD (xem [`05_sdcard_firmware_update.md`](05_sdcard_firmware_update.md))
4. **Kết nối Raspberry Pi** qua cáp USB vào cổng PA11/PA12
5. **Gửi lệnh** điểm đến theo giao thức trong [`04_usb_communication.md`](04_usb_communication.md)

---

## 📎 Liên Kết Nhanh — Source Code

- [`Core/Src/main.c`](../Core/Src/main.c) — Vòng lặp chính, TIM2 DDA callback
- [`Core/Inc/main.h`](../Core/Inc/main.h) — Định nghĩa tất cả chân GPIO
- [`Core/Src/gpio.c`](../Core/Src/gpio.c) — Khởi tạo GPIO
- [`Core/Src/sdio.c`](../Core/Src/sdio.c) — Giao tiếp SDIO MicroSD
- [`Core/Src/usb_otg.c`](../Core/Src/usb_otg.c) — USB OTG FS
- [`final.ioc`](../final.ioc) — STM32CubeMX project config
