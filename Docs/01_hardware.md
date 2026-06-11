# 01 — Phần Cứng & Sơ Đồ Chân

## MCU: STM32F446ZETx

| Thông số | Giá trị |
|----------|---------|
| Package | LQFP144 |
| Flash | 512 KB |
| RAM | 128 KB |
| Max Clock | 180 MHz |
| Firmware package | STM32Cube FW_F4 V1.28.3 |

---

## 🔌 Sơ Đồ Chân — 6 Trục Motor Bước (STEP / DIR / EN)

| Trục | STEP | Port | DIR | Port | EN | Port | CS (SPI) | Port |
|------|------|------|-----|------|----|------|----------|------|
| **M0** | PF13 | GPIOF | PF12 | GPIOF | PF14 | GPIOF | PC4 (CS0) | GPIOC |
| **M1** | PG0  | GPIOG | PG1  | GPIOG | PF15 | GPIOF | PD11 (CS1) | GPIOD |
| **M2** | PF11 | GPIOF | —   | —    | —  | —    | PC6 (CS2) | GPIOC |
| **M3** | PG4  | GPIOG | PG3  | GPIOG | PG5 | GPIOG | PC7 (CS3) | GPIOC |
| **M4** | PF9  | GPIOF | PF10 | GPIOF | PG2 | GPIOG | PF2 (CS4) | GPIOF |
| **M5** | PC13 | GPIOC | PF0  | GPIOF | PF1 | GPIOF | PE4 (CS5) | GPIOE |

> **Lưu ý:** M2 (STEP2) = PF11, chưa có EN riêng trong cấu hình hiện tại. Cần bổ sung hoặc dùng chung EN.

---

## 🔵 Test Đã Hoàn Thành (Phase 2)

| Tín hiệu | Pin | Kết quả |
|----------|-----|---------|
| **STEP1** | PG0 (GPIOG Pin0) | ✅ Bắn xung thành công @ 100kHz |
| **STEP2** | PF11 (GPIOF Pin11) | ✅ Bắn xung thành công @ 150kHz |

---

## 📡 Giao Tiếp Ngoại Vi

### USB OTG FS — Nhận lệnh từ Raspberry Pi
| Tín hiệu | Pin | Chức năng |
|----------|-----|-----------|
| USB D- | PA11 | USB OTG FS DM (Device Only) |
| USB D+ | PA12 | USB OTG FS DP (Device Only) |

### SDIO — MicroSD (4-bit wide bus)
| Tín hiệu | Pin |
|----------|-----|
| SDIO_D0 | PC8 |
| SDIO_D1 | PC9 |
| SDIO_D2 | PC10 |
| SDIO_D3 | PC11 |
| SDIO_CK | PC12 |
| SDIO_CMD | PD2 |
| Card Detect | PC14 (DET) |

### USART2 — Kênh dự phòng Raspberry Pi (nếu dùng Serial thay USB)
| Tín hiệu | Pin |
|----------|-----|
| RAS_TX (STM32→Raspi) | PD5 |
| RAS_RX (Raspi→STM32) | PD6 |

### CAN1 — Mở rộng tương lai
| Tín hiệu | Pin |
|----------|-----|
| CAN1_TX | PD1 |
| CAN1_RX | PD0 |
| Baud rate | 1 Mbit/s |

### SPI — Giao tiếp Driver (TMC/A4988/DRV8825)
| Bus | MOSI | MISO | SCK | Dùng cho |
|-----|------|------|-----|---------|
| SPI1 | PA7 | PA6 | PA5 | Driver CS0–CS3 |
| SPI2 | PC3 | PC2 | PB13 | Driver CS4–CS5 |
| SPI3 | PB5 | PB4 | PB3 | Driver CS6–CS7 |

### I2C1 — EEPROM
| Tín hiệu | Pin |
|----------|-----|
| EE_SCL | PB8 |
| EE_SDA | PB9 |

---

## 🛑 End-stops (Cảm Biến Giới Hạn)

| Tên | Pin |
|-----|-----|
| STOP_0 | PG6 |
| STOP_1 | PG9 |
| STOP_2 | PG10 |
| STOP_3 | PG11 |
| STOP_4 | PG12 |
| STOP_5 | PG13 |
| STOP_6 | PG14 |
| STOP_7 | PG15 |

---

## 💡 Tín Hiệu Khác

| Nhãn | Pin | Chức năng |
|------|-----|-----------|
| WORK_LED | PA13 | LED trạng thái firmware |
| PWR_DET | PC0 | Phát hiện nguồn |
| KILL | PC5 | Ngắt khẩn cấp |
| PS_ON | PE11 | Bật nguồn PSU |
| BED_OUT | PA1 | Heated bed output |
| HE0–HE3 | PA2,PA3,PB10,PB11 | Heater 0–3 |
| FAN0–FAN5 | PA8,PE5,PD12–PD15 | Quạt làm mát |
| RGB | PB0 | LED RGB |
| BLTOUCH | PB6 | BLTouch servo |
| BLTRIGG | PB7 | BLTouch trigger |

---

## ⚡ Nguồn & Điện Áp

| Rail | Giá trị |
|------|---------|
| VDD MCU | 3.3V |
| Motor Driver | 12V / 24V (tùy driver) |
| USB | 5V từ Raspberry Pi |
| MicroSD | 3.3V |
