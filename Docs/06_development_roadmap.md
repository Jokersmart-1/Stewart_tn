# 06 — Lộ Trình Phát Triển (Development Roadmap)

## 🗺️ Tổng Quan Các Giai Đoạn

```
Phase 1 ──✅──► Phase 2 ──✅──► Phase 3 ──🔄──► Phase 4 ──⏳──► Phase 5 ──⏳──► Phase 6 ──⏳
Hardware      DDA 2-axis    DDA 6-axis    USB/Raspi     SD Update    Integration
Config        Test          Sync                        Bootloader   & Test
```

---

## ✅ Phase 1 — Cấu Hình Phần Cứng (HOÀN THÀNH)

**Mục tiêu:** Thiết lập môi trường phát triển và cấu hình phần cứng cơ bản.

### Đã Hoàn Thành
- [x] Tạo project STM32CubeIDE cho STM32F446ZETx (LQFP144)
- [x] Cấu hình HSE 12MHz → PLL → 180MHz SYSCLK
- [x] Cấu hình PLLSAI → 48MHz cho USB và SDIO
- [x] Định nghĩa tất cả GPIO: STEP0–STEP7, DIR0–DIR7, EN0–EN7
- [x] Cấu hình SDIO 4-bit wide bus (PC8–PC12, PD2)
- [x] Cấu hình USB OTG FS Device (PA11/PA12)
- [x] Cấu hình USART2 cho Raspi backup (PD5/PD6)
- [x] Cấu hình CAN1 (PD0/PD1) — dự phòng tương lai
- [x] End-stops: STOP_0 đến STOP_7 (PG6–PG15)
- [x] SPI1/SPI2/SPI3 cho driver chips
- [x] I2C1 cho EEPROM (PB8/PB9)
- [x] Fix SysTick recalibrate sau PLL config

### Files Chính
- [`Core/Inc/main.h`](../Core/Inc/main.h) — Tất cả #define GPIO
- [`Core/Src/gpio.c`](../Core/Src/gpio.c) — MX_GPIO_Init()
- [`final.ioc`](../final.ioc) — CubeMX config

---

## ✅ Phase 2 — Bắn Xung DDA 2 Trục (HOÀN THÀNH)

**Mục tiêu:** Chứng minh thuật toán DDA hoạt động trên phần cứng thực.

### Đã Hoàn Thành
- [x] Implement TIM2 @ 200kHz (PSC=89, ARR=4)
- [x] DDA accumulator cho STEP1 (PG0) — velocity=2000/4000 → 100kHz
- [x] DDA accumulator cho STEP2 (PF11) — velocity=3000/4000 → 150kHz
- [x] Pulse width control (state_timer = 1 tick = 5µs)
- [x] Auto-stop sau 4000 ngắt (20ms)
- [x] WORK_LED toggle để xác nhận completion
- [x] Test thành công trên oscilloscope

### Kết Quả Đo Được
| Tín hiệu | Tần số lý thuyết | Tần số đo được | Sai số |
|----------|-----------------|----------------|--------|
| STEP1 | 100 kHz | ~100 kHz | < 1% |
| STEP2 | 150 kHz | ~150 kHz | < 1% |

### Files Chính
- [`Core/Src/main.c`](../Core/Src/main.c) — TIM2_DDA_Callback(), MX_TIM2_Init()

---

## 🔄 Phase 3 — DDA 6 Trục Đồng Bộ (ĐANG PHÁT TRIỂN)

**Mục tiêu:** Mở rộng DDA cho cả 6 trục, tất cả đến đích cùng một lúc.

**Thời gian ước tính:** 1–2 tuần

### Việc Cần Làm

#### 3.1 Cấu Trúc Dữ Liệu
- [ ] Tạo `AxisConfig_t` struct (xem [`03_dda_algorithm.md`](03_dda_algorithm.md))
- [ ] Định nghĩa mảng `axes[6]` với port/pin của từng trục
- [ ] Định nghĩa `STEP_SCALE = 65536` (độ phân giải cao hơn)

#### 3.2 Hàm DDA
- [ ] Implement `DDA_SetTarget(uint32_t steps[6])`
- [ ] Implement `DDA_SetDirection(uint8_t axis, uint8_t dir)`
- [ ] Implement `DDA_Start()` và `DDA_Stop()`
- [ ] Refactor ISR callback để loop qua `axes[]`

#### 3.3 Tối Ưu ISR
- [ ] Đo thời gian thực thi ISR (mục tiêu: < 3µs cho 6 trục)
- [ ] Nếu quá chậm: dùng direct register write thay vì HAL_GPIO_WritePin
  ```c
  // Nhanh hơn HAL:
  GPIOG->BSRR = STEP1_Pin;        // SET
  GPIOG->BSRR = STEP1_Pin << 16; // RESET
  ```
- [ ] Cân nhắc unroll loop nếu cần tốc độ tối đa

#### 3.4 Test
- [ ] Test 6 trục với tốc độ khác nhau (xác nhận đồng bộ)
- [ ] Test chiều quay DIR cho từng trục
- [ ] Test end-condition (tất cả trục dừng khi xong)
- [ ] Đo actual step count vs target (sai số < 1 bước)

### Bản đồ STEP/DIR cho 6 Trục

```c
// Thứ tự trục: M0, M1, M2, M3, M4, M5
const GPIO_TypeDef* STEP_PORTS[] = {GPIOF, GPIOG, GPIOF, GPIOG, GPIOF, GPIOC};
const uint16_t      STEP_PINS[]  = {STEP0_Pin, STEP1_Pin, STEP2_Pin,
                                    STEP3_Pin, STEP4_Pin, STEP5_Pin};
const GPIO_TypeDef* DIR_PORTS[]  = {GPIOF, GPIOG, ?, GPIOG, GPIOF, GPIOF};
const uint16_t      DIR_PINS[]   = {DIR0_Pin, DIR1_Pin, ?, DIR2_Pin,
                                    DIR4_Pin, DIR5_Pin};
```

> ⚠️ **M2 DIR cần xác nhận** — Trong schematic, STEP2=PF11 nhưng DIR2=PG3 được assign cho M3 trong main.h. Cần review schematic gốc.

---

## ⏳ Phase 4 — Giao Tiếp USB với Raspberry Pi

**Mục tiêu:** STM32 nhận lệnh điểm đến từ Raspberry Pi qua USB CDC.

**Phụ thuộc:** Phase 3 phải hoàn thành trước.  
**Thời gian ước tính:** 1–2 tuần

### Việc Cần Làm

#### 4.1 USB CDC Setup
- [ ] Enable USB_DEVICE middleware trong CubeMX
- [ ] Generate code, thêm vào project
- [ ] Verify `MX_USB_DEVICE_Init()` hoạt động (LED xanh trên Raspi)

#### 4.2 Protocol Implementation
- [ ] Implement `CDC_Receive_FS` callback
- [ ] Implement Command Parser (xem [`04_usb_communication.md`](04_usb_communication.md))
- [ ] Implement Status Response
- [ ] Implement Emergency Stop

#### 4.3 Raspberry Pi Side
- [ ] Cài thư viện `pyserial` trên Raspi
- [ ] Viết `stepper_controller.py` (xem [`04_usb_communication.md`](04_usb_communication.md))
- [ ] Test gửi lệnh → xác nhận 6 trục di chuyển

#### 4.4 Robustness
- [ ] Timeout nếu không nhận packet đầy đủ trong 100ms
- [ ] Watchdog để reset nếu STM32 treo
- [ ] Buffer overflow protection

---

## ⏳ Phase 5 — Bootloader & Nạp Firmware qua MicroSD

**Mục tiêu:** Tạo bootloader đọc `firmware.bin` từ thẻ SD và nạp vào Flash.

**Thời gian ước tính:** 2–3 tuần

### Việc Cần Làm

#### 5.1 Tạo Project Bootloader Riêng
- [ ] Tạo STM32CubeIDE project mới: `final_bootloader`
- [ ] Linker origin: `0x08000000` (Sector 0–1, tối đa 32KB)
- [ ] Enable SDIO, FATFS, minimal GPIO

#### 5.2 FatFS Integration
- [ ] Thêm FatFS middleware (CubeMX hoặc thủ công)
- [ ] Implement `diskio.c` liên kết với HAL SD
- [ ] Test đọc file từ thẻ SD

#### 5.3 Flash Programming
- [ ] Implement Erase Sectors 2–7
- [ ] Implement Write từng word vào Flash
- [ ] Implement CRC32 verification

#### 5.4 Application Project
- [ ] Thay đổi linker script: origin = `0x08008000`
- [ ] Thêm `SCB->VTOR = 0x08008000` vào đầu SystemInit

#### 5.5 Test
- [ ] Nạp bootloader qua ST-Link (lần đầu)
- [ ] Đặt `firmware.bin` vào thẻ SD
- [ ] Reset → xác nhận Application khởi động
- [ ] Test update firmware mới

---

## ⏳ Phase 6 — Tích Hợp & Kiểm Tra Toàn Hệ Thống

**Mục tiêu:** Kết hợp tất cả giai đoạn, kiểm tra toàn bộ hệ thống.

**Thời gian ước tính:** 1–2 tuần

### Việc Cần Làm

#### 6.1 Tích Hợp
- [ ] Kết hợp Phase 3 (DDA 6 trục) + Phase 4 (USB) + Phase 5 (SD boot)
- [ ] Kiểm tra không có conflict giữa TIM2 ISR và USB interrupt
- [ ] Đảm bảo priority NVIC đúng:
  - TIM2: Priority 0 (highest — timing critical)
  - USB: Priority 1
  - USART: Priority 2

#### 6.2 Test Hệ Thống
- [ ] Test full flow: Raspi gửi lệnh → STM32 di chuyển 6 trục → báo hoàn thành
- [ ] Test emergency stop từ Raspi
- [ ] Test nạp firmware mới qua SD trong khi không có kết nối Raspi
- [ ] Test restart sau update: Raspi kết nối lại bình thường

#### 6.3 Reliability
- [ ] Chạy liên tục 1 giờ không lỗi
- [ ] Test nguồn điện: tắt/bật đột ngột
- [ ] Test thẻ SD bị tháo khi đang chạy (không crash)

#### 6.4 Documentation
- [ ] Cập nhật README với hướng dẫn đầy đủ
- [ ] Ghi lại pinout cuối cùng
- [ ] Video demo hệ thống

---

## 📊 Tóm Tắt Timeline

| Phase | Mô tả | Thời gian | Trạng thái |
|-------|-------|-----------|------------|
| 1 | Hardware Config | 1 tuần | ✅ Hoàn thành |
| 2 | DDA 2-axis Test | 3 ngày | ✅ Hoàn thành |
| 3 | DDA 6-axis Sync | 1–2 tuần | 🔄 Đang làm |
| 4 | USB/Raspi Comms | 1–2 tuần | ⏳ Chưa bắt đầu |
| 5 | SD Bootloader | 2–3 tuần | ⏳ Chưa bắt đầu |
| 6 | Integration | 1–2 tuần | ⏳ Chưa bắt đầu |
| **Tổng** | | **~7–10 tuần** | |

---

## 🔮 Tính Năng Tương Lai (Backlog)

| Tính năng | Mô tả | Độ ưu tiên |
|-----------|-------|-----------|
| Acceleration ramp | Tăng/giảm tốc dạng trapezoid/S-curve | Cao |
| Position feedback | Đọc encoder, closed-loop | Trung bình |
| Homing sequence | Tự động về home dùng end-stops | Cao |
| G-code parser | Nhận G-code từ Raspi thay vì raw steps | Thấp |
| CAN bus | Điều khiển thêm module qua CAN1 | Thấp |
| Web interface | Raspi chạy web server điều khiển | Thấp |
| EEPROM config | Lưu config (steps/mm, speed) vào I2C EEPROM | Trung bình |
