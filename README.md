# 🤖 6-Axis Stepper Motor Controller Firmware (STM32F446ZE)

Dự án này chứa mã nguồn firmware (ngôn ngữ C) chạy trên vi điều khiển **STM32F446ZETx** để điều khiển đồng bộ 6 động cơ bước (M0–M5) theo thời gian thực. Hệ thống nhận lệnh di chuyển từ máy tính điều khiển (Host) qua kết nối USB CDC và tự động phát xung với tần số lên tới 200 kHz nhờ sự kết hợp giữa thuật toán DDA, cơ chế bộ đệm kép (Double-Buffering) và đồng bộ hóa phần cứng DMA - Timer.

---

## 📂 1. Cấu Trúc Thư Mục Dự Án (C Firmware)

```
├── Core/
│   ├── Src/
│   │   ├── main.c              # Khởi tạo hệ thống, clock, GPIO, DMA, Timer và vòng lặp chính
│   │   ├── main_pingpong.c     # Trình điều khiển DDA 6 trục, hàng đợi chuyển động & bộ đệm kép
│   │   ├── stm32f4xx_it.c      # Trình phục vụ ngắt (ISR), xử lý hoán đổi bộ đệm DMA
│   │   ├── gpio.c              # Cấu hình các chân GPIO vật lý
│   │   ├── sdio.c              # Trình điều khiển thẻ nhớ MicroSD qua giao tiếp SDIO
│   │   └── usb_otg.c           # Cấu hình phần cứng USB On-The-Go Full Speed
│   └── Inc/
│       ├── main.h              # Định nghĩa chân GPIO, cấu trúc dữ liệu AxisConfig_t
│       └── ...
├── USB_DEVICE/                 # Ngăn xếp (Stack) USB Device cho cổng Virtual COM Port (VCP)
│   ├── App/
│   │   ├── usb_device.c        # Khởi tạo thiết bị USB
│   │   └── usbd_cdc_if.c       # Giao tiếp truyền nhận dữ liệu USB CDC
│   └── Target/
├── Docs/                       # Tài liệu thiết kế kỹ thuật chi tiết
│   ├── 01_hardware.md          # Sơ đồ chân, đấu nối Driver động cơ bước
│   ├── 02_clock_timer.md       # Cấu hình Clock 180MHz & Bộ tạo xung Timer
│   ├── 03_dda_algorithm.md     # Chi tiết thuật toán DDA đồng bộ 6 trục
│   ├── 04_usb_communication.md # Giao thức truyền nhận lệnh USB CDC
│   └── 05_sdcard_firmware_update.md # Hướng dẫn cập nhật firmware qua thẻ nhớ
└── archi.md                    # Báo cáo giải thuật DDA & Sơ đồ logic (dành cho người đọc phổ thông)
```

*(Lưu ý: Các file kịch bản hỗ trợ bên ngoài như các script Python `.py` nằm ở thư mục gốc chỉ dùng cho mục đích giả lập quỹ đạo hoặc kiểm thử giao tiếp, không thuộc thành phần mã nguồn nạp vào vi điều khiển).*

---

## 🛠️ 2. Kiến Trúc Phần Cứng & Cấu Hình Vi Điều Khiển

### Cấu hình Clock hệ thống (HCLK = 180 MHz)
* **Nguồn cấp:** Thạch anh ngoài (HSE) 12 MHz.
* **SYSCLK / AHB:** 180 MHz (sử dụng mạch PLL và chế độ Over-Drive để đạt hiệu năng tối đa).
* **APB1 Clock:** 45 MHz (Timer x2 = 90 MHz).
* **APB2 Clock:** 90 MHz (Timer x2 = 180 MHz).
* **Clock Ngoại vi đặc biệt:** Mạch PLLSAI cấu hình để tạo ra clock 48 MHz chuẩn xác cấp cho bộ điều khiển USB OTG FS và SDIO.

### Sơ đồ ánh xạ chân điều khiển (Pinout Mapping)
Các chân phát xung (STEP), chiều quay (DIR) và cho phép động cơ (EN) của 6 trục được bố trí tập trung trên các cổng GPIO tốc độ cao nhằm hỗ trợ ghi thanh ghi trực tiếp bằng DMA:

| Động cơ | Chức năng (STEP / DIR / EN) | Chân GPIO vật lý | Cổng ngoại vi (GPIO Port) |
| :---: | :---: | :---: | :---: |
| **M0** | STEP0 / DIR0 / EN0 | PF13 / PF12 / PF14 | **Port F** |
| **M1** | STEP1 / DIR1 / EN1 | PG0 / PG1 / PF15 | **Port G / Port F** |
| **M2** | STEP2 / DIR2 / EN2 | PF11 / PG3 / PG5 | **Port F / Port G** |
| **M3** | STEP3 / DIR3 / EN3 | PG4 / PC1 / PA0 | **Port G / Port C / Port A** |
| **M4** | STEP4 / DIR4 / EN4 | PF9 / PF10 / PG2 | **Port F / Port G** |
| **M5** | STEP5 / DIR5 / EN5 | PC13 / PF0 / PF1 | **Port C / Port F** |

---

## ⚙️ 3. Phương Pháp Điều Khiển Lõi

Firmware tích hợp ba cơ chế cốt lõi để đảm bảo chuyển động của 6 trục luôn mượt mà và đồng bộ:

### A. Thuật toán DDA (Digital Differential Analyzer)
Hệ thống sử dụng thuật toán DDA để điều khiển đồng bộ đa trục. Mọi phép toán trong vòng lặp thời gian thực được tối ưu hóa chỉ dùng phép cộng và phép so sánh số nguyên trên bộ tích lũy (accumulator), loại bỏ hoàn toàn các phép chia số thực vốn rất chậm. Chi tiết giải thuật và sơ đồ khối được trình bày riêng tại file [archi.md](file:///d:/final/archi.md).

### B. Cơ chế Bộ đệm kép Ping-Pong (Double-Buffering)
* **Giới hạn RAM:** Lưu trữ toàn bộ chuỗi xung cho lệnh 20ms (4000 ticks ở tần số 200kHz) của 6 trục sẽ gây tràn bộ nhớ 128KB SRAM của chip.
* **Giải pháp:** Phân chia nhỏ chuyển động thành các phân đoạn 5ms (1000 ticks DDA). Hệ thống sử dụng hai bộ đệm song song (Buffer 0 và Buffer 1) lưu trữ trạng thái cổng GPIO dạng thanh ghi BSRR.
* **Vận hành:** Trong khi DMA đang tự động phát xung từ Buffer 0 ra các chân động cơ, CPU sẽ tính toán trước trạng thái xung cho phân đoạn tiếp theo và điền vào Buffer 1. Khi phát hết 5ms, ngắt DMA hoán đổi tức thì (độ trễ 0ms) để phát tiếp Buffer 1, CPU quay lại nạp Buffer 0.

### C. Đồng bộ hóa DMA - Timer 1 (Zero CPU Overhead)
* **Timer 1** đóng vai trò làm clock nhịp đập 200 kHz (5 µs mỗi chu kỳ) để kích hoạt DMA.
* **DMA2 (Stream 5, 1, 2)** tự động lấy dữ liệu từ RAM ghi thẳng vào thanh ghi BSRR của Port F, Port G và Port C mà không cần CPU xử lý ngắt từng xung.
* **Chống nhiễu nguồn:** Các luồng DMA kích hoạt lệch pha nhau một khoảng cực nhỏ ($0.5\mu\text{s}$ và $1.0\mu\text{s}$ trễ bằng các kênh Compare CCR1/CCR2 của Timer 1) nhằm phân tán dòng điện tức thời khi bật/tắt GPIO.
* **Đồng bộ pha khởi động:** Trước khi bật DMA, Timer 1 được reset cứng bộ đếm (`CNT = 0`) và xóa cờ ngắt (`SR = 0`) để đảm bảo các luồng DMA xuất phát đồng pha tuyệt đối.

---

## 📡 4. Giao Thức Giao Tiếp USB CDC

Hệ thống hoạt động dưới dạng thiết bị USB Virtual COM Port (VCP), giao tiếp với máy chủ Host qua cấu trúc gói tin chuẩn hóa:

### Định dạng gói tin (Host -> STM32):
```
┌──────┬──────────────┬────────────────────────┬──────┐
│ SOF  │ Lệnh (1 byte)│     Dữ liệu Payload    │ EOL  │
│ 0xAA │  'M' / 'E'   │        N bytes         │ 0x0A │
└──────┴──────────────┴────────────────────────┴──────┘
```

1. **Lệnh di chuyển (`'M'` - Move):**
   * **Payload:** 6 số nguyên 16-bit có dấu (`int16_t` dạng Big-Endian) tương ứng với số bước cần đi cho M0–M5. Dấu của số bước quy định chiều quay.
   * **Độ dài gói:** 15 bytes.
   * **Tín hiệu phản hồi:** Trả về `'K'` (Ack) nếu xếp vào hàng đợi thành công, trả về `'N'` (Nack) nếu hàng đợi đầy. Gửi `'D'` (Done) sau khi hoàn thành phát hết 20ms của lệnh đó.
2. **Lệnh dừng khẩn cấp (`'E'` - Emergency Stop):**
   * **Độ dài gói:** 3 bytes (`AA 45 0A`).
   * **Xử lý:** Dừng ngay Timer 1, tắt tất cả các luồng DMA, xóa sạch hàng đợi lệnh và khóa cứng toàn bộ chân STEP ở mức HIGH để dừng động cơ an toàn.
3. **Lệnh truy vấn trạng thái (`'S'` - Status Query):**
   * **Độ dài gói:** 3 bytes (`AA 53 0A`).
   * **Phản hồi:** Trả về gói dữ liệu chứa trạng thái đang chạy (0x01) hoặc dừng (0x00) kèm số bước thực tế đã đi được của cả 6 trục (dạng số nguyên 32-bit `uint32_t`).

---

## 💾 5. Cập Nhật Firmware Qua Thẻ Nhớ (Bootloader)

Firmware hỗ trợ tính năng cập nhật chương trình ứng dụng tự động bằng thẻ nhớ MicroSD thông qua giao tiếp SDIO 4-bit bus:

1. **Phân vùng bộ nhớ Flash (512 KB):**
   * **Bootloader (Sector 0-1 - 32 KB):** Nằm tại địa chỉ bắt đầu `0x08000000`. Nhiệm vụ kiểm tra thẻ nhớ, đọc ghi file và nhảy tới chương trình chính.
   * **Application (Sector 2-7 - 448 KB):** Nằm tại địa chỉ `0x08008000`. Đây là nơi chứa chương trình điều khiển động cơ bước chính.
2. **Quy trình nạp:**
   * Build chương trình ứng dụng và xuất file nhị phân đặt tên là `firmware.bin`.
   * Copy file `firmware.bin` vào thư mục gốc của thẻ MicroSD (định dạng FAT32).
   * Cắm thẻ vào bo mạch và nhấn nút Reset.
   * Bootloader tự phát hiện thẻ (chân PC14), tiến hành xóa các Sector từ 2 đến 7 trên Flash, copy nội dung file vào vùng Application và thực hiện kiểm tra tính toàn vẹn (CRC).
   * Sau khi hoàn tất, Bootloader cấu hình lại Vector Table và nhảy tới địa chỉ ứng dụng `0x08008000` để chạy chương trình mới.

---

## 🚦 6. Chế Độ Hoạt Động (Standalone Test Mode)

Để hỗ trợ kiểm tra mạch và động cơ tại thực địa khi không có máy tính kết nối:
* Nếu sau **10 giây kể từ khi khởi động** mạch không nhận được bất kỳ dữ liệu nào từ cổng USB, vi điều khiển tự động chuyển sang **Test Mode** (Standalone). Đèn LED trạng thái trên chân PA13 sẽ tắt.
* Ở chế độ này, vi điều khiển tự động nạp chu kỳ di chuyển mô phỏng cho 6 trục (phát xung di chuyển liên tục, nghỉ 1 giây rồi lặp lại).
* Ngay khi máy tính kết nối lại và gửi gói dữ liệu bất kỳ qua USB, hệ thống sẽ lập tức thoát khỏi Test Mode bằng cơ chế ngắt ưu tiên, xóa hàng đợi và chuyển tức thì sang **USB Streaming Mode** (đèn LED trạng thái sáng cố định).
