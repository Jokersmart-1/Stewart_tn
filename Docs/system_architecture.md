# Cấu trúc Kiến trúc Hệ thống (System Architecture)

Tài liệu này mô tả chi tiết kiến trúc phần mềm và phần cứng của bộ điều khiển động cơ bước 6 trục đồng bộ sử dụng vi điều khiển **STM32F446ZETx**, tập trung vào cơ chế bộ đệm kép (Double-Buffering Ping-Pong) và giao tiếp thời gian thực qua USB CDC.

---

## 1. Sơ đồ khối tổng thể (Block Diagram)

Hệ thống được thiết kế theo mô hình **Master-Slave** thời gian thực giữa máy tính điều khiển (Raspberry Pi/PC) và mạch STM32:

```
┌─────────────────┐     USB Serial      ┌────────────────────────┐
│  Raspberry Pi   │ ──────────────────► │  STM32F446ZE Motherboard│
│  (Host/Master)  │   PA11(D-) PA12(D+) │                        │
│                 │                     │  6x Step/Dir Output    │
│                 │                     │  TIM1 DDA @ 200kHz     │
│                 │                     │  Double-Buffering DMA  │
└─────────────────┘                     └────────────────────────┘
                                               │  │  │  │  │  │
                                            M0 M1 M2 M3 M4 M5
                                          (6 stepper motor drivers)
```

### Luồng xử lý chi tiết:
1. **Host PC / Raspberry Pi** truyền lệnh di chuyển 20 ms dưới dạng mảng xung (ví dụ: `[1800, 2000, 3000, 2000, 500, 0]`) qua cổng USB CDC VCP.
2. **USB CDC Interrupt** tiếp nhận gói tin thô, kích hoạt ngắt giao tiếp và chuyển tới Parser để giải mã lệnh.
3. Lệnh giải mã thành công được đẩy vào hàng đợi chuyển động (`cmd_queue`).
4. Bộ máy DDA phân chia nhỏ lệnh 20 ms thành **bốn phân đoạn 5 ms** (1000 ticks) để tiết kiệm RAM.
5. CPU tính toán trước trạng thái chân GPIO lưu vào bộ đệm phụ (Buffer B) trong khi DMA đang tự động phát bộ đệm chính (Buffer A) ra các cổng ngoại vi.
6. Mỗi nhịp đếm của **Timer 1 (200kHz)** sẽ trực tiếp kích hoạt DMA truyền dữ liệu ghi đè lên các thanh ghi GPIO BSRR (cổng F, G, C) mà không cần can thiệp của CPU.
7. Khi phát hết 1 phân đoạn 5 ms, ngắt DMA Transfer Complete hoán đổi tức thì địa chỉ đệm (swap buffer) với độ trễ 0 ms.
8. Khi hoàn thành toàn bộ 4 phân đoạn (đủ 20 ms), ngắt báo cho ứng dụng để gửi tín hiệu `'D'` (Done) về PC nhằm nạp phân đoạn tiếp theo.

---

## 2. Cơ chế Bộ đệm kép Ping-Pong (Double-Buffering)

### Ràng buộc bộ nhớ (SRAM Constraint)
* **Vi điều khiển STM32F446ZE** chỉ có **128 KB SRAM**.
* Một lệnh di chuyển đầy đủ kéo dài **20 ms (4000 ticks DDA)**.
* Nếu lưu trữ cả 3 cổng GPIO (F, G, C) cho 4000 ticks DDA ở dạng bộ đệm kép, ta cần: 
  $$\text{SRAM} = 4000 \times 2 \times 4 \text{ bytes} \times 3 \text{ ports} \times 2 \text{ buffers} = 192 \text{ KB}$$
  Mức này sẽ làm tràn SRAM lập tức.
* **Giải pháp**: Sử dụng cơ chế chia nhỏ phân đoạn (Internal Sub-segmenting). Bộ đệm kép được cấu hình ở kích thước **5 ms (1000 ticks DDA)**:
  $$\text{RAM sử dụng} = 1000 \times 2 \times 4 \times 3 \times 2 = 48 \text{ KB}$$
  Cấu hình này chiếm **37.5% SRAM**, đảm bảo an toàn tuyệt đối cho stack và heap của hệ thống.

### Phân đoạn lệnh di chuyển (Sub-segmenting Flow)
Khi host gửi lệnh di chuyển 20 ms dưới dạng mảng xung cho 6 trục (ví dụ: `[1800, 2000, 3000, 2000, 500, 0]`), hệ thống thực hiện:
1. Đọc và xếp hàng vào `cmd_queue` (kích thước `CMD_QUEUE_SIZE = 4`).
2. Chia nhỏ lệnh 20 ms thành **bốn phân đoạn 5 ms**.
3. Bộ tích lũy DDA (`accum`) và bộ đếm thời gian (`state_timer`) được lưu trữ liên tục để giữ nguyên pha DDA giữa các phân đoạn 5 ms, đảm bảo tần số xung ra không bị méo.
4. Khi Buffer 0 đang được DMA phát (phân đoạn 1), CPU tính toán trước Buffer 1 (phân đoạn 2).
5. Khi DMA phát hết Buffer 0, ngắt Transfer Complete sẽ hoán đổi địa chỉ phát sang Buffer 1 ngay lập tức với **độ trễ 0 ms**.
6. Ký tự báo hoàn thành lệnh di chuyển (`'D'`) chỉ được gửi ngược lại PC sau khi phát xong phân đoạn 5 ms thứ 4 (đủ 20 ms).

---

## 3. Bản đồ cổng GPIO và Drivers (Pinout Mapping)

Các chân STEP, DIR, EN của 6 trục được định cấu hình trực tiếp trên các cổng GPIO tốc độ cao nhằm hỗ trợ ghi thanh ghi BSRR trực tiếp qua DMA:

| Động cơ (Axis) | Chức năng | Chân GPIO (Pin) | Cổng ngoại vi (GPIO Port) |
|---|---|---|---|
| **M0** (Trục 0) | STEP0 / DIR0 / EN0 | PF13 / PF12 / PF14 | **Port F** |
| **M1** (Trục 1) | STEP1 / DIR1 / EN1 | PG0 / PG1 / PF15 | **Port G / Port F** |
| **M2** (Trục 2) | STEP2 / DIR2 / EN2 | PF11 / PG3 / PG5 | **Port F / Port G** |
| **M3** (Trục 3) | STEP3 / DIR3 / EN3 | PG4 / PC1 / PA0 | **Port G / Port C / Port A** |
| **M4** (Trục 4) | STEP4 / DIR4 / EN4 | PF9 / PF10 / PG2 | **Port F / Port G** |
| **M5** (Trục 5) | STEP5 / DIR5 / EN5 | PC13 / PF0 / PF1 | **Port C / Port F** |

> 💡 **Logic Xung STEP (Active-Low)**: Do mạch sử dụng bộ đệm đảo (Inverting Hardware Buffer), chân STEP trên MCU được cấu hình mặc định là **HIGH (mức cao)**. Khi phát xung, chân STEP sẽ kéo xuống **LOW (mức thấp)** để bắt đầu sườn lên ở driver bên ngoài, sau đó kéo lại lên **HIGH** để kết thúc xung.

---

## 4. Chế độ Hoạt động & Ưu tiên Phản ứng (Preemption)

Hệ thống có hai chế độ hoạt động chính:

### Chế độ USB Streaming Mode (`usb_mode = 1`)
* Hệ thống giao tiếp liên tục với PC. Đèn `WORK_LED` sáng cố định.
* Các lệnh di chuyển được nạp nối tiếp thông qua USB CDC VCP.

### Chế độ Test Mode (`usb_mode = 0`)
* Kích hoạt nếu sau **10 giây kể từ khi khởi động** không nhận được bất kỳ dữ liệu nào từ USB. Đèn `WORK_LED` sẽ tắt.
* MCU tự động nạp lệnh di chuyển mô phỏng `{1800, 2000, 3000, 2000, 500, 0}`, phát xung 20ms thông qua bộ đệm kép, dừng 1 giây và lặp lại chu kỳ.

### Cơ chế Ngắt ưu tiên (Pre-emption)
Nếu hệ thống đang chạy ở **Test Mode** (hoặc đang trễ 1 giây giữa các chu kỳ) mà nhận được lệnh bất kỳ từ USB:
1. Ngắt USB lập tức dừng TIM1 và vô hiệu hóa các luồng DMA hiện tại.
2. Thiết lập lại trạng thái hệ thống: tắt chế độ Test Mode, gán `usb_mode = 1`, `usb_active = 1`.
3. Giải phóng hàng đợi lệnh cũ và chuẩn bị nạp lệnh trực tiếp từ USB.
4. Đèn `WORK_LED` sáng trở lại, đánh dấu chuyển đổi tức thời sang **USB Streaming Mode** mà không cần khởi động lại mạch.

---

## 5. Cấu hình Ngắt NVIC (Interrupt Priority Scheme)

Để tránh hiện tượng trễ xung cơ học và nhiễu giao tiếp khi truyền tốc độ cao, các mức ưu tiên ngắt được định nghĩa như sau:

1. **DMA2 Stream 5 Interrupt (TCIF5)**: **Priority 0** (Mức ưu tiên cao nhất, quản lý nạp địa chỉ swap buffer kép lập tức để tránh mất bước).
2. **TIM1 Trigger/Compare Interrupt**: **Priority 1** (Cung cấp nhịp đập thời gian thực 200kHz cho DDA).
3. **USB OTG FS Interrupt**: **Priority 2** (Xử lý giao tiếp USB CDC, parser lệnh và chuyển trạng thái).
4. **USART / SDIO Interrupt**: **Priority 3** (Các giao tiếp phụ trợ).
