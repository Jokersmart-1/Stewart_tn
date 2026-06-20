# Lưu đồ Giải thuật Toàn hệ thống & Quy trình Homing (System Flowchart & Homing)

Tài liệu này mô tả luồng hoạt động tổng thể của bộ điều khiển động cơ bước 6 trục từ khi khởi động, quy trình thiết lập gốc tọa độ (Homing Sequence) sử dụng cảm biến hành trình (End-stops), cho đến vòng lặp nhận lệnh và phát chuyển động.

---

## 1. Các Trạng thái chính của Hệ thống (System States)

Hệ thống vận hành xoay quanh 5 trạng thái logic cơ bản:
1. **INIT (Khởi tạo):** Thiết lập Clock, GPIO, USB CDC, Timer, DMA, cài đặt mức ưu tiên ngắt NVIC và cấu hình ban đầu cho 6 trục.
2. **HOMING (Tìm gốc tọa độ):** Di chuyển độc lập hoặc đồng thời các trục để tìm vị trí cảm biến hành trình (`STOP_0` đến `STOP_5`), hiệu chuẩn điểm gốc ($0$).
3. **IDLE (Chờ lệnh):** Trạng thái rảnh rỗi, lắng nghe lệnh gửi đến từ Raspberry Pi qua cổng USB.
4. **RUNNING (Chuyển động):** Nhận lệnh chuyển động, thực hiện tính toán Ping-Pong và kích hoạt DMA phát xung điều khiển động cơ.
5. **ESTOP (Dừng khẩn cấp):** Khi nhận lệnh `'E'` từ Host hoặc phát hiện sự cố, hệ thống lập tức dừng cứng tất cả hoạt động để đảm bảo an toàn cơ khí.

---

## 2. Lưu đồ Giải thuật Toàn hệ thống (High-Level System Flowchart)

Dưới đây là lưu đồ hoạt động mức cao của hệ thống từ lúc cấp nguồn, tích hợp quy trình tự động về Home:

```mermaid
flowchart TD
    %% Định nghĩa các Style màu sắc
    classDef init fill:#d4ebf2,stroke:#333,stroke-width:2px;
    classDef homing fill:#ffe5b4,stroke:#333,stroke-width:2px;
    classDef idle fill:#e6e6fa,stroke:#333,stroke-width:2px;
    classDef run fill:#d1e7dd,stroke:#333,stroke-width:2px;
    classDef estop fill:#f8d7da,stroke:#333,stroke-width:2px;

    %% Luồng Khởi tạo
    PowerOn([Cấp nguồn / Reset]) --> Init[Khởi tạo phần cứng:<br>- SYSCLK 180MHz<br>- GPIO pins STEP/DIR/EN/STOP<br>- USB CDC, TIM1, DMA<br>- Reset hàng đợi cmd_queue]:::init
    Init --> CheckHoming{Nhận lệnh Homing 'H' <br/> từ Host?}:::idle

    %% Luồng Homing (Thiết lập gốc tọa độ)
    CheckHoming -- Yes --> HomeStart[Bắt đầu Homing 6 trục]:::homing
    HomeStart --> HomeDir[Đặt hướng di chuyển DIR ngược về phía cảm biến hành trình]:::homing
    HomeDir --> HomeLoop[Phát xung chậm cho các trục chưa chạm End-stop]:::homing
    HomeLoop --> ReadStops{Đọc chân STOP_0..5 <br/> Trục nào chạm End-stop?}:::homing
    
    ReadStops -- Trục i chạm --> StopAxis[Dừng phát xung trục i]:::homing
    StopAxis --> CheckAllHome{Tất cả 6 trục <br/> đã chạm End-stop?}:::homing
    
    ReadStops -- Chưa chạm --> HomeLoop
    
    CheckAllHome -- Chưa --> HomeLoop
    
    %% Quy trình lùi nhẹ (Backoff) để tăng độ chính xác homing
    CheckAllHome -- Rồi --> Backoff[Lùi nhẹ các trục ra xa cảm biến hành trình <br/> và chạm lại lần 2 với tốc độ cực chậm]:::homing
    Backoff --> SetZero[Thiết lập vị trí hiện tại làm gốc tọa độ 0 <br/> Xóa sạch hàng đợi lệnh]:::homing
    SetZero --> SendHomeDone[Gửi phản hồi 'Homing Done' về Host]:::homing
    SendHomeDone --> IdleState:::idle

    %% Luồng Hoạt động bình thường (IDLE & RUNNING)
    CheckHoming -- No --> IdleState[Trạng thái IDLE:<br>- Lắng nghe cổng USB CDC<br>- Đèn WORK_LED báo hiệu]:::idle
    
    IdleState --> RxCmd{Nhận dữ liệu <br/> từ USB CDC?}:::idle
    
    RxCmd -- Lệnh dừng khẩn 'E' --> EmergencyStop:::estop
    RxCmd -- Lệnh di chuyển 'M' --> ProcessMove:::run
    RxCmd -- Không nhận được --> IdleState

    %% Xử lý di chuyển
    ProcessMove[Xử lý lệnh di chuyển M:<br>- Giải mã số bước 6 trục<br>- Đẩy vào cmd_queue và gửi ACK 'K'<br>- Tính toán phân đoạn DDA Ping-Pong]:::run
    ProcessMove --> StartDma[Kích hoạt TIM1 + DMA <br/> Tự động phát xung ra GPIO]:::run
    
    StartDma --> RunLoop{Đang di chuyển?}:::run
    RunLoop -- Phát hiện lệnh khẩn 'E' --> EmergencyStop:::estop
    RunLoop -- Vẫn đang chạy bình thường --> WaitSegment[DMA tự động phát bộ đệm RAM]:::run
    
    WaitSegment --> FinishSegment{DMA phát hết 20ms <br/> của lệnh hiện tại?}:::run
    FinishSegment -- Chưa --> RunLoop
    FinishSegment -- Rồi --> SendDone[Gửi tín hiệu 'D' Done về Host]:::run
    SendDone --> IdleState

    %% Luồng Dừng khẩn cấp (ESTOP)
    EmergencyStop[Dừng khẩn cấp:<br>- Tắt cứng Timer 1 & các luồng DMA<br>- Kéo tất cả chân STEP về HIGH (Idle)<br>- Xóa sạch hàng đợi cmd_queue<br>- Gửi trạng thái lỗi về Host]:::estop
    EmergencyStop --> ReInit[Chờ lệnh phục hồi từ Host]:::estop
    ReInit --> IdleState

    %% Áp dụng màu sắc cho từng block
```

---

## 3. Mô tả chi tiết Quy trình Homing (Homing Sequence)

Quy trình Homing trong lưu đồ được thiết kế để định vị điểm không ($0$) vật lý cho 6 cơ cấu chấp hành một cách độc lập và chính xác:

1. **Kích hoạt:** Khi nhận ký tự `'H'` từ Raspberry Pi, hệ thống chuyển sang trạng thái Homing.
2. **Dò thô (Fast Search):**
   * Thiết lập chân `DIR` của 6 trục quay ngược về hướng lắp cảm biến hành trình (End-stops).
   * Phát chuỗi xung với tốc độ trung bình. 
   * Bộ xử lý liên tục đọc trạng thái các chân `STOP_0` đến `STOP_5` (định cấu hình ngắt hoặc quét nhanh).
3. **Phản ứng va chạm:** Ngay khi một chân `STOP_i` chuyển sang trạng thái tích cực (chạm cảm biến), trục tương ứng sẽ bị dừng phát xung ngay lập tức trong khi các trục khác vẫn tiếp tục dò.
4. **Lùi lò xo (Backoff) & Dò tinh (Slow Touch):**
   * Sau khi cả 6 trục đều chạm cảm biến lần đầu, hệ thống sẽ điều khiển cả 6 trục quay ngược lại một khoảng nhỏ (ví dụ: 100 bước) để nhả cảm biến hành trình.
   * Tiến hành di chuyển chạm cảm biến lần thứ 2 với tốc độ cực kỳ chậm (thường bằng 1/10 tốc độ ban đầu) nhằm loại bỏ sai số cơ học do quán tính và nâng cao độ chính xác lặp lại.
5. **Đồng bộ hóa:** Thiết lập thanh ghi bộ đếm vị trí thực tế của 6 trục về $0$, gửi phản hồi hoàn thành để báo cho Host bắt đầu chu trình chạy quỹ đạo.
