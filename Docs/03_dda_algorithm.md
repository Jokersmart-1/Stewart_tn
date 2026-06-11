# 03 — Thuật Toán DDA — 6 Trục Đồng Bộ

## 📐 DDA (Digital Differential Analyzer) là gì?

DDA là thuật toán điều khiển động cơ bước cho phép **nhiều trục chạy đồng thời và đến đích cùng lúc** (synchronized motion).

**Nguyên lý:**
- Mỗi trục có một **bộ tích lũy (accumulator)** là số nguyên
- Mỗi tick timer, bộ tích lũy được cộng thêm **`velocity`** (tỉ lệ tốc độ)
- Khi tích lũy **≥ threshold** (thường là `4000` hoặc `STEP_SCALE`): bắn 1 xung STEP, trừ threshold
- Tỉ lệ `velocity / threshold` = tốc độ tương đối của trục đó

---

## 🧮 Công Thức DDA

```
velocity_i = (steps_i / steps_max) × STEP_SCALE
```

Trong đó:
- `steps_i` = số bước cần đi của trục i
- `steps_max` = số bước nhiều nhất trong các trục (trục chủ)
- `STEP_SCALE` = hằng số chia (ví dụ: 4000)

**Mỗi tick:**
```c
accum_i += velocity_i;
if (accum_i >= STEP_SCALE) {
    accum_i -= STEP_SCALE;
    // Bắn xung STEP_i
}
```

---

## 🔬 Ví Dụ Đang Chạy (Phase 2 — 2 Trục Test)

```c
// Trong TIM2 ISR callback:

// STEP1: velocity = 2000, threshold = 4000 → tần số = 200kHz × (2000/4000) = 100kHz
step1_accum += 2000;
if (step1_accum >= 4000) {
    step1_accum -= 4000;
    HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_SET);
    step1_state_timer = 1;  // Giữ HIGH trong 1 tick (5µs)
    step1_count++;
}

// STEP2: velocity = 3000, threshold = 4000 → tần số = 200kHz × (3000/4000) = 150kHz
step2_accum += 3000;
if (step2_accum >= 4000) {
    step2_accum -= 4000;
    HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_SET);
    step2_state_timer = 1;
    step2_count++;
}
```

**Kết quả sau 20ms (4000 ticks):**
| Trục | Velocity | Bước lý thuyết | Bước thực tế |
|------|----------|----------------|--------------|
| STEP1 | 2000 | 4000 × 0.5 = 2000 | ~2000 |
| STEP2 | 3000 | 4000 × 0.75 = 3000 | ~3000 |

---

## 🚀 Phase 3 — Mở Rộng 6 Trục

### Cấu Trúc Dữ Liệu Cần Thêm

```c
// Trong main.c — USER CODE BEGIN PV

#define NUM_AXES    6
#define STEP_SCALE  65536  // Độ phân giải cao hơn (16-bit)

// Bảng cấu hình mỗi trục
typedef struct {
    GPIO_TypeDef *step_port;
    uint16_t      step_pin;
    GPIO_TypeDef *dir_port;
    uint16_t      dir_pin;
    uint32_t      accum;        // Bộ tích lũy DDA
    uint32_t      velocity;     // velocity = (steps_i / steps_max) × STEP_SCALE
    uint32_t      steps_total;  // Tổng số bước cần đi
    uint32_t      steps_done;   // Số bước đã thực hiện
    uint8_t       state_timer;  // Đếm tick để kéo STEP xuống LOW
    uint8_t       dir;          // 0 = forward, 1 = reverse
} AxisConfig_t;

AxisConfig_t axes[NUM_AXES] = {
    // {STEP_port,    STEP_pin, DIR_port,    DIR_pin,   accum, vel, total, done, timer, dir}
    {GPIOF, STEP0_Pin, GPIOF, DIR0_Pin, 0, 0, 0, 0, 0, 0},  // M0
    {GPIOG, STEP1_Pin, GPIOG, DIR1_Pin, 0, 0, 0, 0, 0, 0},  // M1
    {GPIOF, STEP2_Pin, GPIOF, GPIO_PIN_12, 0, 0, 0, 0, 0, 0},// M2 (DIR2 cần xác nhận)
    {GPIOG, STEP3_Pin, GPIOG, DIR2_Pin, 0, 0, 0, 0, 0, 0},  // M3
    {GPIOF, STEP4_Pin, GPIOF, DIR4_Pin, 0, 0, 0, 0, 0, 0},  // M4
    {GPIOC, STEP5_Pin, GPIOF, DIR5_Pin, 0, 0, 0, 0, 0, 0},  // M5
};
```

### Hàm Tính Velocity Từ Target Steps

```c
/**
 * @brief Tính velocity cho mỗi trục dựa trên số bước mục tiêu
 *        Thời gian di chuyển cố định là 20ms (4000 ticks ngắt ở tần số 200kHz).
 *        Các trục được tính velocity tỉ lệ để hoàn thành trong đúng 4000 ticks.
 * @param target_steps  Mảng số bước cần đi cho mỗi trục
 */
void DDA_SetTarget(uint32_t target_steps[NUM_AXES]) {
    // Tìm số bước lớn nhất (trục chủ) để check an toàn
    uint32_t max_steps = 0;
    for (int i = 0; i < NUM_AXES; i++) {
        if (target_steps[i] > max_steps)
            max_steps = target_steps[i];
    }
    if (max_steps == 0) return;

    // Tính velocity cho từng trục dựa trên thời gian chạy cố định 4000 ticks (20ms)
    for (int i = 0; i < NUM_AXES; i++) {
        axes[i].steps_total = target_steps[i];
        axes[i].steps_done  = 0;
        axes[i].accum       = 0;
        axes[i].state_timer = 0;
        // Sử dụng phép chia trần (ceiling division) để đảm bảo không bị sai số làm mất bước ở số lượng bước nhỏ
        axes[i].velocity    = ((uint64_t)target_steps[i] * STEP_SCALE + 3999U) / 4000U;
    }
}
```

### ISR Callback 6 Trục

```c
void TIM2_DDA_Callback(void) {
    uint8_t all_done = 1;

    for (int i = 0; i < NUM_AXES; i++) {
        // Kéo STEP xuống LOW sau 1 tick
        if (axes[i].state_timer > 0) {
            axes[i].state_timer--;
            if (axes[i].state_timer == 0) {
                HAL_GPIO_WritePin(axes[i].step_port, axes[i].step_pin, GPIO_PIN_RESET);
            }
        }

        // Chỉ tiếp tục nếu còn bước cần đi
        if (axes[i].steps_done < axes[i].steps_total) {
            all_done = 0;
            axes[i].accum += axes[i].velocity;
            if (axes[i].accum >= STEP_SCALE) {
                axes[i].accum -= STEP_SCALE;
                HAL_GPIO_WritePin(axes[i].step_port, axes[i].step_pin, GPIO_PIN_SET);
                axes[i].state_timer = 1;
                axes[i].steps_done++;
            }
        }
    }

    // Dừng khi tất cả trục hoàn thành
    if (all_done) {
        simulation_running = 0;
        TIM2->CR1 &= ~TIM_CR1_CEN;
    }
}
```

---

## 📊 Ví Dụ: 6 Trục Đến Đích Cùng Lúc

**Target:** M0=1000, M1=2000, M2=500, M3=1500, M4=2000, M5=800 bước

| Trục | Steps | Velocity (SCALE=65536) | Tần số STEP |
|------|-------|------------------------|-------------|
| M0 | 1000 | 32768 | 100 kHz |
| M1 | 2000 | 65536 (full) | 200 kHz |
| M2 | 500 | 16384 | 50 kHz |
| M3 | 1500 | 49152 | 150 kHz |
| M4 | 2000 | 65536 (full) | 200 kHz |
| M5 | 800 | 26214 | 80 kHz |

**Kết quả:** Tất cả 6 trục đến đích sau ~10ms (2000 bước @ 200kHz)

---

## ⚙️ Cấu Hình Chiều Quay (DIR)

```c
// Đặt chiều trước khi start
void DDA_SetDirection(uint8_t axis, uint8_t dir) {
    // dir: 0 = forward (HIGH), 1 = reverse (LOW)
    // TMC driver: cần thiết lập DIR trước ít nhất 200ns trước xung STEP đầu tiên
    axes[axis].dir = dir;
    HAL_GPIO_WritePin(axes[axis].dir_port, axes[axis].dir_pin,
                      dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
```

> ⚠️ **Setup time:** DIR phải ổn định trước STEP ít nhất **200 ns** (TMC2208/TMC2209) hoặc **650 ns** (A4988).

---

## 🔢 Độ Chính Xác DDA

Với `STEP_SCALE = 65536` (16-bit):
- Sai số tích lũy tối đa: ± 0.5 bước trên tổng hành trình
- Phù hợp cho hầu hết ứng dụng CNC/robot

Với `STEP_SCALE = 4000` (như hiện tại test):
- Đủ cho test nhưng có thể có sai số pha nhỏ

---

## 📌 TODO — Phase 3

- [x] Chuyển sang cấu trúc `AxisConfig_t[6]` — **Done** (M1–M6, BSRR optimized)
- [x] Implement `DDA_SetTarget()` — **Done** (velocity = steps_i / max_steps × 65536)
- [x] Implement `DDA_Init()` + `DDA_Start()` — **Done** (EN LOW, DIR HIGH, TIM2 start)
- [x] Test values: `{1000, 2000, 3000, 4000, 500, 0}` (M4 dominant @ 200kHz)
- [ ] Test 6 trục đồng thời trên phần cứng (oscilloscope)
- [ ] Đo thời gian thực thi ISR (mục tiêu < 3µs)
- [ ] Thêm acceleration profile (trapezoidal ramp) — Phase tương lai
