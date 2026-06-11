# 02 — Cấu Hình Clock & Timer TIM2

## 🕐 Cấu Hình Clock Hệ Thống — 180 MHz

### Nguồn Clock
| Thông số | Giá trị |
|----------|---------|
| Oscillator | HSE External (PH0/PH1) |
| HSE Frequency | **12 MHz** |
| PLL Source | HSE |
| PLLM | 6 → VCO Input = 2 MHz |
| PLLN | 180 → VCO Output = 360 MHz |
| PLLP | /2 → **SYSCLK = 180 MHz** |
| PLLQ | 3 → USB FS không dùng kênh này |
| Over-Drive | Enabled (bắt buộc ở 180MHz) |

### Phân Tần Bus
| Bus | Divider | Frequency |
|-----|---------|-----------|
| SYSCLK | /1 | **180 MHz** |
| AHB (HCLK) | /1 | **180 MHz** |
| APB1 (PCLK1) | /4 | **45 MHz** |
| APB1 Timer | ×2 | **90 MHz** ← TIM2 input |
| APB2 (PCLK2) | /2 | **90 MHz** |

### Clock Ngoại Vi Đặc Biệt (PLLSAI)
| Ngoại vi | Clock | Giá trị |
|----------|-------|---------|
| SDIO | CLK48 (PLLSAIP) | **48 MHz** |
| USB OTG FS | CLK48 (PLLSAIP) | **48 MHz** |
| PLLSAIM | 6 | VCO SAI input = 2 MHz |
| PLLSAIN | 96 | VCO SAI output = 192 MHz |
| PLLSAIP | /4 | → 48 MHz |

---

## ⏱️ TIM2 — Bộ Tạo Xung DDA (200 kHz)

### Mục Đích
TIM2 tạo ngắt đều đặn mỗi **5 µs (200,000 lần/giây)** để:
- Cộng bộ tích lũy DDA cho từng trục
- Bắn xung STEP khi tích lũy tràn
- Kéo chân STEP xuống LOW sau 1 tick (pulse width ≈ 5µs)

### Tham Số Timer

```
APB1 Timer Clock = 90 MHz
PSC (Prescaler)  = 90 - 1 = 89   → Timer count clock = 1 MHz
ARR (Auto-Reload) = 5 - 1 = 4    → Interrupt period = 5 µs
Interrupt freq   = 1 MHz / 5 = 200,000 Hz = 200 kHz
```

### Code Khởi Tạo TIM2

```c
void MX_TIM2_Init(void) {
    // 1. Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // 2. Prescaler: 90MHz → 1MHz count clock
    TIM2->PSC = 90 - 1;

    // 3. Auto-Reload: 1MHz / 5 = 200kHz interrupt
    TIM2->ARR = 5 - 1;

    // 4. Reset counter & clear flag
    TIM2->CNT = 0;
    TIM2->SR  = ~TIM_SR_UIF;

    // 5. Enable Update Interrupt
    TIM2->DIER |= TIM_DIER_UIE;

    // 6. NVIC: Priority 0 (highest) — critical for pulse timing
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}
```

### Sơ Đồ Timing

```
TIM2 tick period = 1 µs (at 1 MHz count clock)

Tick:  0    1    2    3    4    0    1    2    3    4   ...
       │────────────────────│────────────────────│
       ↑ Interrupt          ↑ Interrupt
       (every 5 ticks = 5µs = 200kHz)
```

### Xung STEP — Dạng Sóng

```
STEP pin:
         ┌──┐        ┌──┐
─────────┘  └────────┘  └──────
         5µs 5µs      5µs 5µs
         ▲              ▲
      SET (HIGH)      SET (HIGH)
      at tick N       at tick N+k
         ▲              ▲
      RESET (LOW)    RESET (LOW)
      at tick N+1    at tick N+k+1
      (state_timer=0)
```

> **Pulse Width:** 1 tick = **5 µs** (đủ để driver A4988/TMC2208/DRV8825 nhận diện — yêu cầu tối thiểu 1µs)

---

## 🔁 Vòng Lặp Chính (main loop)

```c
while (1) {
    // Reset biến trạng thái
    step1_count = 0; step2_count = 0;
    interrupt_count = 0;
    step1_accum = 0;  step2_accum = 0;
    step1_state_timer = 0; step2_state_timer = 0;

    // Khởi động TIM2
    simulation_running = 1;
    TIM2->CNT = 0;
    TIM2->SR  = ~TIM_SR_UIF;
    TIM2->CR1 |= TIM_CR1_CEN;

    // Chờ hoàn thành 20ms (4000 ngắt)
    while (simulation_running) { }

    // Toggle LED khi xong
    HAL_GPIO_TogglePin(WORK_LED_GPIO_Port, WORK_LED_Pin);
    HAL_Delay(1000);
}
```

**Giải thích:**
- Mỗi chu kỳ: TIM2 chạy **4000 ngắt** (20ms @ 200kHz)
- Sau 4000 ngắt: `simulation_running = 0`, TIM2 tự tắt
- LED WORK_LED (PA13) toggle để báo hiệu

---

## 📊 Tần Số Bước Tối Đa

| Tần số Timer | Tốc độ tối đa (lý thuyết) |
|-------------|--------------------------|
| 200 kHz | 200,000 step/s = 200 kstep/s |
| Thực tế (DDA 6 trục) | ~100–150 kstep/s (có overhead ISR) |

> **Lưu ý:** Khi chạy 6 trục song song, thời gian xử lý ISR tăng lên. Cần đo thực tế và giảm PSC nếu cần throughput cao hơn.

---

## ⚠️ Lưu Ý Quan Trọng

1. **SysTick Recalibrate:** Sau khi config PLL, phải gọi lại:
   ```c
   SystemCoreClock = HAL_RCC_GetSysClockFreq();
   HAL_SYSTICK_Config(SystemCoreClock / 1000U);
   ```
   Vì CubeMX có thể set sai `SystemCoreClock` trước khi PLL lock.

2. **Over-Drive Mode:** Bắt buộc để chạy 180MHz. Nếu `HAL_PWREx_EnableOverDrive()` fail → MCU chạy ở tần số thấp hơn.

3. **TIM2 là 32-bit:** ARR và CNT của TIM2 trên STM32F4 là 32-bit, có thể dùng prescaler lớn hơn nếu cần.
