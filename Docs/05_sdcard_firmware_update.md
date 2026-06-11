# 05 — Nạp Firmware qua MicroSD (SDIO)

## 📌 Tổng Quan

STM32F446ZE có **SDIO interface** kết nối MicroSD (4-bit wide bus @ 48MHz).  
Firmware mới được đặt vào thẻ SD → Bootloader đọc và nạp vào Flash nội.

```
MicroSD Card                    STM32F446ZE
────────────                    ───────────
firmware.bin ──── SDIO 4-bit ──→  Flash 0x08008000
(thẻ nhớ)         @ 48MHz        (Application Space)
```

---

## 🔌 Sơ Đồ Kết Nối SDIO

| Tín hiệu SDIO | Pin STM32 | Mô tả |
|---------------|-----------|-------|
| SDIO_D0 | **PC8** | Data bit 0 |
| SDIO_D1 | **PC9** | Data bit 1 |
| SDIO_D2 | **PC10** | Data bit 2 |
| SDIO_D3 | **PC11** | Data bit 3 |
| SDIO_CK | **PC12** | Clock |
| SDIO_CMD | **PD2** | Command |
| Card Detect | **PC14 (DET)** | Phát hiện thẻ SD |

**Clock SDIO:** 48 MHz (từ PLLSAI → CLK48)

---

## 🏗️ Kiến Trúc Bộ Nhớ Flash

```
STM32F446ZE Flash Layout (512KB):

┌─────────────────────────────────┐ 0x08000000
│   Sector 0 (16KB)               │
│   BOOTLOADER                    │ ← Bootloader nạp từ SD
│   - Khởi động, kiểm tra SD     │
│   - Copy firmware.bin vào Flash │
├─────────────────────────────────┤ 0x08004000
│   Sector 1 (16KB)               │
│   BOOTLOADER (tiếp)             │
├─────────────────────────────────┤ 0x08008000
│   Sector 2 (16KB)               │
│   APPLICATION                   │ ← Firmware chính
│   (Stepper Controller)          │
├─────────────────────────────────┤ 0x0800C000
│   Sector 3 (16KB)               │
│   APPLICATION (tiếp)            │
├─────────────────────────────────┤ 0x08010000
│   Sector 4 (64KB)               │
│   APPLICATION (tiếp)            │
├─────────────────────────────────┤ 0x08020000
│   Sector 5 (128KB)              │
│   APPLICATION (tiếp)            │
├─────────────────────────────────┤ 0x08040000
│   Sector 6 (128KB)              │
│   APPLICATION / Storage         │
├─────────────────────────────────┤ 0x08060000
│   Sector 7 (128KB)              │
│   Dự phòng / Config             │
└─────────────────────────────────┘ 0x08080000
```

**Lưu ý:** Application phải được link tại địa chỉ `0x08008000`.

---

## 📋 Quy Trình Nạp Firmware

### Phía Người Dùng

```
1. Build firmware trong STM32CubeIDE
   → tạo ra file: final.bin (hoặc final.hex)

2. Chuyển sang binary (nếu dùng .hex):
   arm-none-eabi-objcopy -O binary final.elf firmware.bin

3. Copy firmware.bin vào thẻ MicroSD (root directory)

4. Cắm thẻ SD vào bo mạch

5. Reset bo mạch (nhấn RESET hoặc tắt/bật nguồn)

6. Bootloader tự động:
   - Phát hiện thẻ SD (DET pin = LOW)
   - Tìm file "firmware.bin" ở root
   - Erase Flash Sector 2–7
   - Copy firmware.bin vào 0x08008000
   - Xác nhận CRC
   - Jump đến Application

7. LED WORK_LED (PA13) nhấp nháy → nạp thành công
```

---

## 🔧 Cấu Hình SDIO trong CubeMX

```
SDIO Mode: SD_4_bits_Wide_bus
Clock divide factor: 0 (SDIO_CK = 48MHz / (2+0) = 24MHz for init, thay đổi sau)
Hardware flow control: Enable
```

**Trong `sdio.c`** (đã được CubeMX tạo):
```c
SD_HandleTypeDef hsd;

void MX_SDIO_SD_Init(void) {
    hsd.Instance = SDIO;
    hsd.Init.ClockEdge           = SDIO_CLOCK_EDGE_RISING;
    hsd.Init.ClockBypass         = SDIO_CLOCK_BYPASS_DISABLE;
    hsd.Init.ClockPowerSave      = SDIO_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide             = SDIO_BUS_WIDE_4B;
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
    hsd.Init.ClockDiv            = 0;
    HAL_SD_Init(&hsd);
}
```

---

## 📝 Code Bootloader — Đọc Firmware từ SD

```c
#include "ff.h"         // FatFS
#include "sdio.h"

#define APP_START_ADDR  0x08008000
#define FIRMWARE_FILE   "firmware.bin"

typedef void (*AppEntry_t)(void);

/**
 * @brief Kiểm tra và copy firmware từ SD vào Flash
 * @return 1 nếu thành công, 0 nếu thất bại
 */
uint8_t Bootloader_LoadFromSD(void) {
    FATFS fs;
    FIL   fil;
    FRESULT res;
    
    // 1. Mount filesystem
    res = f_mount(&fs, "", 1);
    if (res != FR_OK) return 0;
    
    // 2. Mở file firmware.bin
    res = f_open(&fil, FIRMWARE_FILE, FA_READ);
    if (res != FR_OK) {
        f_unmount("");
        return 0;
    }
    
    // 3. Lấy kích thước file
    FSIZE_t file_size = f_size(&fil);
    if (file_size == 0 || file_size > 448 * 1024) {  // Tối đa 448KB
        f_close(&fil);
        f_unmount("");
        return 0;
    }
    
    // 4. Erase Flash (Sector 2 đến Sector 7)
    FLASH_EraseInitTypeDef erase = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
        .Sector       = FLASH_SECTOR_2,
        .NbSectors    = 6,
    };
    uint32_t sector_error;
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase, &sector_error);
    
    // 5. Đọc và ghi từng block
    uint8_t  buf[512];
    uint32_t addr = APP_START_ADDR;
    UINT     bytes_read;
    
    while (f_read(&fil, buf, sizeof(buf), &bytes_read) == FR_OK && bytes_read > 0) {
        for (uint32_t i = 0; i < bytes_read; i += 4) {
            uint32_t word = *(uint32_t*)(buf + i);
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, word);
            addr += 4;
        }
    }
    HAL_FLASH_Lock();
    
    // 6. Đóng file
    f_close(&fil);
    f_unmount("");
    
    return 1;
}

/**
 * @brief Nhảy đến Application
 */
void Bootloader_JumpToApp(void) {
    // Lấy Stack Pointer và Entry Point từ vector table của Application
    uint32_t app_sp    = *(uint32_t*) APP_START_ADDR;
    uint32_t app_entry = *(uint32_t*)(APP_START_ADDR + 4);
    
    // Kiểm tra địa chỉ hợp lệ
    if ((app_sp & 0x2FFE0000) != 0x20000000) return;  // SP phải trong RAM
    
    // Disable tất cả ngắt
    __disable_irq();
    
    // Set VTOR (Vector Table Offset Register)
    SCB->VTOR = APP_START_ADDR;
    
    // Set Stack Pointer
    __set_MSP(app_sp);
    
    // Nhảy đến Application
    AppEntry_t app = (AppEntry_t) app_entry;
    __enable_irq();
    app();
}
```

---

## 📦 FatFS Integration

### Thêm FatFS vào project

1. Trong CubeMX: **Middleware → FATFS → User-defined**
2. Hoặc thêm thủ công thư mục `Middlewares/Third_Party/FatFs/`
3. File cần: `ff.h`, `ff.c`, `diskio.c`

### diskio.c — Link SDIO với FatFS

```c
DSTATUS disk_initialize(BYTE pdrv) {
    if (HAL_SD_InitCard(&hsd) != HAL_OK) return STA_NOINIT;
    return 0;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) {
    if (HAL_SD_ReadBlocks(&hsd, buff, sector, count, 5000) != HAL_OK)
        return RES_ERROR;
    return RES_OK;
}
```

---

## 🔐 Kiểm Tra Tính Toàn Vẹn (CRC32)

Để tăng độ tin cậy, thêm file `firmware.crc` vào thẻ SD:

```
firmware.bin   ← firmware binary
firmware.crc   ← CRC32 checksum (4 bytes)
```

```c
// Kiểm tra CRC trước khi nạp
uint32_t Bootloader_VerifyCRC(void) {
    // Đọc firmware.bin và tính CRC32
    // So sánh với firmware.crc
    // Trả về 1 nếu khớp
}
```

---

## 🚦 Cấu Hình Linker Script cho Application

Trong `STM32F446ZETx_FLASH.ld` của project Application:

```ld
MEMORY {
    RAM    (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
    FLASH  (rx)  : ORIGIN = 0x08008000, LENGTH = 448K  /* ← Bắt đầu tại Sector 2 */
}
```

---

## ✅ Checklist Nạp Firmware

```
[ ] 1. Build project thành công (không có lỗi)
[ ] 2. Linker origin = 0x08008000
[ ] 3. Copy firmware.bin vào root thẻ SD
[ ] 4. Cắm thẻ SD vào bo mạch (DET pin PC14 nhận biết)
[ ] 5. Reset bo mạch
[ ] 6. Quan sát LED: nhấp nháy nhanh = đang nạp, nhấp nháy chậm = thành công
[ ] 7. Tháo thẻ SD (tùy chọn — bootloader có thể bỏ qua nếu không có thẻ)
[ ] 8. Reset lần nữa → Application chạy
```

---

## ⚠️ Lưu Ý

1. **Card Detect (PC14/DET):** Kiểm tra mức logic. Nếu LOW = có thẻ, bootloader sẽ thử nạp.
2. **Timeout:** Nếu đọc SD quá 3 giây → tự động boot Application cũ.
3. **Backup:** Nếu nạp thất bại (CRC lỗi), giữ nguyên Application cũ.
4. **SDIO clock:** Init ở 400kHz, sau đó tăng lên 24MHz khi đã nhận diện thẻ.
