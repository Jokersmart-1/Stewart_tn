# Sơ Đồ Quy Trình Mô Phỏng Sóng Stewart Platform

Sơ đồ tóm tắt luồng truyền dữ liệu và phối hợp giữa **Phần mềm (Software)**, **Phần cứng (Hardware)** và các **Tệp dữ liệu (CSV)**:

```mermaid
flowchart LR
    subgraph SW["💻 PHẦN MỀM"]
        M1["MATLAB<br/>(Động học nghịch)"]
        S1["PC Web Server<br/>(Bộ đệm & Gửi lệnh)"]
        LA_SW["Logic SW<br/>(Ghi xung thực tế)"]
        M2["MATLAB<br/>(Tính ngược hành trình)"]
        R1["RecurDyn<br/>(Mô phỏng 3D)"]
    end

    subgraph DATA["📄 DỮ LIỆU"]
        D1[("1. trajectory.csv<br/>(Xung/DIR lý thuyết)")]
        D2[("2. measured.csv<br/>(Xung/DIR thực tế)")]
        D3[("3. motion.csv<br/>(Hành trình khôi phục)")]
    end

    subgraph HW["🔌 PHẦN CỨNG"]
        PC["Host PC"]
        MCU["MCU (STM32)<br/>(GPIO phát xung)"]
        LA["Logic Analyzer<br/>(Que đo tín hiệu)"]
        ST["Stewart Platform<br/>(Mô hình 3D)"]
    end

    %% Luồng kết nối
    M1 --> D1 --> S1
    S1 ==>|"USB/Serial"| MCU
    MCU -->|"Tín hiệu GPIO"| LA --> LA_SW
    LA_SW --> D2 --> M2 --> D3 --> R1
    R1 --> ST
```

---

### Tóm tắt Luồng Thực Thi

1. **MATLAB (`M1`)** tính động học nghịch $\rightarrow$ xuất `trajectory.csv` (`D1`).
2. **PC Web Server (`S1`)** đọc file $\rightarrow$ truyền gói tin qua **USB Serial** xuống **MCU (`MCU`)**.
3. **MCU (`MCU`)** phát xung GPIO $\rightarrow$ **Logic Analyzer (`LA`)** đo tín hiệu số và xuất file `measured.csv` (`D2`) qua **Logic SW (`LA_SW`)**.
4. **MATLAB (`M2`)** giải mã xung từ `measured.csv` $\rightarrow$ tính ngược hành trình 6 chân $\rightarrow$ xuất `motion.csv` (`D3`).
5. **RecurDyn (`R1`)** đọc `motion.csv` để mô phỏng chuyển động sóng trên mô hình **Stewart Platform (`ST`)**.
