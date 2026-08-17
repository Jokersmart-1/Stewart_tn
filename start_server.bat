@echo off
title Stewart 6-DOF Web Server
chcp 65001 > nul
echo ========================================================
echo   STEWART 6-DOF - BÁO CÁO ĐỒ ÁN TỐT NGHIỆP
echo   Đang khởi động Web Server phục vụ iPhone / iPad / PC...
echo ========================================================
echo.

:: Lấy địa chỉ IP mạng Wi-Fi/LAN của máy tính
for /f "tokens=2 delims=:" %%a in ('ipconfig ^| findstr /c:"IPv4 Address" /c:"Địa chỉ IPv4"') do (
    set IP=%%a
)
:: Cắt khoảng trắng đầu dòng
for /f "tokens=* delims= " %%b in ("%IP%") do set IP=%%b

echo [OK] Máy chủ đang chạy tại cổng 8000!
echo.
echo 📱 ĐỊA CHỈ TRUY CẬP TRÊN IPHONE (Cùng mạng Wi-Fi):
echo 👉 http://%IP%:8000/presentation.html
echo.
echo 💻 TRUY CẬP TRÊN MÁY TÍNH NÀY:
echo 👉 http://localhost:8000/presentation.html
echo.
echo --------------------------------------------------------
echo (Giữ cửa sổ này mở trong lúc thuyết trình. Nhấn Ctrl+C để tắt)
echo --------------------------------------------------------
echo.

cd /d "D:\final"
python -m http.server 8000 --bind 0.0.0.0
pause
