Set WshShell = CreateObject("WScript.Shell")
WshShell.Run "python -m http.server 8000 --bind 0.0.0.0 --directory D:\final", 0, False
