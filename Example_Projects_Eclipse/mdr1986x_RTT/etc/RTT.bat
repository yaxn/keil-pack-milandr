@echo off
set SCRIPT_DIR=%~dp0

start JLinkGDBServerCL.exe -if swd -device "Cortex-M1" -endian little -speed 2000 -port 2331 -vd -localhostonly 1 -strict -notimeout
start arm-none-eabi-gdb-py --batch -x "%SCRIPT_DIR%RTT.py" -ex "py RTT('%1')"

rem start JLinkRTTClient.exe
start "" "%SCRIPT_DIR%KiTTY\kitty_portable.exe" -load "RTT"
