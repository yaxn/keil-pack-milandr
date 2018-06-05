@echo off
set SCRIPT_DIR=%~dp0

start "" "%SCRIPT_DIR%KiTTY\kitty_portable.exe" -load "RS232"
