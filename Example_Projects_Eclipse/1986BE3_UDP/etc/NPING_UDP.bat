@echo off
if "%1"=="" exit /B

ping 192.168.1.%1
c:\dev\nmap-7.40\nping --udp -p 18334 -N --data-length 1024 -c 99999 -rate 200 192.168.1.%1
