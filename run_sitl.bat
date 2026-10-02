@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo ============================================
echo  CUADC_fixedwing SITL launcher
echo  Ensure MP SITL is running first (TCP 5762)
echo ============================================
x64\Release\CUADC_fixedwing.exe
echo.
echo [exit] program stopped, press any key to close...
pause >nul
