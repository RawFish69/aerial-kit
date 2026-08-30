@echo off
REM CRSF Live Dashboard Launcher

REM ============= SET YOUR COM PORT =============
set COM_PORT=COM3
REM ============================================

python crsf_live.py %COM_PORT%
pause

