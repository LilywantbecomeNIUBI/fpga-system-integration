@echo off
cd /d "%~dp0"
python tools\mecanum\start_unified_web.py %*
if errorlevel 1 pause
