@echo off
cd /d "%~dp0"
python tools\mecanum\start_router_gui.py %*
if errorlevel 1 pause
