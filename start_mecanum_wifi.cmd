@echo off
cd /d "%~dp0"
python tools\mecanum\start_router_gui.py --wifi-profile "CMCC-rDtN-5G" %*
if errorlevel 1 pause
