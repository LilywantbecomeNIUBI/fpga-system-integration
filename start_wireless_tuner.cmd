@echo off
cd /d "%~dp0"
if not exist config\local_motor_wireless.json (
  echo Missing config\local_motor_wireless.json. See docs\WIRELESS_TUNING_20261030.md
  pause
  exit /b 1
)
python host\motor_tuner.py --live --home --config config\local_motor_wireless.json %*
