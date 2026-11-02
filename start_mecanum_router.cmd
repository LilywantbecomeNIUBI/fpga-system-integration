@echo off
cd /d "%~dp0"
python host\mecanum\gui.py --transport router %*
