@echo off
chcp 65001 >nul
py -3.12 "%~dp0tools\pwm_ssh_gui.py" %*
if errorlevel 1 pause
