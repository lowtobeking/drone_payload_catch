@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0connect_a_ssh.ps1" %*
exit /b %ERRORLEVEL%
