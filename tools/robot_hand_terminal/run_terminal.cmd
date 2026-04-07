@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -STA -File "%SCRIPT_DIR%terminal.ps1" %*
exit /b %ERRORLEVEL%
