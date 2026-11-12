@echo off
setlocal
set "PYTHONHOME="
set "PYTHONPATH="
"%~dp0..\..\runtime\python.exe" -I -B "%~dp0..\..\portable.py" stop %*
if errorlevel 1 pause
