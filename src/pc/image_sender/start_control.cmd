@echo off
setlocal
cd /d "%~dp0" || exit /b 1

set "PYTHON_EXE=%APPDATA%\uv\python\cpython-3.11-windows-x86_64-none\python.exe"
if exist "%PYTHON_EXE%" goto run_python
set "PYTHON_EXE=%USERPROFILE%\.local\bin\python3.12.exe"
if exist "%PYTHON_EXE%" goto run_python
where py.exe >nul 2>nul
if errorlevel 1 goto use_path
py -3-64 control_web.py
goto finish

:use_path
python control_web.py
goto finish

:run_python
"%PYTHON_EXE%" control_web.py

:finish
set "RESULT=%ERRORLEVEL%"
if not "%RESULT%"=="0" pause
exit /b %RESULT%
