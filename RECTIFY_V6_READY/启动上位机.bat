@echo off
cd /d "%~dp0"
if exist "host_app\RECTIFY_HOST\RECTIFY_HOST.exe" goto packed
set "RECTIFY_PYTHON=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if exist "%RECTIFY_PYTHON%" (
  "%RECTIFY_PYTHON%" run_host.py
) else (
  py -3 run_host.py
)
if errorlevel 1 pause
goto end
:packed
"host_app\RECTIFY_HOST\RECTIFY_HOST.exe"
:end
