@echo off
cd /d "%~dp0"
set "RECTIFY_PYTHON=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if exist "%RECTIFY_PYTHON%" (
  "%RECTIFY_PYTHON%" tools\build_project.py
) else (
  py -3 tools\build_project.py
)
if errorlevel 1 pause
