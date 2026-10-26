@echo off
setlocal
set "SCRIPT_DIR=%~dp0"

where python >nul 2>nul
if %ERRORLEVEL% equ 0 (
    python "%SCRIPT_DIR%build_matrix.py" %*
    exit /b %ERRORLEVEL%
)

where py >nul 2>nul
if %ERRORLEVEL% equ 0 (
    py -3 "%SCRIPT_DIR%build_matrix.py" %*
    exit /b %ERRORLEVEL%
)

where python3 >nul 2>nul
if %ERRORLEVEL% equ 0 (
    python3 "%SCRIPT_DIR%build_matrix.py" %*
    exit /b %ERRORLEVEL%
)

echo ERROR: Python 3 could not be found in your PATH. Please install Python from python.org.
exit /b 1
