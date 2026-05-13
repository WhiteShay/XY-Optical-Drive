@echo off
REM Quick launcher for UserJourney1 test suite
REM Run this batch file to execute the test with default settings

setlocal enabledelayedexpansion

echo.
echo ================================================================================
echo XY Optical Drive - UserJourney1 Test Launcher
echo ================================================================================
echo.

REM Check if running as Administrator
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo ERROR: This script must be run as Administrator
    echo.
    echo Please:
    echo 1. Right-click on Command Prompt
    echo 2. Select "Run as administrator"
    echo 3. Navigate to this directory
    echo 4. Run: run-test.bat
    echo.
    pause
    exit /b 1
)

REM Get the script directory
set SCRIPT_DIR=%~dp0
echo Test Directory: %SCRIPT_DIR%
echo.

REM Run the PowerShell test runner
echo Launching UserJourney1 Test Suite...
echo.

powershell -NoProfile -ExecutionPolicy Bypass -File "%SCRIPT_DIR%Run-UserJourney1.ps1"

set TEST_RESULT=%ERRORLEVEL%

echo.
echo Test execution completed with exit code: %TEST_RESULT%
echo.

if exist "%SCRIPT_DIR%test-report.json" (
    echo Report generated: %SCRIPT_DIR%test-report.json
    echo.
    echo Press any key to view the report...
    pause >nul
    
    REM Try to open report with default JSON viewer
    start "" "%SCRIPT_DIR%test-report.json"
)

exit /b %TEST_RESULT%
