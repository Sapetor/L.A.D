@echo off
REM scripts\start-all.bat - Start all L.A.D services (Windows)

setlocal enabledelayedexpansion

REM Get the directory where this script is located
set "SCRIPT_DIR=%~dp0"
REM Go to the parent directory (L.A.D root)
cd /d "%SCRIPT_DIR%.."

echo ========================================
echo    L.A.D Platform - Starting Services
echo ========================================
echo.
echo Working directory: %CD%
echo.

REM Detect IP using Node.js
echo [1/4] Detecting local IP address...
cd /d "%CD%\AVEDU\avedu"
for /f "tokens=*" %%i in ('node scripts\detect-ip.js 2^>^&1 ^| findstr /C:"Detected local IP:"') do (
    for %%a in (%%i) do set IP=%%a
)
cd /d "%SCRIPT_DIR%.."

if "%IP%"=="" (
    echo Warning: Could not auto-detect IP, reading from config
    for /f "tokens=2 delims=:," %%a in ('type config\ip_config.json ^| findstr exposed_ip') do (
        set IP=%%a
        set IP=!IP:"=!
        set IP=!IP: =!
    )
)

echo [OK] Using IP: %IP%
echo.

REM Start ROS Docker
echo [2/4] Starting ROS 2 Docker...
cd /d "%SCRIPT_DIR%..\qcar_docker"
docker compose up -d
cd /d "%SCRIPT_DIR%.."
echo [OK] ROS services running on:
echo    - rosbridge: ws://%IP%:9090
echo    - static server: http://%IP%:7000
echo.

REM Start Django Backend
echo [3/4] Starting Django backend...
cd /d "%SCRIPT_DIR%..\LAD\lad"
if not exist "..\\.venv" (
    echo Creating virtual environment...
    python -m venv ..\.venv
    call ..\\.venv\Scripts\activate
    pip install -r requirements.txt
) else (
    call ..\\.venv\Scripts\activate
)

python manage.py migrate --no-input
start "Django Backend" python manage.py runserver 0.0.0.0:8000
cd /d "%SCRIPT_DIR%.."
echo [OK] Django API running on: http://%IP%:8000
echo.

REM Start React Frontend
echo [4/4] Starting React frontend...
cd /d "%SCRIPT_DIR%..\AVEDU\avedu"
start "React Frontend" cmd /k npm start
cd /d "%SCRIPT_DIR%.."
echo.

echo ========================================
echo    All services started successfully!
echo ========================================
echo.
echo Access your application:
echo    Local:   http://localhost:3000
echo    Network: http://%IP%:3000
echo.
echo To stop services, close the terminal windows
echo or run: scripts\stop-all.bat
echo.

pause
