@echo off
:: OAK-Drive-Sync — Start the Pi drive controller with uv
:: Run this from anywhere; it always resolves to the project directory.

cd /d "%~dp0"

echo Starting OAK Drive Controller...
echo GUI will be available at http://localhost:8080
echo.

uv run oak-pi run

pause
