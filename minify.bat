@echo off
where python >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    python tools/minify_html.py main/homepage_full.html main/homepage.html
) else (
    powershell -ExecutionPolicy Bypass -File tools/minify_html.ps1 main/homepage_full.html main/homepage.html
)
echo HTML minification completed successfully!
