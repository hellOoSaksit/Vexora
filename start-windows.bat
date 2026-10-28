@echo off
rem Runs Vexora directly from the source tree (no installer). Builds what is missing first.
setlocal
cd /d "%~dp0"
where cmake >nul 2>nul || (echo [ERROR] CMake not found. Install it with:  winget install Kitware.CMake & goto :fail)
where npm >nul 2>nul || (echo [ERROR] Node.js not found. Install it with:  winget install OpenJS.NodeJS.LTS & goto :fail)

cmake -S . -B build -DVEXORA_BUILD_TESTS=OFF -DVEXORA_BUILD_BENCH=OFF >nul || goto :fail
cmake --build build --config Release --target vexora_sim -j || goto :fail
pushd app
if not exist node_modules (call npm install || (popd & goto :fail))
call npm start
popd
exit /b 0

:fail
echo.
echo [FAILED] Scroll up for the first error message.
pause
exit /b 1
