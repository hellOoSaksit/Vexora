@echo off
rem Builds Vexora for Windows: app\dist\Vexora-Setup-<version>.exe and Vexora-Portable-<version>.exe
rem Needs: CMake, Visual Studio 2022 (C++ workload or Build Tools), Node.js LTS, internet on first run.
setlocal
cd /d "%~dp0"
echo ==== Vexora: build Windows app ====

where cmake >nul 2>nul || (echo [ERROR] CMake not found. Install it with:  winget install Kitware.CMake & goto :fail)
where npm >nul 2>nul || (echo [ERROR] Node.js not found. Install it with:  winget install OpenJS.NodeJS.LTS & goto :fail)

echo.
echo [1/4] Configuring the simulation engine (C++)...
cmake -S . -B build -DVEXORA_BUILD_TESTS=OFF -DVEXORA_BUILD_BENCH=OFF || goto :fail

echo.
echo [2/4] Building the simulation engine (first time takes several minutes)...
cmake --build build --config Release --target vexora_sim -j || goto :fail

echo.
echo [3/4] Installing app dependencies...
pushd app
call npm install || (popd & goto :fail)

echo.
echo [4/4] Packaging the app...
call npm run dist:win || (popd & goto :fail)
popd

echo.
echo ==== Done ====
echo Installer : %cd%\app\dist\Vexora-Setup-*.exe
echo Portable  : %cd%\app\dist\Vexora-Portable-*.exe  (runs without installing)
start "" "%cd%\app\dist"
exit /b 0

:fail
echo.
echo [FAILED] Scroll up for the first error message.
pause
exit /b 1
