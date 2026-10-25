@echo off
setlocal
set "PATH=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer;%PATH%"
if defined VSCMD_VER goto compiler_ready
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
 echo ERROR: Install Visual Studio Build Tools with Desktop development with C++.
 exit /b 1
)
for /f "usebackq tokens=*" %%V in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSROOT=%%V"
if not defined VSROOT exit /b 1
call "%VSROOT%\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
:compiler_ready
cd /d "%~dp0.."
if not exist ".pio\host-tests" mkdir ".pio\host-tests"
cl /nologo /std:c++17 /EHsc /utf-8 /W4 /WX /I"test\stubs" /I"include" test\chassis_regression.cpp /Fo".pio\host-tests\\" /Fe".pio\host-tests\chassis_regression.exe"
if errorlevel 1 exit /b 1
for %%T in (math boot_routing center_hold motion rc_loss rc_invalid heartbeat_timeout axis_error wrong_frames arm_movement arm_timeout late_arm_reply arm_send_failure motion_send_failure closed_loop_exit recovery rollover without_usb) do (
 ".pio\host-tests\chassis_regression.exe" %%T
 if errorlevel 1 exit /b 1
)
exit /b 0
