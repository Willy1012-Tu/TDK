@echo off
setlocal
set "PATH=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer;%PATH%"
if not defined ROBOT_TEST_PYTHON (
 powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_tests.ps1"
 exit /b
)
if defined VSCMD_VER goto compiler_ready
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
 echo ERROR: MSVC Desktop development with C++ is required.
 exit /b 1
)
for /f "usebackq tokens=*" %%V in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSROOT=%%V"
if not defined VSROOT (
 echo ERROR: No MSVC C++ toolchain found.
 exit /b 1
)
call "%VSROOT%\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
:compiler_ready
cd /d "%~dp0.."
if not exist ".pio\python-deps\yaml\__init__.py" (
 echo ERROR: Build the project once to prepare PyYAML.
 exit /b 1
)
"%ROBOT_TEST_PYTHON%" test\test_generate_steps.py 2>&1
if errorlevel 1 exit /b 1
"%ROBOT_TEST_PYTHON%" test\test_web_gui.py 2>&1
if errorlevel 1 exit /b 1
"%ROBOT_TEST_NODE%" test\frontend_regression.mjs 2>&1
if errorlevel 1 exit /b 1
for %%S in (1 2 3 4 5 6) do (
 "%ROBOT_TEST_PYTHON%" tools\generate_steps.py stage%%S_steps.yaml .pio\host-tests\generated\stage%%S_steps.generated.h 2>&1
 if errorlevel 1 exit /b 1
)
for %%P in (1_basket 2_vase 3_tube 4_chicken 5_bird) do (
 for /f "tokens=1,2 delims=_" %%I in ("%%P") do (
  "%ROBOT_TEST_PYTHON%" tools\generate_steps.py stage1_%%J.yaml .pio\host-tests\generated\stage1_%%J.generated.h --expected-stage 1 --expected-prop %%I 2>&1
  if errorlevel 1 exit /b 1
  for %%M in (basket vase chicken bird) do if "%%J"=="%%M" (
   "%ROBOT_TEST_PYTHON%" tools\generate_steps.py stage1_%%J.yaml include\stage1_%%J.generated.h --expected-stage 1 --expected-prop %%I 2>&1
   if errorlevel 1 exit /b 1
  )
 )
)
cl /nologo /std:c++17 /EHsc /utf-8 /W4 /WX /D_CRT_SECURE_NO_WARNINGS /DROBOT_BENCH_TEST_MODE=0 /I"test\stubs" /I"include" /I".pio\host-tests\generated" test\mechanism_regression.cpp src\mechanism_controller.cpp src\cylinder_controller.cpp /Fo".pio\host-tests\\" /Fe".pio\host-tests\mechanism_regression.exe"
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /EHsc /utf-8 /W4 /WX /D_CRT_SECURE_NO_WARNINGS /DROBOT_BENCH_TEST_MODE=0 /I"test\stubs" /I"include" /I".pio\host-tests\generated" test\chassis_rc_regression.cpp /Fo".pio\host-tests\\" /Fe".pio\host-tests\chassis_rc_regression.exe"
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /EHsc /utf-8 /W4 /WX /D_CRT_SECURE_NO_WARNINGS /DROBOT_BENCH_TEST_MODE=0 /I"test\stubs" /I"include" /I".pio\host-tests\generated" test\system_regression.cpp src\main.cpp src\can_transport.cpp src\chassis_controller.cpp src\rc_controller.cpp src\mechanism_controller.cpp src\serial_gateway.cpp src\system_safety.cpp src\cylinder_controller.cpp src\pwm_controller.cpp /Fo".pio\host-tests\\" /Fe".pio\host-tests\system_regression.exe"
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /EHsc /utf-8 /W4 /WX /D_CRT_SECURE_NO_WARNINGS /DROBOT_BENCH_TEST_MODE=0 /I"test\stubs" /I"include" /I".pio\host-tests\generated" test\chassis_motion_regression.cpp src\main.cpp src\can_transport.cpp src\chassis_controller.cpp src\rc_controller.cpp src\mechanism_controller.cpp src\serial_gateway.cpp src\system_safety.cpp src\cylinder_controller.cpp src\pwm_controller.cpp /Fo".pio\host-tests\\" /Fe".pio\host-tests\chassis_motion_regression.exe"
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /EHsc /utf-8 /W4 /WX /D_CRT_SECURE_NO_WARNINGS /DROBOT_BENCH_TEST_MODE=1 /I"test\stubs" /I"include" /I".pio\host-tests\generated" test\bench_regression.cpp src\main.cpp src\can_transport.cpp src\chassis_controller.cpp src\rc_controller.cpp src\mechanism_controller.cpp src\serial_gateway.cpp src\system_safety.cpp src\cylinder_controller.cpp src\pwm_controller.cpp /Fo".pio\host-tests\\" /Fe".pio\host-tests\bench_regression.exe"
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /EHsc /utf-8 /W4 /WX /D_CRT_SECURE_NO_WARNINGS /I"test\stubs" /I"include" /I".pio\host-tests\generated" test\bench_regression.cpp src\main.cpp src\can_transport.cpp src\chassis_controller.cpp src\rc_controller.cpp src\mechanism_controller.cpp src\serial_gateway.cpp src\system_safety.cpp src\cylinder_controller.cpp src\pwm_controller.cpp /Fo".pio\host-tests\\" /Fe".pio\host-tests\bench_default_regression.exe"
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /EHsc /utf-8 /W4 /WX /D_CRT_SECURE_NO_WARNINGS /I"test\stubs" /I"include" test\pwm_regression.cpp src\pwm_controller.cpp /Fo".pio\host-tests\\" /Fe".pio\host-tests\pwm_regression.exe"
if errorlevel 1 exit /b 1
"%ROBOT_TEST_PYTHON%" test\run_regressions.py
exit /b %errorlevel%
