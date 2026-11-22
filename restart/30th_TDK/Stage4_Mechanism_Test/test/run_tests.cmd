@echo off
setlocal
if not defined STAGE4_PYTHON (
 powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_tests.ps1"
 exit /b
)
set "PATH=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer;%PATH%"
if defined VSCMD_VER goto compiler_ready
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
 echo ERROR: Install Visual Studio Build Tools with Desktop development with C++.
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
if not defined STAGE4_PYTHON set "STAGE4_PYTHON=%CD%\..\tmp\platformio-venv\Scripts\python.exe"
if not defined STAGE4_LIBRARY_DIR set "STAGE4_LIBRARY_DIR=%CD%\..\Library"
if not exist "%STAGE4_PYTHON%" set "STAGE4_PYTHON=%USERPROFILE%\.platformio\penv\Scripts\python.exe"
if not exist "%STAGE4_PYTHON%" (
 echo ERROR: Set STAGE4_PYTHON to the Python executable used by PlatformIO.
 exit /b 1
)
if not exist ".pio\python-deps\yaml\__init__.py" (
 echo ERROR: PyYAML is missing; use the prepared PlatformIO Python environment.
 exit /b 1
)
"%STAGE4_PYTHON%" test\test_generate_steps.py
if errorlevel 1 exit /b 1
"%STAGE4_PYTHON%" test\test_web_gui.py
if errorlevel 1 exit /b 1
"%STAGE4_PYTHON%" tools\generate_steps.py stage4_steps.yaml .pio\host-tests\generated\stage4_steps.generated.h
if errorlevel 1 exit /b 1
"%STAGE4_PYTHON%" tools\generate_steps.py stage3_steps.yaml .pio\host-tests\generated\stage3_steps.generated.h
if errorlevel 1 exit /b 1
"%STAGE4_PYTHON%" tools\generate_steps.py stage2_steps.yaml .pio\host-tests\generated\stage2_steps.generated.h
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /EHsc /utf-8 /W4 /WX /D_CRT_SECURE_NO_WARNINGS /I"test\stubs" /I"include" /I".pio\host-tests\generated" test\stage4_regression.cpp src\test_controller.cpp src\serial_protocol.cpp /Fo".pio\host-tests\\" /Fe".pio\host-tests\stage4_regression.exe"
if errorlevel 1 exit /b 1
for %%T in (boot_dual_ack_barrier boot_second_enable_retry boot_dual_home_6_first boot_dual_home_7_first boot_dual_home_partial_send boot_dual_home_queue_retry boot_dual_home_start_timeout boot_dual_home_finish_timeout boot_dual_home_stop servo_failure timer_failure thread_failure boot_delay boot_delay_stop boot_first_ack_retry boot_late_enable_ack boot_ack_fixed_deadline boot_ack_last_millisecond boot_retry_rejected_ack boot_retry_stop boot_retry_queue_rejected boot_diagnostics boot_only_active_hardware boot_disabled_absent boot_ready_dual_home boot_without_usb home_motion_not_retried boot_feedback_timeout boot_feedback_failure boot_feedback_failure_latched boot_enable_failure boot_enable_failure_latched boot_enable_timeout boot_bad_reply boot_feedback_queue_retry boot_feedback_queue_timeout boot_enable_queue_retry boot_enable_queue_timeout home_frame home_failure home_start_timeout home_late_start home_finish_timeout home_queue_retry home_queue_timeout home_stop only_stage4 full_cycle run_all wait_start_order start_timeout late_start finish_timeout late_finish limit_failure limit_failure_5 move_failure bad_move_reply queue_retry queue_timeout home busy stop_and_no_resume stop_ramp stop_retry disable_retry serial_valid serial_removed serial_invalid no_usb ramp_finishes_first negative_position_bytes) do (
 ".pio\host-tests\stage4_regression.exe" %%T
 if errorlevel 1 exit /b 1
)
cl /nologo /std:c++17 /EHsc /utf-8 /W4 /WX /D_CRT_SECURE_NO_WARNINGS /I"test\stubs" /I"include" /I".pio\host-tests\generated" test\stage3_regression.cpp src\test_controller.cpp src\serial_protocol.cpp /Fo".pio\host-tests\\" /Fe".pio\host-tests\stage3_regression.exe"
if errorlevel 1 exit /b 1
for %%T in (standby_next standby_run standby_home standby_switch standby_debug_reject standby_ram_reject standby_ram_noop disabled_debug disabled_runtime stage2_steps stage2_repeat stage2_run stage2_home five_servo_timers seven_device_barrier stage2_ram stage2_duplicate_parallel standby_stop stop_five_pwm stage2_activation_once stage2_auto_stop debug_id7_independent stage2_ram_auto_motor) do (
 ".pio\host-tests\stage3_regression.exe" %%T
 if errorlevel 1 exit /b 1
)
"%STAGE4_PYTHON%" tools\generate_steps.py test\action_steps.yaml .pio\host-tests\action-generated\stage4_steps.generated.h
if errorlevel 1 exit /b 1
"%STAGE4_PYTHON%" tools\generate_steps.py stage3_steps.yaml .pio\host-tests\action-generated\stage3_steps.generated.h
if errorlevel 1 exit /b 1
"%STAGE4_PYTHON%" tools\generate_steps.py stage2_steps.yaml .pio\host-tests\action-generated\stage2_steps.generated.h
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /EHsc /utf-8 /W4 /WX /D_CRT_SECURE_NO_WARNINGS /I"test\stubs" /I"include" /I".pio\host-tests\action-generated" test\action_regression.cpp src\test_controller.cpp src\serial_protocol.cpp /Fo".pio\host-tests\\" /Fe".pio\host-tests\action_regression.exe"
if errorlevel 1 exit /b 1
for %%T in (single_actions_progress delay_serial_stop parallel_waits_all parallel_motor_timeout) do (
 ".pio\host-tests\action_regression.exe" %%T
 if errorlevel 1 exit /b 1
)
cl /nologo /std:c++17 /EHsc /utf-8 /W4 /D_CRT_SECURE_NO_WARNINGS /I"test\stubs" /I"%STAGE4_LIBRARY_DIR%" test\library_regression.cpp "%STAGE4_LIBRARY_DIR%\servoxxd_can.cpp" "%STAGE4_LIBRARY_DIR%\trajectory.cpp" "%STAGE4_LIBRARY_DIR%\alg_pid.cpp" "%STAGE4_LIBRARY_DIR%\drv_math.cpp" /Fo".pio\host-tests\\" /Fe".pio\host-tests\library_regression.exe"
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /EHsc /utf-8 /W4 /WX /D_CRT_SECURE_NO_WARNINGS /I"test\stubs" /I"include" /I".pio\host-tests\generated" test\web_regression.cpp src\test_controller.cpp src\serial_protocol.cpp /Fo".pio\host-tests\\" /Fe".pio\host-tests\web_regression.exe"
if errorlevel 1 exit /b 1
for %%T in (web_status debug_validation debug_stepper debug_gohome debug_independent_servos debug_motor_and_servo plan_transaction plan_parallel_validation plan_incomplete_barrier plan_timeout_stop plan_protocol plan_254_steps debug_three_axes debug_node_protocol plan_three_devices plan_switch_parks plan_profile_protocol serial_stage_aliases) do (
 ".pio\host-tests\web_regression.exe" %%T
 if errorlevel 1 exit /b 1
)
"%STAGE4_PYTHON%" test\check_status_fixture.py
if errorlevel 1 exit /b 1
".pio\host-tests\library_regression.exe"
exit /b %errorlevel%

