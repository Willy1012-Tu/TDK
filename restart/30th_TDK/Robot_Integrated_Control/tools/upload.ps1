param([ValidateSet('teensy40', 'teensy41')][string]$Board = 'teensy40')

Write-Host "==> Compiling for $Board..." -ForegroundColor Cyan
& (Join-Path $PSScriptRoot 'compile.ps1') -Board $Board
if ($LASTEXITCODE -ne 0) {
    Write-Error "Compilation failed with exit code $LASTEXITCODE"
    exit $LASTEXITCODE
}

$hexPath = Join-Path $PSScriptRoot "../.pio/build/$Board/firmware.hex"
if (-not (Test-Path $hexPath)) {
    Write-Error "Hex file not found at $hexPath"
    exit 1
}

$teensyCli = Join-Path $PSScriptRoot "../tmp/pio/packages/tool-teensy/teensy_loader_cli.exe"
if (-not (Test-Path $teensyCli)) {
    $teensyCli = Join-Path $env:USERPROFILE ".platformio/packages/tool-teensy/teensy_loader_cli.exe"
}
if (-not (Test-Path $teensyCli)) {
    Write-Error "teensy_loader_cli.exe not found!"
    exit 1
}

# Ensure GUI teensy.exe is not running to avoid USB communication conflicts
Stop-Process -Name teensy -Force -ErrorAction SilentlyContinue

$mcu = if ($Board -eq 'teensy41') { 'TEENSY41' } else { 'TEENSY40' }
Write-Host "==> Flashing to $mcu via teensy_loader_cli..." -ForegroundColor Green
Write-Host "(If the board does not auto-reboot, please press the reset button on Teensy)" -ForegroundColor Yellow

& $teensyCli "--mcu=$mcu" -v -s -w $hexPath
if ($LASTEXITCODE -eq 0) {
    Write-Host "==> [SUCCESS] Upload complete! Board rebooted." -ForegroundColor Green
} else {
    Write-Host "==> [FAILED] Upload failed with exit code $LASTEXITCODE" -ForegroundColor Red
}
exit $LASTEXITCODE
