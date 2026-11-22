$ErrorActionPreference = 'Stop'
$workspacePath = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$pythonPath = Join-Path $workspacePath 'tmp/platformio-python/python.exe'
$venvPath = Join-Path $workspacePath 'tmp/platformio-venv'
$venvPython = Join-Path $venvPath 'Scripts/python.exe'
if (-not (Test-Path -LiteralPath $pythonPath)) {
    $bootstrapPath = Join-Path $workspacePath 'tmp/platformio-bootstrap'
    $archivePath = Join-Path $bootstrapPath 'python-3.13.9-amd64.zip'
    New-Item -ItemType Directory -Path $bootstrapPath -Force | Out-Null
    if (-not (Test-Path -LiteralPath $archivePath)) {
        Invoke-WebRequest -UseBasicParsing -Uri 'https://www.python.org/ftp/python/3.13.9/python-3.13.9-amd64.zip' -OutFile $archivePath
    }
    Expand-Archive -LiteralPath $archivePath -DestinationPath (Split-Path $pythonPath) -Force
}
if (-not (Test-Path -LiteralPath $venvPython)) {
    & $pythonPath -m venv $venvPath
    if ($LASTEXITCODE -ne 0) { throw 'Could not create the project Python environment.' }
}
& $venvPython -m pip install --disable-pip-version-check --cache-dir (Join-Path $workspacePath 'tmp/pip-cache') -r (Join-Path $PSScriptRoot 'requirements-platformio.txt')
if ($LASTEXITCODE -ne 0) { throw 'Could not install PlatformIO Core.' }
Write-Host 'PlatformIO is ready. Run tools/pio.ps1 run from this project.'
