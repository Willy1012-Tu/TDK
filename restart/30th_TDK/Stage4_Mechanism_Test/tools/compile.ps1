param([ValidateSet('teensy40', 'teensy41')][string]$Board = 'teensy40')
$logDirectory = Join-Path $PSScriptRoot '../.pio/logs'
New-Item -ItemType Directory -Path $logDirectory -Force | Out-Null
& (Join-Path $PSScriptRoot 'pio.ps1') run -e $Board 2>&1 | Tee-Object -FilePath (Join-Path $logDirectory "$Board.log")
exit $LASTEXITCODE
