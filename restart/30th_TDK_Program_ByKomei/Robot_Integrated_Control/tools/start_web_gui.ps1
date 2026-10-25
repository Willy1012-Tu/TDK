param([int]$Port=8765,[string]$BindAddress='0.0.0.0')
$ErrorActionPreference='Stop'
$projectPath=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
. (Join-Path $PSScriptRoot 'python_runtime.ps1')
$runtime=Get-ProjectPythonRuntime -ProjectPath $projectPath
$previousPythonPath=$env:PYTHONPATH
$env:PYTHONPATH=$runtime.PythonPath
try {
    if (-not (Test-Path -LiteralPath (Join-Path $projectPath '.pio/python-deps/yaml/__init__.py'))) {
        & $runtime.Executable -m pip install --disable-pip-version-check --no-cache-dir --target (Join-Path $projectPath '.pio/python-deps') -r (Join-Path $PSScriptRoot 'requirements-yaml.txt')
        if ($LASTEXITCODE -ne 0) { throw 'Could not install YAML dependency.' }
    }
    & $runtime.Executable (Join-Path $PSScriptRoot 'web_gui.py') --host $BindAddress --port $Port
    $webExitCode=$LASTEXITCODE
} finally {
    $env:PYTHONPATH=$previousPythonPath
}
exit $webExitCode
