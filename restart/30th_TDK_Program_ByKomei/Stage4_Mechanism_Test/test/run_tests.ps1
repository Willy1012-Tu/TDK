param([string]$RuntimeProjectPath, [string]$LibraryDirectory)
$ErrorActionPreference = 'Stop'
$testProjectPath = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
if (-not $RuntimeProjectPath) { $RuntimeProjectPath = $testProjectPath }
. (Join-Path $testProjectPath 'tools/python_runtime.ps1')
$testRuntime = Get-ProjectPythonRuntime -ProjectPath $RuntimeProjectPath
if (-not $LibraryDirectory) {
    $LibraryDirectory = [IO.Path]::GetFullPath((Join-Path $RuntimeProjectPath '../Library'))
}
$testPreviousPython = $env:STAGE4_PYTHON
$testPreviousPackages = $env:PYTHONPATH
$testPreviousLibrary = $env:STAGE4_LIBRARY_DIR
try {
    $env:STAGE4_PYTHON = $testRuntime.Executable
    $env:PYTHONPATH = $testRuntime.PythonPath
    $env:STAGE4_LIBRARY_DIR = $LibraryDirectory
    $ErrorActionPreference = 'Continue'
    & (Join-Path $PSScriptRoot 'run_tests.cmd') 2>&1 | ForEach-Object { $_.ToString() }
    $testExitCode = $LASTEXITCODE
} finally {
    $env:STAGE4_PYTHON = $testPreviousPython
    $env:PYTHONPATH = $testPreviousPackages
    $env:STAGE4_LIBRARY_DIR = $testPreviousLibrary
}
exit $testExitCode
