param([string]$RuntimeProjectPath, [string]$NodePath)
$ErrorActionPreference = 'Stop'
$testProjectPath = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
if (-not $RuntimeProjectPath) { $RuntimeProjectPath = $testProjectPath }
. (Join-Path $testProjectPath 'tools/python_runtime.ps1')
$testRuntime = Get-ProjectPythonRuntime -ProjectPath $RuntimeProjectPath
if (-not $NodePath) {
    $testNodeCommand = Get-Command node.exe -ErrorAction SilentlyContinue
    if ($testNodeCommand) { $NodePath = $testNodeCommand.Source }
    else { $NodePath = Join-Path $env:USERPROFILE '.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node.exe' }
}
if (-not (Test-Path -LiteralPath $NodePath -PathType Leaf)) {
    throw 'Node.js is required for the frontend regression. Pass -NodePath to a node executable.'
}
$testPreviousPython = $env:ROBOT_TEST_PYTHON
$testPreviousNode = $env:ROBOT_TEST_NODE
$testPreviousPackages = $env:PYTHONPATH
$testLogDirectory = Join-Path $testProjectPath '.pio/logs'
New-Item -ItemType Directory -Path $testLogDirectory -Force | Out-Null
try {
    $env:ROBOT_TEST_PYTHON = $testRuntime.Executable
    $env:ROBOT_TEST_NODE = $NodePath
    $env:PYTHONPATH = $testRuntime.PythonPath
    $ErrorActionPreference = 'Continue'
    & (Join-Path $PSScriptRoot 'run_tests.cmd') 2>&1 |
        ForEach-Object { $_.ToString() } |
        Tee-Object -FilePath (Join-Path $testLogDirectory 'host-tests.log')
    $testExitCode = $LASTEXITCODE
} finally {
    $env:ROBOT_TEST_PYTHON = $testPreviousPython
    $env:ROBOT_TEST_NODE = $testPreviousNode
    $env:PYTHONPATH = $testPreviousPackages
}
exit $testExitCode