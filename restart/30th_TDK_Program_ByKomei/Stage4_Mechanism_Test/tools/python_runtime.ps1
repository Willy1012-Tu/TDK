# Resolve a usable interpreter after the project has been copied to another PC.
# Python venv launchers may still point to the original author's absolute path.
function Get-ProjectPythonRuntime {
    param([Parameter(Mandatory=$true)][string]$ProjectPath)
    $runtimeWorkspace = [IO.Path]::GetFullPath((Join-Path $ProjectPath '..'))
    $runtimeVenv = Join-Path $runtimeWorkspace 'tmp/platformio-venv'
    $runtimeCandidates = @(
        (Join-Path $runtimeVenv 'Scripts/python.exe'),
        (Join-Path $env:USERPROFILE '.platformio/penv/Scripts/python.exe')
    )
    foreach ($runtimeCandidate in $runtimeCandidates) {
        if (-not (Test-Path -LiteralPath $runtimeCandidate -PathType Leaf)) { continue }
        $runtimePreviousPreference = $ErrorActionPreference
        try {
            $ErrorActionPreference = 'Continue'
            & $runtimeCandidate -c 'import sys' 2>&1 | Out-Null
            $runtimeUsable = $LASTEXITCODE -eq 0
        } catch { $runtimeUsable = $false }
        finally { $ErrorActionPreference = $runtimePreviousPreference }
        if ($runtimeUsable) {
            return [PSCustomObject]@{Executable=$runtimeCandidate; PythonPath=$env:PYTHONPATH; PortableFallback=$false}
        }
    }
    $runtimePortable = Join-Path $runtimeWorkspace 'tmp/platformio-python/python.exe'
    $runtimePackages = Join-Path $runtimeVenv 'Lib/site-packages'
    if ((Test-Path -LiteralPath $runtimePortable -PathType Leaf) -and (Test-Path -LiteralPath $runtimePackages -PathType Container)) {
        $runtimePythonPath = $runtimePackages
        if ($env:PYTHONPATH) { $runtimePythonPath += [IO.Path]::PathSeparator + $env:PYTHONPATH }
        return [PSCustomObject]@{Executable=$runtimePortable; PythonPath=$runtimePythonPath; PortableFallback=$true}
    }
    throw 'No usable project Python. Install PlatformIO IDE or run tools/setup_platformio.ps1 first.'
}
