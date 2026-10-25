# Pass through PlatformIO arguments, e.g. tools/pio.ps1 run -e teensy41.
$ErrorActionPreference = 'Stop'
$projectPath = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$runtimeProjectPath = $projectPath
if ($env:STAGE4_RUNTIME_PROJECT) { $runtimeProjectPath = [IO.Path]::GetFullPath($env:STAGE4_RUNTIME_PROJECT) }
$workspacePath = [IO.Path]::GetFullPath((Join-Path $runtimeProjectPath '..'))
. (Join-Path $PSScriptRoot 'python_runtime.ps1')
$runtime = Get-ProjectPythonRuntime -ProjectPath $runtimeProjectPath
# GCC's Windows runtime cannot locate its libraries through a non-ASCII
# installation path. Build the original project through a temporary alias.
function Get-AsciiBuildPath {
    param([Parameter(Mandatory=$true)][string]$Path)
    $fullPath = [IO.Path]::GetFullPath($Path)
    if ($fullPath -notmatch '[^\x00-\x7F]') { return $fullPath }
    $aliasRoot = Join-Path ([IO.Path]::GetTempPath()) 'stage4-platformio-paths'
    if ($aliasRoot -match '[^\x00-\x7F]') {
        throw 'Set TEMP to an ASCII directory before compiling this project.'
    }
    $hasher = [Security.Cryptography.SHA256]::Create()
    try {
        $digest = $hasher.ComputeHash([Text.Encoding]::UTF8.GetBytes($fullPath.ToLowerInvariant()))
        $aliasName = ([BitConverter]::ToString($digest)).Replace('-', '').Substring(0, 16)
    } finally { $hasher.Dispose() }
    New-Item -ItemType Directory -Path $aliasRoot -Force | Out-Null
    $aliasPath = Join-Path $aliasRoot $aliasName
    if (Test-Path -LiteralPath $aliasPath) {
        $existing = Get-Item -LiteralPath $aliasPath -Force
        $target = @($existing.Target)[0]
        if ($existing.LinkType -ne 'Junction' -or -not $target -or
            [IO.Path]::GetFullPath($target) -ne $fullPath) {
            throw "Build path alias already exists with a different target: $aliasPath"
        }
    } else {
        New-Item -ItemType Junction -Path $aliasPath -Target $fullPath | Out-Null
    }
    return $aliasPath
}
function Get-AsciiPackageCache {
    param([Parameter(Mandatory=$true)][string]$Path)
    $fullPath = [IO.Path]::GetFullPath($Path)
    if ($fullPath -notmatch '[^\x00-\x7F]') { return $fullPath }
    $cacheRoot = Join-Path ([IO.Path]::GetTempPath()) 'stage4-platformio-cache'
    if ($cacheRoot -match '[^\x00-\x7F]') {
        throw 'Set TEMP to an ASCII directory before compiling this project.'
    }
    # The compiler resolves junctions back to its physical installation path,
    # so copy the already installed packages once instead of linking them.
    $signature = $fullPath.ToLowerInvariant()
    $installedPackages = @()
    if (Test-Path -LiteralPath $fullPath -PathType Container) {
        $installedPackages = @(Get-ChildItem -LiteralPath $fullPath -Directory | Sort-Object Name)
    }
    foreach ($package in $installedPackages) {
        $signature += "`n$($package.Name)"
        foreach ($manifest in @('package.json', '.piopm', '.version')) {
            $manifestPath = Join-Path $package.FullName $manifest
            if (Test-Path -LiteralPath $manifestPath -PathType Leaf) {
                $signature += (Get-FileHash -LiteralPath $manifestPath -Algorithm SHA256).Hash
            }
        }
    }
    $hasher = [Security.Cryptography.SHA256]::Create()
    try {
        $digest = $hasher.ComputeHash([Text.Encoding]::UTF8.GetBytes($signature))
        $cacheName = ([BitConverter]::ToString($digest)).Replace('-', '').Substring(0, 16)
    } finally { $hasher.Dispose() }
    $cachePath = Join-Path $cacheRoot $cacheName
    $packagesPath = Join-Path $cachePath 'packages'
    $completePath = Join-Path $cachePath 'complete.txt'
    if (-not (Test-Path -LiteralPath $completePath -PathType Leaf)) {
        New-Item -ItemType Directory -Path $cachePath -Force | Out-Null
        if (Test-Path -LiteralPath $fullPath -PathType Container) {
            Write-Host 'Copying existing PlatformIO packages to the ASCII temporary build cache.'
            Copy-Item -LiteralPath $fullPath -Destination $cachePath -Recurse -Force
        } else {
            New-Item -ItemType Directory -Path $packagesPath -Force | Out-Null
        }
        [IO.File]::WriteAllText($completePath, $signature, [Text.Encoding]::UTF8)
    }
    return $packagesPath
}
$buildProjectPath = Get-AsciiBuildPath -Path $projectPath
$packagePath = Get-AsciiPackageCache -Path (Join-Path $workspacePath 'tmp/pio/packages')
$previousCorePath = $env:PLATFORMIO_CORE_DIR
$previousPackagePath = $env:PLATFORMIO_PACKAGES_DIR
$previousTelemetry = $env:PLATFORMIO_SETTING_ENABLE_TELEMETRY
$previousPythonPath = $env:PYTHONPATH
$env:PLATFORMIO_CORE_DIR = Join-Path $workspacePath 'tmp/pio'
$env:PLATFORMIO_PACKAGES_DIR = $packagePath
$env:PLATFORMIO_SETTING_ENABLE_TELEMETRY = 'No'
$env:PYTHONPATH = $runtime.PythonPath
Push-Location -LiteralPath $buildProjectPath
try {
    $ErrorActionPreference = 'Continue'
    # Teensy prints the memory report on stderr even after a successful build.
    & $runtime.Executable -m platformio @args 2>&1 | ForEach-Object { $_.ToString() }
    $pioExitCode = $LASTEXITCODE
} finally {
    Pop-Location
    $env:PLATFORMIO_CORE_DIR = $previousCorePath
    $env:PLATFORMIO_PACKAGES_DIR = $previousPackagePath
    $env:PLATFORMIO_SETTING_ENABLE_TELEMETRY = $previousTelemetry
    $env:PYTHONPATH = $previousPythonPath
}
exit $pioExitCode
