param([switch]$BuildWorker)
$ErrorActionPreference = 'Stop'

# Run all ESP-IDF environment changes in a child PowerShell process.
if (-not $BuildWorker) {
    $workerShell = (Get-Process -Id $PID).Path
    & $workerShell -NoProfile -ExecutionPolicy Bypass -File $PSCommandPath -BuildWorker
    if ($LASTEXITCODE -ne 0) { throw "ESP-IDF build failed: $LASTEXITCODE" }
    return
}

$project = $PSScriptRoot
$idfRoot = 'D:/ESP32/ESP-IDF-v5.3.2'
$toolsRoot = 'D:/ESP32/.espressif'
$pythonScripts = Join-Path $toolsRoot 'python_env/idf5.3_py3.13_env/Scripts'
$python = Join-Path $pythonScripts 'python.exe'
$cmake = 'D:/CMAKE/bin/cmake.exe'
$ninja = 'D:/Ninja/ninja.exe'
$export = Join-Path $idfRoot 'export.ps1'
foreach ($required in @($python, $cmake, $ninja, $export)) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) {
        throw "Required installed tool is missing: $required"
    }
}

$env:IDF_PATH = $idfRoot
$env:IDF_TOOLS_PATH = $toolsRoot
$env:IDF_PYTHON_ENV_PATH = Split-Path -Parent $pythonScripts
$env:IDF_COMPONENT_MANAGER = '0'
$env:PYTHONNOUSERSITE = '1'
$env:PATH = "$pythonScripts;D:/CMAKE/bin;D:/Ninja;" + $env:PATH
. $export
if ($LASTEXITCODE -ne 0) { throw "ESP-IDF environment export failed: $LASTEXITCODE" }
# Keep the confirmed CMake/Ninja ahead of any optional IDF tool versions.
$env:PATH = "D:/CMAKE/bin;D:/Ninja;" + $env:PATH

Push-Location -LiteralPath $project
try {
    # Components are local; this command only configures/compiles this project.
    & $python (Join-Path $idfRoot 'tools/idf.py') -B (Join-Path $project 'build') `
        -DIDF_COMPONENT_MANAGER=0 build
    if ($LASTEXITCODE -ne 0) { throw "ESP-IDF compilation failed: $LASTEXITCODE" }
    $application = Join-Path $project 'build/robotfish.bin'
    if (-not (Test-Path -LiteralPath $application -PathType Leaf)) {
        throw "Application image was not generated: $application"
    }
    $artifacts = Join-Path $project 'artifacts'
    New-Item -ItemType Directory -Path $artifacts -Force | Out-Null
    $timestamp = Get-Date -Format 'yyyyMMdd-HHmmss'
    $otaImage = Join-Path $artifacts "RobotFish-ESP32S3-OTA-$timestamp.bin"
    Copy-Item -LiteralPath $application -Destination $otaImage
    $digest = (Get-FileHash -LiteralPath $otaImage -Algorithm SHA256).Hash.ToLowerInvariant()
    Write-Output "OTA application: $otaImage"
    Write-Output "Bytes: $((Get-Item -LiteralPath $otaImage).Length)"
    Write-Output "SHA256: $digest"
} finally {
    Pop-Location
}
