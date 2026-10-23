param(
  [string]$Compiler = 'C:/msys64/ucrt64/bin/g++.exe'
)
$ErrorActionPreference = 'Stop'
$project = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
$output = Join-Path $project 'build/host-motion'
New-Item -ItemType Directory -Path $output -Force | Out-Null
$compilerDirectory = Split-Path -Parent $Compiler
$env:PATH = "$compilerDirectory;$env:PATH"
$test = Join-Path $PSScriptRoot 'regression.cpp'
$sources = @('Mode/Motion/MotionMode.cpp', 'Device/Actuator/TailMotion.cpp',
  'Device/Actuator/PropulsionMotion.cpp', 'Device/Actuator/PectoralMotion.cpp')
$includeRoot = Join-Path $project 'main'
$inputs = $sources | ForEach-Object { Join-Path $includeRoot $_ }
$executable = Join-Path $output 'current.exe'
& $Compiler -std=c++17 -O1 -Wall -Wextra -Werror "-I$includeRoot" $test @inputs -o $executable
if ($LASTEXITCODE -ne 0) { throw 'Motion compile failed' }
& $executable | Set-Content -LiteralPath (Join-Path $output 'current.csv') -Encoding UTF8
if ($LASTEXITCODE -ne 0) { throw 'Motion behavior and combined trajectory checks failed' }
Write-Output 'PASS: tuned motion behavior, slew, steering, 1.5 Hz gait and 51,840 bounded control ticks'
