param([string]$Compiler = $(if ($env:ROBOFISH_CXX) { $env:ROBOFISH_CXX } else { 'g++.exe' }))
$ErrorActionPreference = 'Stop'
$project = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
$output = Join-Path $project 'build/host-attitude'
New-Item -ItemType Directory -Path $output -Force | Out-Null
$env:PATH = "$(Split-Path -Parent $Compiler);$env:PATH"
$executable = Join-Path $output 'attitude-regression.exe'
& $Compiler -std=c++17 -O1 -Wall -Wextra -Werror "-I$project/main" `
  "$PSScriptRoot/regression.cpp" "$project/main/Mode/Motion/MotionMode.cpp" `
  "$project/main/Device/Actuator/TailMotion.cpp" "$project/main/Device/Actuator/PropulsionMotion.cpp" `
  "$project/main/Device/Actuator/PectoralMotion.cpp" -o $executable
if ($LASTEXITCODE -ne 0) { throw 'Attitude regression compile failed' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'Attitude regression failed' }
