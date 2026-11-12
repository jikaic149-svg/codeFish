param([string]$Compiler = $(if ($env:ROBOFISH_CXX) { $env:ROBOFISH_CXX } else { 'g++.exe' }),
      [string]$JsonSource = $(Join-Path $env:IDF_PATH 'components/json/cJSON'))
$ErrorActionPreference = 'Stop'
$project = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
$output = Join-Path $project 'build/host-control'
New-Item -ItemType Directory -Path $output -Force | Out-Null
$env:PATH = "$(Split-Path -Parent $Compiler);$env:PATH"
$jsonObject = Join-Path $output 'calibration-json.o'
& (Join-Path (Split-Path -Parent $Compiler) 'gcc.exe') -std=c99 -O1 "-I$JsonSource" -c "$JsonSource/cJSON.c" -o $jsonObject
if ($LASTEXITCODE -ne 0) { throw 'Calibration cJSON compile failed' }
$executable = Join-Path $output 'calibration-regression.exe'
& $Compiler -std=c++17 -O1 -Wall -Wextra -Werror "-I$project/main" "-I$JsonSource" `
  "$PSScriptRoot/calibration_regression.cpp" "$project/main/Service/Wifi/ControlProtocol.cpp" `
  "$project/main/Mode/Motion/MotionMode.cpp" "$project/main/Device/Actuator/TailMotion.cpp" `
  "$project/main/Device/Actuator/PropulsionMotion.cpp" "$project/main/Device/Actuator/PectoralMotion.cpp" `
  $jsonObject -o $executable
if ($LASTEXITCODE -ne 0) { throw 'Calibration regression compile failed' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'Calibration regression failed' }
