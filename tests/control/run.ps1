param([string]$Compiler = $(if ($env:ROBOFISH_CXX) { $env:ROBOFISH_CXX } else { 'g++.exe' }))
$ErrorActionPreference = 'Stop'
$project = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
$output = Join-Path $project 'build/host-control'
New-Item -ItemType Directory -Path $output -Force | Out-Null
$env:PATH = "$(Split-Path -Parent $Compiler);$env:PATH"
$executable = Join-Path $output 'control-regression.exe'
& $Compiler -std=c++17 -O1 -Wall -Wextra "-I$(Join-Path $project 'main')" `
  (Join-Path $PSScriptRoot 'regression.cpp') -o $executable
if ($LASTEXITCODE -ne 0) { throw 'Control state compile failed' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'Control state checks failed' }
$headingExecutable = Join-Path $output 'heading-target-regression.exe'
& $Compiler -std=c++17 -O1 -Wall -Wextra -Werror "-I$project/main" "$PSScriptRoot/heading_target_regression.cpp" -o $headingExecutable
if ($LASTEXITCODE -ne 0) { throw 'Heading target state compile failed' }
& $headingExecutable
if ($LASTEXITCODE -ne 0) { throw 'Heading target state checks failed' }
$ownerExecutable = Join-Path $output 'owner-regression.exe'
& $Compiler -std=c++17 -O1 -Wall -Wextra -Werror `
  "-I$PSScriptRoot/stubs" "-I$project/tests/radio_board/stubs" "-I$project/main" `
  "$PSScriptRoot/lora_owner_regression.cpp" "$project/main/robotfish.cpp" `
  "$project/main/BSP/LoRaUart.cpp" "$project/main/BSP/ImuUart.cpp" `
  "$project/main/Service/LoRa/LoRaService.cpp" "$project/main/Service/LoRa/LoraProtocol.cpp" `
  "$project/main/Mode/Motion/MotionMode.cpp" "$project/main/Device/Actuator/TailMotion.cpp" `
  "$project/main/Device/Actuator/PropulsionMotion.cpp" "$project/main/Device/Actuator/PectoralMotion.cpp" `
  -o $ownerExecutable
if ($LASTEXITCODE -ne 0) { throw 'Control owner compile failed' }
foreach ($scenario in @('normal','clock_race','hold','wifi_hold','recovery','fault','attitude','yaw_idle','wifi_yaw_idle','yaw_full','wifi_yaw_full','heading_target','wifi_heading_target')) {
  & $ownerExecutable $scenario
  if ($LASTEXITCODE -ne 0) { throw "LoRa/IMU owner checks failed ($scenario)" }
}

$dualExecutable = Join-Path $output 'dual-link-regression.exe'
& $Compiler -std=c++17 -O1 -Wall -Wextra -Werror "-I$project/main" `
  "$PSScriptRoot/dual_link_regression.cpp" "$project/main/Service/LoRa/LoraProtocol.cpp" -o $dualExecutable
if ($LASTEXITCODE -ne 0) { throw 'Dual link compile failed' }
& $dualExecutable
if ($LASTEXITCODE -ne 0) { throw 'Dual link arbitration checks failed' }
