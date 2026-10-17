param([string]$Compiler = 'C:/msys64/ucrt64/bin/g++.exe')
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
$ownerExecutable = Join-Path $output 'owner-regression.exe'
& $Compiler -std=c++17 -O1 -Wall -Wextra -Werror `
  "-I$PSScriptRoot/stubs" "-I$project/tests/radio_board/stubs" "-I$project/main" `
  "$PSScriptRoot/owner_regression.cpp" "$project/main/robotfish.cpp" `
  "$project/main/Mode/Motion/MotionMode.cpp" "$project/main/Device/Actuator/TailMotion.cpp" `
  "$project/main/Device/Actuator/PropulsionMotion.cpp" "$project/main/Device/Actuator/PectoralMotion.cpp" `
  -o $ownerExecutable
if ($LASTEXITCODE -ne 0) { throw 'Control owner compile failed' }
& $ownerExecutable
if ($LASTEXITCODE -ne 0) { throw 'Control owner checks failed' }
& $ownerExecutable deadlines
if ($LASTEXITCODE -ne 0) { throw 'Control owner deadline checks failed' }
