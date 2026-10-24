param([string]$Compiler = 'C:/msys64/ucrt64/bin/g++.exe')
$ErrorActionPreference = 'Stop'
$project = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
$temporary = Join-Path $project 'build-host/radio-board'
New-Item -ItemType Directory -Path $temporary -Force | Out-Null
$env:PATH = (Split-Path -Parent $Compiler) + ';' + $env:PATH
$executable = Join-Path $temporary 'board-regression.exe'
& $Compiler -std=c++17 -O1 -Wall -Wextra -Werror `
  "-I$PSScriptRoot/stubs" "-I$project/main" `
  "$PSScriptRoot/board_regression.cpp" `
  "$project/main/bsp.cpp" -o $executable
if ($LASTEXITCODE -ne 0) { throw "Board compile failed: $LASTEXITCODE" }
& $executable
if ($LASTEXITCODE -ne 0) { throw "Board regression failed: $LASTEXITCODE" }
$radioExecutable = Join-Path $temporary 'lora-regression.exe'
& $Compiler -std=c++17 -O1 -Wall -Wextra -Werror `
  "-I$PSScriptRoot/stubs" "-I$project/main" `
  "$PSScriptRoot/lora_regression.cpp" "$project/main/bsp.cpp" `
  "$project/main/BSP/LoRaUart.cpp" "$project/main/Service/LoRa/LoRaService.cpp" `
  "$project/main/Service/LoRa/LoraProtocol.cpp" -o $radioExecutable
if ($LASTEXITCODE -ne 0) { throw "LoRa compile failed: $LASTEXITCODE" }
& $radioExecutable
if ($LASTEXITCODE -ne 0) { throw "LoRa regression failed: $LASTEXITCODE" }
