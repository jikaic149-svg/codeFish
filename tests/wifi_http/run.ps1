param(
  [string]$Compiler = $(if ($env:ROBOFISH_CXX) { $env:ROBOFISH_CXX } else { 'g++.exe' }),
  [string]$JsonSource = $(Join-Path $env:IDF_PATH 'components/json/cJSON')
)
$ErrorActionPreference = 'Stop'
$project = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
$temporary = Join-Path $project 'build-host/wifi-http'
New-Item -ItemType Directory -Path $temporary -Force | Out-Null
$env:PATH = (Split-Path -Parent $Compiler) + ';' + $env:PATH
$cCompiler = Join-Path (Split-Path -Parent $Compiler) 'gcc.exe'
$jsonObject = Join-Path $temporary 'cJSON.o'
& $cCompiler -std=c99 -O1 -Wall -Wextra -Werror "-I$JsonSource" -c "$JsonSource/cJSON.c" -o $jsonObject
if ($LASTEXITCODE -ne 0) { throw "cJSON compile failed: $LASTEXITCODE" }
$executable = Join-Path $temporary 'wifi-http-regression.exe'
& $Compiler -std=c++17 -O1 -Wall -Wextra -Werror "-I$project/main" "-I$JsonSource" `
  "$PSScriptRoot/regression.cpp" "$project/main/Service/Wifi/ControlProtocol.cpp" $jsonObject -o $executable
if ($LASTEXITCODE -ne 0) { throw "Wi-Fi HTTP compile failed: $LASTEXITCODE" }
& $executable
if ($LASTEXITCODE -ne 0) { throw "Wi-Fi HTTP regression failed: $LASTEXITCODE" }
