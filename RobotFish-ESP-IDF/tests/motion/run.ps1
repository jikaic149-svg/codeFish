param(
  [string]$Compiler = 'C:/msys64/ucrt64/bin/g++.exe',
  [string]$LegacyFirmware = 'C:/Users/rlyoulo/Desktop/robofish_new/embedding/f4/firmware'
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
foreach ($variant in @('current', 'original-f405')) {
  $includeRoot = if ($variant -eq 'current') { Join-Path $project 'main' } else { $LegacyFirmware }
  $inputs = $sources | ForEach-Object { Join-Path $includeRoot $_ }
  $executable = Join-Path $output "$variant.exe"
  & $Compiler -std=c++17 -O1 -Wall -Wextra "-I$includeRoot" $test @inputs -o $executable
  if ($LASTEXITCODE -ne 0) { throw "$variant motion compile failed" }
  & $executable | Set-Content -LiteralPath (Join-Path $output "$variant.csv") -Encoding UTF8
  if ($LASTEXITCODE -ne 0) { throw "$variant motion execution failed" }
}
$actual = Join-Path $output 'current.csv'
$reference = Join-Path $output 'original-f405.csv'
if ((Get-FileHash -LiteralPath $actual).Hash -ne (Get-FileHash -LiteralPath $reference).Hash) {
  $expectedLines = Get-Content -LiteralPath $reference
  $actualLines = Get-Content -LiteralPath $actual
  for ($index = 0; $index -lt [Math]::Min($expectedLines.Count, $actualLines.Count); ++$index) {
    if ($expectedLines[$index] -ne $actualLines[$index]) {
      throw "Motion differs at row $($index + 1): old=$($expectedLines[$index]); current=$($actualLines[$index])"
    }
  }
  throw 'Motion trajectory lengths differ'
}
Write-Output 'PASS: copied ESP-IDF motion matches original F405 source byte for byte (1,620 scenarios / 51,840 control ticks)'
