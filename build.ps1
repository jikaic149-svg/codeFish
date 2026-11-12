$ErrorActionPreference = 'Stop'
& (Join-Path $PSScriptRoot '../runtime/python.exe') -I -B (Join-Path $PSScriptRoot '../portable.py') build
exit $LASTEXITCODE
