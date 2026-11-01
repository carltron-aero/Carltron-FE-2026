param([int]$Port = 8770)
$ErrorActionPreference = 'Stop'
$replayPython = (Get-Command python.exe -ErrorAction Stop).Source
& $replayPython (Join-Path $PSScriptRoot 'server.py') --port $Port
