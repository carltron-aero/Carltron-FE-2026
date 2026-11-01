param([int]$Port = 8765, [switch]$NoBrowser)
$ErrorActionPreference = 'Stop'
$cameraPython = (Get-Command python.exe -ErrorAction Stop).Source
$cameraUrl = "http://127.0.0.1:$Port"
try { $existing = Invoke-RestMethod "$cameraUrl/api/session" -TimeoutSec 1 } catch { $existing = $null }
if (-not $existing -or $existing.app -ne 'Carltron Camera Control') {
    $outputDir = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../../../output/tracking/camera-control'))
    New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
    $stamp = Get-Date -Format 'yyyyMMddTHHmmss'
    Start-Process -FilePath $cameraPython -ArgumentList @('"' + (Join-Path $PSScriptRoot 'server.py') + '"', '--port', $Port) -WindowStyle Hidden -RedirectStandardOutput (Join-Path $outputDir "$stamp-server.log") -RedirectStandardError (Join-Path $outputDir "$stamp-server-error.log")
    $ready = $false
    for ($attempt = 0; $attempt -lt 50; $attempt++) {
        Start-Sleep -Milliseconds 200
        try { $probe = Invoke-RestMethod "$cameraUrl/api/session" -TimeoutSec 1; if ($probe.app -eq 'Carltron Camera Control') { $ready = $true; break } } catch {}
    }
    if (-not $ready) { throw "The local camera app did not start. See the server logs in $outputDir" }
}
if (-not $NoBrowser) {
    $cameraEdge = Join-Path ${env:ProgramFiles(x86)} 'Microsoft/Edge/Application/msedge.exe'
    if (-not (Test-Path -LiteralPath $cameraEdge)) { $cameraEdge = (Get-Command msedge.exe -ErrorAction Stop).Source }
    Start-Process -FilePath $cameraEdge -ArgumentList $cameraUrl
}
Write-Output "Carltron Camera Control: $cameraUrl"
