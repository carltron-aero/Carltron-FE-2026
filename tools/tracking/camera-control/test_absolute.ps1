# Payload validation only. This test never creates a camera session.
$ErrorActionPreference = 'Stop'
Add-Type -Path (Join-Path $PSScriptRoot 'CameraControl.cs')
$range = [Carltron.CameraControl.AbsoluteInfo]::new()
$range.Writable = $true
$range.PanMin = -324000; $range.PanMax = 324000; $range.PanStep = 3600
$range.TiltMin = -144000; $range.TiltMax = 108000; $range.TiltStep = 3600
$payload = [Carltron.CameraControl.Session]::AbsolutePayload($range, 43200, -25200)
if ([BitConverter]::ToInt32($payload, 0) -ne 43200 -or [BitConverter]::ToInt32($payload, 4) -ne -25200) { throw 'Payload mismatch' }
foreach ($target in @(@(324001,0), @(0,-144001), @(1800,0), @(0,1800))) {
    $rejected = $false
    try { [Carltron.CameraControl.Session]::AbsolutePayload($range, $target[0], $target[1]) | Out-Null } catch { $rejected = $true }
    if (-not $rejected) { throw 'Invalid target accepted' }
}
$range.Writable = $false
$rejected = $false
try { [Carltron.CameraControl.Session]::AbsolutePayload($range, 0, 0) | Out-Null } catch { $rejected = $true }
if (-not $rejected) { throw 'Read-only range accepted' }
Write-Output 'Absolute payload tests passed; no hardware commands.'
