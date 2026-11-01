$ErrorActionPreference = 'Stop'
[Console]::InputEncoding = [Text.UTF8Encoding]::new($false)
[Console]::OutputEncoding = [Text.UTF8Encoding]::new($false)
Add-Type -Path (Join-Path $PSScriptRoot 'CameraControl.cs')
$cameraSession = $null
try {
    while ($null -ne ($requestLine = [Console]::ReadLine())) {
        try {
            $request = $requestLine | ConvertFrom-Json
            if ($request.action -eq 'devices') {
                $result = @([Carltron.CameraControl.Session]::Devices())
            } elseif ($request.action -eq 'select') {
                if ($cameraSession) { $cameraSession.Dispose(); $cameraSession = $null }
                $cameraSession = [Carltron.CameraControl.Session]::new([int]$request.index)
                $result = $cameraSession.Read()
            } else {
                if (-not $cameraSession) { throw 'Select a connected camera first.' }
                switch ($request.action) {
                    'state' { $result = $cameraSession.Read() }
                    'set' { $result = $cameraSession.Set([string]$request.group, [int]$request.id, [int]$request.value, [bool]$request.auto) }
                    'jog' { $cameraSession.Jog([int]$request.pan, [int]$request.tilt, [int]$request.duration, [int]$request.speed); $result = @{ stopped = $true } }
                    'stop' { $cameraSession.Stop(); $result = @{ stopped = $true } }
                    'extended' { $result = $cameraSession.SetExtended([int]$request.id, [int]$request.value) }
                    'absolute' { $result = $cameraSession.SetAbsolute([int]$request.pan, [int]$request.tilt) }
                    default { throw 'Unknown action.' }
                }
            }
            [Console]::WriteLine((@{ ok = $true; result = $result } | ConvertTo-Json -Depth 12 -Compress))
        } catch {
            [Console]::WriteLine((@{ ok = $false; error = $_.Exception.Message } | ConvertTo-Json -Compress))
        }
    }
} finally {
    if ($cameraSession) { $cameraSession.Dispose() }
}
