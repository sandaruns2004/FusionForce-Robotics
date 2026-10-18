param([string]$WebotsExe = '')
$ErrorActionPreference = 'Stop'
$worldPath = Join-Path $PSScriptRoot 'worlds\runner4_white_line.wbt'
if (-not $WebotsExe) {
    $candidates = @(
        (Join-Path $env:ProgramFiles 'Webots\msys64\mingw64\bin\webots.exe'),
        (Join-Path $env:ProgramFiles 'Webots\webots.exe')
    )
    $command = Get-Command webots.exe -ErrorAction SilentlyContinue
    if ($command) { $candidates += $command.Source }
    $WebotsExe = $candidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
}
if (-not $WebotsExe -or -not (Test-Path -LiteralPath $WebotsExe)) {
    throw 'Webots was not found. Install Webots, then open worlds/runner4_white_line.wbt from File > Open World, or pass -WebotsExe with its full path.'
}
# Launch the interactive simulator requested by the user.
Start-Process -FilePath $WebotsExe -ArgumentList ('"' + $worldPath + '"')
