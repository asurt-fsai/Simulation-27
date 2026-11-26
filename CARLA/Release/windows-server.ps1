param(
    [Parameter(Mandatory)][ValidateSet('start','stop','ready')][string]$Action,
    [Parameter(Mandatory)][string]$Runtime,
    [string]$Map='/Game/Carla/Maps/FlatAsphalt',
    [string]$Identity
)
$ErrorActionPreference='Stop'
$exe=(Resolve-Path -LiteralPath (Join-Path $Runtime 'WindowsNoEditor/CarlaUE4/Binaries/Win64/CarlaUE4-Win64-Shipping.exe')).ProviderPath
if ($Action -eq 'start') {
    if ($Map -notmatch '^/Game/(?:[A-Za-z0-9_]+/)+[A-Za-z0-9_]+$') { throw 'Invalid map package path.' }
    if (Get-NetTCPConnection -State Listen -ErrorAction SilentlyContinue | Where-Object LocalPort -in 2100,2101,2102) {
        throw 'CARLA ports 2100–2102 are in use.'
    }
    $section='[/Script/EngineSettings.GameMapsSettings]'
    $defaults="-ini:Engine:${section}:GameDefaultMap=$Map,${section}:ServerDefaultMap=$Map"
    $arguments=@($Map,$defaults,'-RenderOffScreen','-quality-level=Low','-unattended','-nosplash','-nosound',
                 '-carla-rpc-port=2100','-carla-streaming-port=2101','-carla-secondary-port=2102')
    $process=Start-Process -FilePath $exe -ArgumentList $arguments -WorkingDirectory (Split-Path -Parent $exe) -WindowStyle Hidden -PassThru
    "$($process.Id):$($process.StartTime.ToUniversalTime().Ticks)"
    exit
}
if ($Identity -notmatch '^([0-9]+):([0-9]+)$') { throw 'Missing simulator process identity.' }
$process=Get-Process -Id ([int]$Matches[1]) -ErrorAction SilentlyContinue
$owned=$process -and $process.Path -eq $exe -and $process.StartTime.ToUniversalTime().Ticks -eq [long]$Matches[2]
if ($Action -eq 'ready') {
    if ($owned) {exit 0}
    exit 1
}
if ($owned) {Stop-Process -Id $process.Id -ErrorAction Stop}
