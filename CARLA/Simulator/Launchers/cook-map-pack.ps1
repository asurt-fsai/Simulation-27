param(
    [Parameter(Mandatory)][ValidatePattern('^[a-z][a-z0-9-]{0,47}$')][string]$Id,
    [Parameter(Mandatory)][string]$Label,
    [Parameter(Mandatory)][string]$Map,
    [Parameter(Mandatory)][string]$OpenDrive,
    [Parameter(Mandatory)][ValidateCount(4,4)][double[]]$Spawn,
    [Parameter(Mandatory)][string]$Destination,
    [ValidateSet('WindowsNoEditor','LinuxNoEditor')][string[]]$Platforms = @('WindowsNoEditor','LinuxNoEditor'),
    [string]$Workspace=$env:ASURT_WORKSPACE,
    [string]$Project,
    [string]$WindowsBase,
    [string]$LinuxBase
)
$ErrorActionPreference = 'Stop'
if (!$Workspace) {
    $Workspace = Join-Path $PSScriptRoot '../../..'
}
$root = (Resolve-Path -LiteralPath $Workspace).ProviderPath
$carla = Join-Path $root 'Simulation/CARLA'
$folder = $Id.Replace('-', '_')
if ($Map -notmatch "^/Game/ASURTMaps/$folder/[A-Za-z0-9_]+$") {
    throw "Place the map and its map-specific assets under /Game/ASURTMaps/$folder before saving."
}
if ($Id -in @('flat-asphalt','silverstone')) { throw 'Choose a unique map pack ID.' }
if (!$Project) {
    $Project = Join-Path $root 'reconstruction/carla-asurt-windows-20261024/Unreal/CarlaUE4/CarlaUE4.uproject'
}
$Project = (Resolve-Path -LiteralPath $Project).Path
$OpenDrive = (Resolve-Path -LiteralPath $OpenDrive).Path
$source = Join-Path (Split-Path -Parent $Project) ('Content/' + $Map.Substring(6) + '.umap')
if (!(Test-Path -LiteralPath $source -PathType Leaf)) { throw "Map asset missing: $source" }

$bases = @{}
if ('WindowsNoEditor' -in $Platforms) {
    if (!$WindowsBase) {
        $active = (Get-Content -LiteralPath (Join-Path $carla 'Runtime/Active/asurt-win64-flat.txt') -Raw).Trim()
        $WindowsBase = Join-Path $active 'WindowsNoEditor'
    }
    $bases['WindowsNoEditor'] = (Resolve-Path -LiteralPath $WindowsBase).ProviderPath
}
if ('LinuxNoEditor' -in $Platforms) {
    if (!$LinuxBase) {
        $LinuxBase = (& wsl.exe -d Ubuntu-22.04 -- bash -c 'wslpath -w "$(readlink -f "$HOME/ASURT/Simulation/CARLA/Runtime/Active/asurt-linux-flat")"').Trim()
        if ($LASTEXITCODE -ne 0) { throw 'Could not locate the Linux FlatAsphalt runtime.' }
    }
    $bases['LinuxNoEditor'] = (Resolve-Path -LiteralPath $LinuxBase).ProviderPath
}

$Destination = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($Destination)
if (Test-Path -LiteralPath $Destination) { throw "Use a new output directory: $Destination" }
New-Item -ItemType Directory -Path $Destination | Out-Null
$cooked = Join-Path $Destination 'Cooked'
$editor = Join-Path $root 'UE426CARLA/Engine/Binaries/Win64/UE4Editor-Cmd.exe'
$deps = Join-Path $Destination 'map-dependencies.json'
$collector = Join-Path $PSScriptRoot 'collect-map-dependencies.py'
$env:ASURT_MAP_DEPENDENCIES = $deps
$env:ASURT_COOK_MAP = $Map
try {
    & $editor $Project '-run=pythonscript' "-script=$collector" '-unattended' '-nop4' '-nosplash' '-nosound' '-nullrhi' `
        '-DisablePlugins=MagicLeap,MagicLeapAR,MagicLeapMedia,MagicLeapPassableWorld,USDImporter' "-abslog=$Destination/dependencies.log" *> "$Destination/dependencies-console.log"
    if (!(Test-Path -LiteralPath $deps -PathType Leaf)) { throw "Could not read map dependencies. See $Destination/dependencies.log" }
} finally {
    Remove-Item Env:ASURT_MAP_DEPENDENCIES,Env:ASURT_COOK_MAP
}
$log = Join-Path $Destination 'cook.log'
& $editor $Project '-run=cook' "-Map=$Map" '-CookSinglePackage' "-TargetPlatform=$($Platforms -join '+')" `
    "-OutputDir=$cooked/[Platform]" '-unversioned' '-unattended' '-nop4' '-nosplash' '-nosound' '-stdout' `
    '-DisablePlugins=MagicLeap,MagicLeapAR,MagicLeapMedia,MagicLeapPassableWorld,USDImporter' "-abslog=$log" *> "$Destination/cook-console.log"
if ($LASTEXITCODE -ne 0) { throw "Map cook failed. See $log" }

$archive = Join-Path $Destination "$Id.zip"
$argsMap = @((Join-Path $PSScriptRoot 'map_packs.py'),'build',$archive,'--id',$Id,'--label',$Label,
             '--map',$Map,'--xodr',$OpenDrive,'--dependencies',$deps,'--spawn')
$argsMap += $Spawn | ForEach-Object { $_.ToString([Globalization.CultureInfo]::InvariantCulture) }
foreach ($platform in $Platforms) {
    $argsMap += @('--platform',$platform,(Join-Path $cooked $platform),$bases[$platform])
}
& python @argsMap
if ($LASTEXITCODE -ne 0) { throw 'The cooked map could not be packaged against the selected FlatAsphalt base.' }
