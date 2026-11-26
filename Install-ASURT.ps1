param(
    [string]$Repository=$(if ($env:ASURT_REPOSITORY) { $env:ASURT_REPOSITORY } else { 'asurt-fsai/Simulation-27' }),
    [string]$Manifest,
    [string]$Catalog,
    [string]$Branch='main',
    [string]$Distribution='Ubuntu-22.04',
    [ValidateSet('ask','all','none')][string]$Maps='ask',
    [switch]$SkipFirewall
)
$ErrorActionPreference='Stop'
if (!$Manifest -and $Repository -notmatch '^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$') {
    throw 'Supply the GitHub owner and project with -Repository OWNER/REPO.'
}
if ($Distribution -notmatch '^[A-Za-z0-9_.-]+$') { throw 'Invalid WSL distribution name.' }
if ([System.Runtime.InteropServices.RuntimeInformation]::OSArchitecture.ToString() -ne 'X64') {
    throw 'ASURT Windows requires an x64 PC.'
}
$local=$Manifest -and $Manifest -notmatch '^https://'
if ($local) { $Manifest=(Resolve-Path -LiteralPath $Manifest).ProviderPath }

$wslPath=Join-Path $env:SystemRoot 'System32/wsl.exe'
if ([Environment]::Is64BitOperatingSystem -and ![Environment]::Is64BitProcess) {
    $wslPath=Join-Path $env:SystemRoot 'Sysnative/wsl.exe'
}

function Invoke-AsurtWsl {
    $output=(& $wslPath @args 2>$null | Out-String) -replace "\x00",''
    if ($LASTEXITCODE -ne 0) { return $null }
    return $output.Trim()
}

$entries=$null
if (Test-Path -LiteralPath $wslPath) {
    $entries=Invoke-AsurtWsl --list --quiet
    $wslStatus=Invoke-AsurtWsl --status
}
if (!$entries -and !$wslStatus) {
    if (Test-Path -LiteralPath $wslPath) {
        Write-Host 'Installing WSL and the Windows virtualization prerequisites.'
        $bootstrap=Start-Process -FilePath $wslPath -ArgumentList '--install','--no-distribution' -Verb RunAs -WindowStyle Hidden -Wait -PassThru
    } else {
        Write-Host 'Enabling WSL and Virtual Machine Platform.'
        $enable='$ErrorActionPreference="Stop"; Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux,VirtualMachinePlatform -All -NoRestart | Out-Null'
        $encoded=[Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($enable))
        $bootstrap=Start-Process -FilePath 'powershell.exe' -ArgumentList '-NoProfile','-EncodedCommand',$encoded -Verb RunAs -WindowStyle Hidden -Wait -PassThru
    }
    if ($bootstrap.ExitCode -notin 0,3010) { throw "WSL setup returned $($bootstrap.ExitCode). Check Windows virtualization and Windows Update." }
    if ($bootstrap.ExitCode -eq 3010 -or !(Test-Path -LiteralPath $wslPath) -or
        (!(Invoke-AsurtWsl --status) -and !(Invoke-AsurtWsl --list --quiet))) {
        Write-Host 'Restart Windows, then rerun this installer to continue with Ubuntu 22.04.'
        exit 3010
    }
    $entries=Invoke-AsurtWsl --list --quiet
}
if (($entries -split '\r?\n').Trim() -notcontains $Distribution) {
    if ($Distribution -ne 'Ubuntu-22.04') {
        throw 'Use Ubuntu-22.04 for a new installation. -Distribution can select an existing Ubuntu 22.04 distribution.'
    }
    & $wslPath --set-default-version 2
    if ($LASTEXITCODE -ne 0) { throw 'WSL2 could not be enabled. Restart Windows and rerun the installer.' }
    & $wslPath --install --distribution Ubuntu-22.04 --no-launch
    if ($LASTEXITCODE -notin 0,3010) {
        & $wslPath --install --distribution Ubuntu-22.04 --web-download --no-launch
    }
    if ($LASTEXITCODE -eq 3010) {
        Write-Host 'Restart Windows, then rerun this installer to finish Ubuntu setup.'
        exit 3010
    }
    if ($LASTEXITCODE -ne 0) { throw "Ubuntu 22.04 installation returned $LASTEXITCODE." }
    Write-Host 'Complete the Ubuntu first-run prompt and type exit when you reach its shell.'
    & $wslPath -d $Distribution
    if ($LASTEXITCODE -ne 0) { throw 'Ubuntu first-run setup did not finish. Rerun the installer after creating the Ubuntu user.' }
}
$version=Invoke-AsurtWsl -d $Distribution -- uname -r
if ($version -notmatch 'WSL2|microsoft-standard') {
    throw "$Distribution must use WSL2. Run wsl --set-version $Distribution 2, then rerun the installer."
}
$osRelease=Invoke-AsurtWsl -d $Distribution -- cat /etc/os-release
$architecture=Invoke-AsurtWsl -d $Distribution -- uname -m
if ($osRelease -notmatch '(?m)^ID="?ubuntu"?\r?$' -or $osRelease -notmatch '(?m)^VERSION_ID="?22\.04"?\r?$' -or $architecture -ne 'x86_64') {
    throw "$Distribution must be Ubuntu 22.04 x86-64. Select the matching distro with -Distribution."
}
$linuxHome=Invoke-AsurtWsl -d $Distribution -- printenv HOME
if ($linuxHome -notmatch '^/home/[^/]+$') {
    Write-Host 'Complete the Ubuntu first-run prompt and type exit when you reach its shell.'
    & $wslPath -d $Distribution
    $linuxHome=Invoke-AsurtWsl -d $Distribution -- printenv HOME
    if ($linuxHome -notmatch '^/home/[^/]+$') { throw "Open $Distribution and set up its desktop user, then rerun." }
}
$root=Join-Path $env:LOCALAPPDATA 'ASURT'
$download=Join-Path $root 'installer'
New-Item -ItemType Directory -Path $download -Force | Out-Null
$vc=Get-ItemProperty -Path 'HKLM:\SOFTWARE\Microsoft\VisualStudio\14.0\VC\Runtimes\x64' -ErrorAction SilentlyContinue
if (!$vc -or $vc.Installed -ne 1) {
    $redist=Join-Path $download 'vc_redist.x64.exe'
    Invoke-WebRequest -UseBasicParsing -Uri 'https://aka.ms/vs/17/release/vc_redist.x64.exe' -OutFile $redist
    $auth=Get-AuthenticodeSignature -FilePath $redist
    if ($auth.Status -ne 'Valid' -or $auth.SignerCertificate.Subject -notmatch 'Microsoft Corporation') {
        throw 'The Microsoft Visual C++ redistributable signature could not be verified.'
    }
    $install=Start-Process -FilePath $redist -ArgumentList '/install','/quiet','/norestart' -Verb RunAs -WindowStyle Hidden -Wait -PassThru
    if ($install.ExitCode -notin 0,3010,1638) { throw "Microsoft Visual C++ setup returned $($install.ExitCode)." }
}
$script=Join-Path $download 'Install-ASURT.sh'
$embeddedLinuxInstaller='__ASURT_EMBEDDED_LINUX_INSTALLER__'
if ($embeddedLinuxInstaller -ne ('__ASURT_' + 'EMBEDDED_LINUX_INSTALLER__')) {
    [IO.File]::WriteAllBytes($script, [Convert]::FromBase64String($embeddedLinuxInstaller))
} else {
    $sourceShell=Join-Path $PSScriptRoot 'Install-ASURT.sh'
    if (!(Test-Path -LiteralPath $sourceShell -PathType Leaf)) { throw 'Keep the two source installers together or use the published Windows installer.' }
    Copy-Item -LiteralPath $sourceShell -Destination $script -Force
}
function Convert-AsurtPath([string]$Path) {
    $forwardPath=$Path.Replace('\','/')
    $value=((& $wslPath -d $Distribution --exec wslpath -u $forwardPath) | Out-String).Trim()
    if ($LASTEXITCODE -ne 0 -or !$value) { throw "Could not translate $Path into WSL." }
    return $value
}
$working=Convert-AsurtPath $root
$entry=Convert-AsurtPath $script
$source=$Manifest
if ($local) { $source=Convert-AsurtPath $Manifest }
$installArgs=@('--platform','windows','--windows-root',$working,'--maps',$Maps,'--branch',$Branch,'--no-shortcuts')
if ($source) { $installArgs+=@('--manifest',$source) }
if ($Repository) { $installArgs+=@('--repository',$Repository) }
if ($Catalog) {
    if ($Catalog -notmatch '^https://') { $Catalog=Convert-AsurtPath (Resolve-Path -LiteralPath $Catalog).ProviderPath }
    $installArgs+=@('--catalog',$Catalog)
}
$oldWslEnv=$env:WSLENV
try {
    if ($env:GH_TOKEN -and !(($env:WSLENV -split ':') -match '^GH_TOKEN(?:/|$)')) {
        $env:WSLENV=(@($env:WSLENV,'GH_TOKEN/u') | Where-Object { $_ }) -join ':'
    }
    & $wslPath -d $Distribution -- /bin/bash $entry @installArgs
    if ($LASTEXITCODE -ne 0) { throw 'ASURT installation did not finish. Rerun it to continue.' }
} finally { $env:WSLENV=$oldWslEnv }

if (!$SkipFirewall) {
    $addr=(((& $wslPath -d $Distribution -- hostname -I) | Out-String).Trim() -split '\s+') |
        Where-Object { $_ -match '^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$' } | Select-Object -First 1
    $runtime=Invoke-AsurtWsl -d $Distribution -- cat "$linuxHome/.local/share/asurt/current/Simulation/CARLA/Runtime/Active/asurt-win64-flat.txt"
    $runtimeParent=Join-Path $root 'runtimes'
    if (!$runtime -or (Split-Path -Parent $runtime) -ine $runtimeParent -or (Split-Path -Leaf $runtime) -notmatch '^[a-f0-9]{64}$') {
        throw 'The installed Windows runtime path could not be verified.'
    }
    $game=Join-Path $runtime 'WindowsNoEditor/CarlaUE4/Binaries/Win64/CarlaUE4-Win64-Shipping.exe'
    if ($addr -and (Test-Path -LiteralPath $game -PathType Leaf)) {
        $rule='ASURT-CARLA-WSL-' + [System.Security.Principal.WindowsIdentity]::GetCurrent().User.Value
        $previous=Get-NetFirewallRule -Name $rule -ErrorAction SilentlyContinue
        $filter=$previous | Get-NetFirewallApplicationFilter -ErrorAction SilentlyContinue
        $address=$previous | Get-NetFirewallAddressFilter -ErrorAction SilentlyContinue
        if (!$previous -or $filter.Program -ne $game -or $addr -notin @($address.RemoteAddress)) {
            $escapedGame=$game.Replace("'","''")
            $command=('$ErrorActionPreference="Stop"; $old=Get-NetFirewallRule -Name ''{0}'' -ErrorAction SilentlyContinue; if ($old) {{ $old | Remove-NetFirewallRule }}; New-NetFirewallRule -Name ''{0}'' -DisplayName ''ASURT CARLA from WSL'' -Direction Inbound -Action Allow -Program ''{1}'' -RemoteAddress ''{2}'' -Protocol TCP -LocalPort 2100,2101,2102 | Out-Null' -f $rule,$escapedGame,$addr)
            $encoded=[Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($command))
            $firewall=Start-Process -FilePath 'powershell.exe' -ArgumentList '-NoProfile','-EncodedCommand',$encoded -Verb RunAs -WindowStyle Hidden -Wait -PassThru
            if ($firewall.ExitCode -ne 0) { throw 'Could not configure Windows CARLA access from WSL. Rerun the installer.' }
        }
    }
}

$shortcutActions=[ordered]@{
    'ASURT FlatAsphalt'='start'
    'ASURT FlatAsphalt WSLg'='start-wslg'
    'ASURT FlatAsphalt noVNC'='start-novnc'
    'ASURT noVNC Viewer'='novnc-viewer'
    'ASURT Stop Simulator'='stop'
    'ASURT CARLA System Manager'='manager'
    'ASURT CARLA Map Packs'='map-packs'
    'ASURT CARLA Tools'='tools'
    'ASURT Cone Track Importer'='track'
    'ASURT Weather Editor'='weather'
    'ASURT Trackdraw'='trackdraw'
    'ASURT Perception RViz'='rviz'
    'ASURT ZED RViz'='zed-rviz'
    'ASURT SLAM RViz'='slam-rviz'
    'ASURT SLAM Diagnostics'='slam-diagnostics'
    'ASURT Planning RViz'='planning-rviz'
    'ASURT Control RViz'='control-rviz'
    'ASURT Track Control RViz'='track-control-rviz'
}
$native=New-Object -ComObject WScript.Shell
foreach ($directory in @([Environment]::GetFolderPath('Desktop'),[Environment]::GetFolderPath('Programs'))) {
    $folder=Join-Path $directory 'ASURT CARLA'
    New-Item -ItemType Directory -Path $folder -Force | Out-Null
    foreach ($name in $shortcutActions.Keys) {
        $link=$native.CreateShortcut((Join-Path $folder "$name.lnk"))
        $link.TargetPath=Join-Path $env:SystemRoot 'System32/wsl.exe'
        $link.Arguments='-d {0} -- /bin/bash "{1}/.local/share/asurt/current/bin/asurt" {2}' -f $Distribution,$linuxHome,$shortcutActions[$name]
        $icon=Join-Path $root "shortcut-icons/$($shortcutActions[$name]).ico"
        if (Test-Path -LiteralPath $icon -PathType Leaf) {
            $link.IconLocation="$icon,0"
        }
        $link.WindowStyle=7
        $link.Save()
    }
    $workspacePath='\\wsl.localhost\' + $Distribution + $linuxHome.Replace('/', '\') + '\.local\share\asurt\workspace'
    $workspaceLink=$native.CreateShortcut((Join-Path $folder 'ASURT Workspace.lnk'))
    $workspaceLink.TargetPath=Join-Path $env:SystemRoot 'explorer.exe'
    $workspaceLink.Arguments='"' + $workspacePath + '"'
    $workspaceLink.Save()
}
Write-Host 'ASURT is installed. Open ASURT CARLA on the desktop and launch FlatAsphalt.'
Write-Host "Open ASURT Workspace in the same folder to find Modules, tools, and Assets."
Write-Host $workspacePath
