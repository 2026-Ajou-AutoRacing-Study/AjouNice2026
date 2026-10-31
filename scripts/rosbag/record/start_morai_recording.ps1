#requires -Version 5.1
<#
One-click companion for a Windows MORAI host. Ordinary PowerShell, no admin:
  powershell.exe -NoProfile -File .\start_morai_recording.ps1
Logs go to a NEW Desktop\AjouNice_LAN\<timestamp>_<uuid> folder. Ctrl+C stops
measurement only, never MORAI. Copy the whole folder beside the Linux bag.
Both launchers default to SessionId lan_debug; use a unique override when needed.
Optional: -Duration 30 (short Windows smoke test), -NoGpu, -SessionId name.
If the sibling collector is absent, download its pinned, SHA256-checked version
into the new log folder. No installation, policy bypass, firewall, clock, NIC,
MORAI, or existing-output changes. Missing counters remain unavailable, not zero.
Actual Windows execution still needs a short smoke test before a competition run.
#>
[CmdletBinding()]
param(
    [ValidateNotNullOrEmpty()][string]$SessionId = 'lan_debug',
    [ValidateRange(0,864000)][double]$Duration = 0,
    [ValidateRange(1,3600)][double]$Interval = 2,
    [switch]$NoGpu
)
$ErrorActionPreference = 'Stop'
if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) { throw 'Windows only' }
foreach ($number in @($Duration,$Interval)) {
    if ([double]::IsNaN($number) -or [double]::IsInfinity($number)) { throw 'Finite intervals required' }
}
$desktop = [Environment]::GetFolderPath([Environment+SpecialFolder]::DesktopDirectory)
if ([string]::IsNullOrWhiteSpace($desktop) -or -not [IO.Directory]::Exists($desktop)) {
    $desktop = [Environment]::GetFolderPath([Environment+SpecialFolder]::UserProfile)
}
if ([string]::IsNullOrWhiteSpace($desktop) -or -not [IO.Directory]::Exists($desktop)) {
    throw 'No existing Desktop or user-profile directory found; no files created.'
}
$base = Join-Path $desktop 'AjouNice_LAN'
if (Test-Path -LiteralPath $base) {
    if (-not (Test-Path -LiteralPath $base -PathType Container)) { throw "Not a directory: $base" }
} else {
    [void](New-Item -ItemType Directory -Path $base)
}
$name = [DateTime]::Now.ToString('yyyyMMdd_HHmmss') + '_' + [Guid]::NewGuid().ToString('N')
$folder = Join-Path $base $name
[void](New-Item -ItemType Directory -Path $folder)
$expected = 'd5c40e401b7ea642fc946bad68e1f0733ae9b1b61493d9ca494058d452c5d640'
$collector = Join-Path $PSScriptRoot 'record_morai_host.ps1'
if (-not (Test-Path -LiteralPath $collector -PathType Leaf)) {
    $collector = Join-Path $folder 'record_morai_host.ps1'
    $url = 'https://raw.githubusercontent.com/2026-Ajou-AutoRacing-Study/AjouNice2026/9fb690d79cde2ebc9bd8a0396e3ac2c10fc5622a/scripts/rosbag/record/record_morai_host.ps1'
    Write-Host 'Downloading the pinned read-only collector into the new log folder...'
    Invoke-WebRequest -UseBasicParsing -Uri $url -OutFile $collector
}
$actual = (Get-FileHash -LiteralPath $collector -Algorithm SHA256).Hash
if ($actual -ne $expected) {
    throw "Collector SHA256 mismatch; nothing executed. Keep this folder for diagnosis: $folder"
}
$output = Join-Path $folder 'host_morai.jsonl'
Write-Host "MORAI host measurement started. Session: $SessionId"
Write-Host "Log folder: $folder"
Write-Host 'Ctrl+C stops measurement only. Keep MORAI and the driving stack running.'
& $collector -Output $output -SessionId $SessionId -Duration $Duration -Interval $Interval -IncludeGpu:(-not $NoGpu)
Write-Host "Finished. Share this whole folder with the Linux recording: $folder"
