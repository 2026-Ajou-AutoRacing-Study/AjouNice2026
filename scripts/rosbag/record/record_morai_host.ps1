#requires -Version 5.1
<#
Read-only Windows MORAI-PC companion to the Linux developer bag recorder.
Example (ordinary Windows PowerShell; destination directory must already exist):
  powershell.exe -NoProfile -File .\record_morai_host.ps1 -Output "$HOME\Desktop\morai_host.jsonl" -SessionId "LAN_20261024_01"
Optional: -Duration 300 -Interval 2 -ProcessName "MORAI*" -ProcessId 1234 -IncludeGpu
Use the SAME SessionId on both PCs. Stop with Ctrl+C; this never stops MORAI.
Copy this JSONL beside the Linux bag after recording. Existing output is refused.
Closing the console/power loss may omit the final record; keep that partial log.
No installation, administrator access, firewall/NIC/time-sync changes, or process
command lines/environment are needed. Do not disable execution policy globally;
if local policy blocks this script, ask the PC owner before changing anything.

Limits: Windows-only and not Windows-tested in this Linux workspace. Missing or
denied counters are unavailable/null, NOT evidence of zero drops. NIC counters
cover all traffic on each interface; UDP statistics cover all IPv4 UDP on the PC.
IncomingDatagramsDiscarded means bad-port discards, NOT Linux RcvbufErrors.
No per-socket drops, sensor packet sequence, source FPS, or clock-sync proof.
The CPU provider is formatted WMI/CIM data, not instantaneous Task Manager data.
Sampling duration/lateness/missed slots expose slow counter queries. GPU opt-in
queries only nvidia-smi, with a 1-second timeout; only its own query child may be
terminated on timeout. Default GPU OFF avoids that additional measurement load.
Use a short 30-second dry run on Windows before requesting a competition drive.
Sources: https://learn.microsoft.com/en-us/dotnet/api/system.net.networkinformation.udpstatistics
https://learn.microsoft.com/en-us/powershell/module/netadapter/get-netadapterstatistics
https://learn.microsoft.com/en-us/windows/win32/fwp/wmi/netadaptercimprov/msft-netadapterstatisticssettingdata
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][string]$Output,
    [string]$SessionId = '', [string]$Role = 'morai',
    [ValidateRange(0,864000)][double]$Duration = 0,
    [ValidateRange(1,3600)][double]$Interval = 2,
    [string[]]$ProcessName = @('MORAI*'), [int[]]$ProcessId = @(),
    [switch]$IncludeGpu, [ValidateRange(1,3600)][double]$GpuInterval = 5
)
$ErrorActionPreference = 'Stop'
if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) { throw 'Windows only' }
foreach ($number in @($Duration,$Interval,$GpuInterval)) {
    if ([double]::IsNaN($number) -or [double]::IsInfinity($number)) { throw 'Finite intervals required' }
}
if (@($ProcessId | Where-Object { $_ -le 0 }).Count) { throw 'Process IDs must be positive' }
$schema = 'ajounice.host_telemetry.windows.v1'
$frequency = [Diagnostics.Stopwatch]::Frequency
$watch = [Diagnostics.Stopwatch]::StartNew()
$previous = @{}; $sequence = 0; $missedTotal = 0; $lastGpu = -100000.0; $gpuCache = @{status='disabled'}
$hash = [Security.Cryptography.SHA256]::Create()
try { $hostId = ([BitConverter]::ToString($hash.ComputeHash([Text.Encoding]::UTF8.GetBytes([Environment]::MachineName)))).Replace('-','').Substring(0,16) }
finally { $hash.Dispose() }
$clockId = $hostId + ':Stopwatch:' + $frequency
$nvidia = Get-Command nvidia-smi.exe -ErrorAction SilentlyContinue
function Get-Unavailable($Record) { return @{status='unavailable'; error_type=$Record.Exception.GetType().Name} }
function Write-Record($Value) { $writer.WriteLine(($Value | ConvertTo-Json -Depth 12 -Compress)); $writer.Flush() }
function Get-Delta([string]$Key, [System.Collections.IDictionary]$Counters, [double]$Now) {
    $old = $previous[$Key]; $delta = @{}; $rates = @{}; $reset = @(); $span = $null
    if ($null -ne $old) { $span = $Now - $old.time }
    foreach ($name in $Counters.Keys) {
        $delta[$name] = $null; $rates[$name] = $null
        if ($null -ne $Counters[$name] -and $null -ne $old -and $null -ne $old.counters[$name] -and $span -gt 0) {
            if ($Counters[$name] -lt $old.counters[$name]) { $reset += $name }
            else { $delta[$name] = $Counters[$name] - $old.counters[$name]; $rates[$name] = $delta[$name] / $span }
        }
    }
    $previous[$Key] = @{time=$Now; counters=$Counters}
    return @{status='ok'; counters=$Counters; delta=$delta; per_second=$rates; baseline=($null -eq $old); counter_reset=($reset.Count -gt 0); reset_fields=$reset; elapsed_sec=$span}
}
function Get-Gpu {
    if (-not $IncludeGpu) { return @{status='disabled'} }
    if ($null -eq $nvidia) { return @{status='unavailable'; error_type='NvidiaSmiNotFound'} }
    $child = New-Object Diagnostics.Process
    try {
        $child.StartInfo = New-Object Diagnostics.ProcessStartInfo
        $child.StartInfo.FileName = $nvidia.Source
        $child.StartInfo.Arguments = '--query-gpu=index,utilization.gpu,memory.used,memory.total --format=csv,noheader,nounits'
        $child.StartInfo.UseShellExecute = $false; $child.StartInfo.CreateNoWindow = $true
        $child.StartInfo.RedirectStandardOutput = $true; $child.StartInfo.RedirectStandardError = $true
        [void]$child.Start(); $text = $child.StandardOutput.ReadToEndAsync(); $errors = $child.StandardError.ReadToEndAsync()
        if (-not $child.WaitForExit(1000)) { $child.Kill(); return @{status='unavailable'; error_type='NvidiaSmiTimeout'} }
        if ($child.ExitCode -ne 0) { return @{status='unavailable'; error_type='NvidiaSmiFailed'; exit_code=$child.ExitCode} }
        $rows = @()
        foreach ($line in ($text.Result -split "`r?`n" | Where-Object { $_.Trim() })) {
            $parts = $line.Split(','); if ($parts.Count -ne 4) { throw 'Unexpected GPU columns' }
            $row = @{}; $names = @('index','utilization_gpu_percent','memory_used_mib','memory_total_mib')
            for ($i=0; $i -lt 4; $i++) {
                $value = 0.0
                if ([double]::TryParse($parts[$i].Trim(),[Globalization.NumberStyles]::Float,[Globalization.CultureInfo]::InvariantCulture,[ref]$value) -and -not [double]::IsNaN($value) -and -not [double]::IsInfinity($value)) { $row[$names[$i]]=$value }
                else { $row[$names[$i]]=$null }
            }
            $rows += $row
        }
        return @{status='ok'; gpus=$rows}
    } catch { return Get-Unavailable $_ } finally { $child.Dispose() }
}
$path = [IO.Path]::GetFullPath($Output)
$stream = [IO.File]::Open($path,[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::Read)
$writer = New-Object IO.StreamWriter($stream,(New-Object Text.UTF8Encoding($false)))
# Precompute the end-record prefix: direct .NET writes can run even during Ctrl+C pipeline cancellation.
$endPrefix = (@{record_type='end'; schema=$schema; session_id=$SessionId; role=$Role; host_id=$hostId; clock_id=$clockId} | ConvertTo-Json -Compress).TrimEnd('}')
try {
    Write-Record @{record_type='metadata'; schema=$schema; session_id=$SessionId; role=$Role; host_id=$hostId; clock_id=$clockId;
        utc=[DateTime]::UtcNow.ToString('o'); os=[Environment]::OSVersion.VersionString; powershell=$PSVersionTable.PSVersion.ToString();
        config=@{interval_sec=$Interval; duration_sec=$Duration; process_names=$ProcessName; process_ids=$ProcessId; gpu_enabled=[bool]$IncludeGpu; gpu_interval_sec=$GpuInterval};
        clocks=@{wall='DateTime.UtcNow'; monotonic='Stopwatch/GetTimestamp'; stopwatch_frequency=$frequency; cross_host_synchronization_verified=$false};
        limitations=@('Counters cover whole host/interfaces, not sensor sequence loss.','IPv4 UDP port-error discards are not Linux receive-buffer errors.','No per-socket drops/source FPS/clock-sync proof.','Process CPU uses one CPU as 100%.','Windows smoke validation required before deployment.')}
    $due = 0.0
    while ($Duration -eq 0 -or $watch.Elapsed.TotalSeconds -lt $Duration) {
        $wait = $due - $watch.Elapsed.TotalSeconds
        if ($wait -gt 0) { Start-Sleep -Milliseconds ([int][Math]::Min(200,$wait*1000)); continue }
        $now = $watch.Elapsed.TotalSeconds; $wall = [DateTime]::UtcNow; $ticks = [Diagnostics.Stopwatch]::GetTimestamp()
        $sample = @{record_type='sample'; schema=$schema; session_id=$SessionId; role=$Role; host_id=$hostId; clock_id=$clockId; sequence=$sequence;
            utc=$wall.ToString('o'); wall_time_ns=[long](($wall.Ticks - [long]621355968000000000)*100); monotonic_ticks=$ticks;
            monotonic_ns=[long]([decimal]$ticks*1000000000/$frequency); schedule_lateness_sec=[Math]::Max(0,$now-$due)}
        try {
            $cpus=@{}; foreach ($cpu in @(Get-CimInstance Win32_PerfFormattedData_PerfOS_Processor -Property Name,PercentProcessorTime -OperationTimeoutSec 2)) {
                $cpus[$cpu.Name]=@{utilization_percent=$cpu.PercentProcessorTime}
            }
            if (-not $cpus.Count) { throw 'No CPU counters' }; $sample.cpu=@{status='ok'; provider='CIM formatted'; cpus=$cpus}
        } catch { $sample.cpu=Get-Unavailable $_ }
        try {
            $memory=Get-CimInstance Win32_OperatingSystem -Property TotalVisibleMemorySize,FreePhysicalMemory,TotalVirtualMemorySize,FreeVirtualMemory -OperationTimeoutSec 2
            foreach ($field in @('TotalVisibleMemorySize','FreePhysicalMemory','TotalVirtualMemorySize','FreeVirtualMemory')) {
                if ($null -eq $memory.$field) { throw 'Memory property unavailable' }
            }
            $sample.memory=@{status='ok'; total_physical_bytes=([long]$memory.TotalVisibleMemorySize*1024); available_physical_bytes=([long]$memory.FreePhysicalMemory*1024); total_virtual_bytes=([long]$memory.TotalVirtualMemorySize*1024); free_virtual_bytes=([long]$memory.FreeVirtualMemory*1024)}
        } catch { $sample.memory=Get-Unavailable $_ }
        try {
            $adapters=@{}; foreach ($nic in @(Get-NetAdapterStatistics)) {
                $counters=@{}; foreach ($name in @('ReceivedBytes','SentBytes','ReceivedUnicastPackets','ReceivedMulticastPackets','ReceivedBroadcastPackets','SentUnicastPackets','SentMulticastPackets','SentBroadcastPackets','ReceivedDiscardedPackets','ReceivedPacketErrors','OutboundDiscardedPackets','OutboundPacketErrors')) {
                    $counters[$name]=$nic.$name
                }; $adapters[$nic.Name]=Get-Delta ('nic:'+$nic.Name) $counters $now
            }; $sample.network=@{status='ok'; interfaces=$adapters}
        } catch { $sample.network=Get-Unavailable $_ }
        try {
            $udp=[Net.NetworkInformation.IPGlobalProperties]::GetIPGlobalProperties().GetUdpIPv4Statistics()
            $sample.udp_ipv4=Get-Delta 'udp4' @{DatagramsReceived=$udp.DatagramsReceived; DatagramsSent=$udp.DatagramsSent; IncomingDatagramsDiscarded=$udp.IncomingDatagramsDiscarded; IncomingDatagramsWithErrors=$udp.IncomingDatagramsWithErrors} $now
            $sample.udp_ipv4.listeners=$udp.UdpListeners
        } catch { $sample.udp_ipv4=Get-Unavailable $_ }
        $processes=@{}; $ids=@($ProcessId); $sample.process_selection_errors=@()
        foreach ($name in $ProcessName) {
            try { $ids += @(Get-Process -Name $name -ErrorAction Stop | Select-Object -ExpandProperty Id) }
            catch { $sample.process_selection_errors += @{selector=$name; status='unavailable'; error_type=$_.Exception.GetType().Name} }
        }
        foreach ($processNumber in @($ids | Sort-Object -Unique)) {
            try {
                $p=Get-Process -Id $processNumber; $key='pid:'+$processNumber+':'+$p.StartTime.ToUniversalTime().Ticks
                $stats=Get-Delta $key @{cpu_seconds=$p.TotalProcessorTime.TotalSeconds} $now
                $cpuPercent=$null; if ($null -ne $stats.per_second.cpu_seconds) { $cpuPercent=$stats.per_second.cpu_seconds*100 }
                $processes[[string]$processNumber]=@{status='ok'; name=$p.ProcessName; cpu_seconds=$p.TotalProcessorTime.TotalSeconds; cpu_percent=$cpuPercent; working_set_bytes=$p.WorkingSet64; private_memory_bytes=$p.PrivateMemorySize64; baseline=$stats.baseline}
            } catch { $processes[[string]$processNumber]=Get-Unavailable $_ }
        }; $sample.processes=$processes
        if ($now-$lastGpu -ge $GpuInterval) { $gpuCache=Get-Gpu; $lastGpu=$now }
        $sample.gpu=@{measurement=$gpuCache; age_sec=($now-$lastGpu)}
        $sample.sampling_duration_sec=$watch.Elapsed.TotalSeconds-$now
        $next=$due+$Interval; $missed=[Math]::Max(0,[Math]::Floor(($watch.Elapsed.TotalSeconds-$next)/$Interval)+1)
        $sample.missed_schedule_slots=$missed; $missedTotal+=$missed
        Write-Record $sample; $sequence++; $due=$next+$missed*$Interval
    }
} finally {
    try {
        $writer.WriteLine([string]::Concat($endPrefix,',"utc":"',[DateTime]::UtcNow.ToString('o'),'","samples":',$sequence,',"missed_schedule_slots":',$missedTotal,',"elapsed_sec":',$watch.Elapsed.TotalSeconds.ToString('R',[Globalization.CultureInfo]::InvariantCulture),'}'))
        $writer.Flush()
    } finally { $writer.Dispose(); $stream.Dispose(); $watch.Stop() }
}
