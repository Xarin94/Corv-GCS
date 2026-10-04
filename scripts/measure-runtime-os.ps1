# Read-only Windows counters. JSON requests arrive on stdin; one sample per line.
$ErrorActionPreference = 'Stop'
Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
public static class CorvThreadInfo {
    [DllImport("kernel32.dll")] static extern IntPtr OpenThread(uint access, bool inherit, uint id);
    [DllImport("kernel32.dll")] static extern int GetThreadDescription(IntPtr thread, out IntPtr text);
    [DllImport("kernel32.dll")] static extern IntPtr LocalFree(IntPtr memory);
    [DllImport("kernel32.dll")] static extern bool CloseHandle(IntPtr handle);
    public static string Name(uint id) {
        var handle = OpenThread(0x0800, false, id);
        if (handle == IntPtr.Zero) return "";
        IntPtr text = IntPtr.Zero;
        try { return GetThreadDescription(handle, out text) >= 0 ? Marshal.PtrToStringUni(text) : ""; }
        finally { if (text != IntPtr.Zero) LocalFree(text); CloseHandle(handle); }
    }
}
'@
$utf8 = New-Object System.Text.UTF8Encoding($false)
[Console]::OutputEncoding = $utf8
$threadNames = @{}
while ($null -ne ($line = [Console]::In.ReadLine())) {
    try {
        $request = $line | ConvertFrom-Json
        $watch = [Diagnostics.Stopwatch]::StartNew()
        $threads = @()
        foreach ($idValue in $request.pids) {
            try {
                $processObject = Get-Process -Id $idValue -ErrorAction Stop
                foreach ($thread in $processObject.Threads) {
                    try {
                        $key = [string]$idValue + ':' + [string]$thread.Id
                        if (-not $threadNames.ContainsKey($key)) { $threadNames[$key] = [CorvThreadInfo]::Name($thread.Id) }
                        $threads += [ordered]@{ processId = $idValue; threadId = $thread.Id; name = $threadNames[$key]; cpuSeconds = $thread.TotalProcessorTime.TotalSeconds }
                    } catch { }
                }
            } catch { }
        }
        $gpu = @(Get-CimInstance Win32_PerfRawData_GPUPerformanceCounters_GPUEngine | Where-Object {
            $_.Name -match '^pid_(\d+)_' -and [int]$Matches[1] -in $request.pids
        } | ForEach-Object { [ordered]@{ name = $_.Name; runningTime = [string]$_.RunningTime; timestamp100ns = [string]$_.Timestamp_Sys100NS } })
        $gpuMemory = @(Get-CimInstance Win32_PerfFormattedData_GPUPerformanceCounters_GPUProcessMemory | Where-Object {
            $_.Name -match '^pid_(\d+)_' -and [int]$Matches[1] -in $request.pids
        } | ForEach-Object { [ordered]@{ name = $_.Name; dedicatedBytes = $_.DedicatedUsage; sharedBytes = $_.SharedUsage; committedBytes = $_.TotalCommitted } })
        $sampler = Get-Process -Id $PID
        [ordered]@{ time = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds(); threads = $threads; gpu = $gpu; gpuMemory = $gpuMemory;
            samplerMs = $watch.Elapsed.TotalMilliseconds; samplerCpuSeconds = $sampler.CPU } | ConvertTo-Json -Depth 6 -Compress
    } catch { [ordered]@{ time = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds(); error = $_.Exception.Message } | ConvertTo-Json -Compress }
}
