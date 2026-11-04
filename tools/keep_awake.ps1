param([Parameter(Mandatory = $true)][int]$WatchPid)
# Blocks Windows idle sleep while process $WatchPid lives. Idle sleep stops the
# WSL VM and every campaign in its container; a `wsl -e sleep` keepalive does
# not prevent it. Closing the lid or a manual sleep still wins. Create both
# processes through WMI: Start-Process children die with the launching session.
# Usage, from the repository root:
#   $c = Invoke-CimMethod Win32_Process -MethodName Create -Arguments @{CommandLine='wsl.exe -e bash <campaign script> ...'; CurrentDirectory=$PWD.Path}
#   Invoke-CimMethod Win32_Process -MethodName Create -Arguments @{CommandLine="powershell.exe -NoProfile -WindowStyle Hidden -ExecutionPolicy Bypass -File tools\keep_awake.ps1 -WatchPid $($c.ProcessId)"; CurrentDirectory=$PWD.Path}
Add-Type -Namespace W -Name P -MemberDefinition '[DllImport("kernel32.dll")] public static extern uint SetThreadExecutionState(uint f);'
# ES_CONTINUOUS | ES_SYSTEM_REQUIRED
[W.P]::SetThreadExecutionState([uint32]"0x80000001") | Out-Null
while (Get-Process -Id $WatchPid -ErrorAction SilentlyContinue) { Start-Sleep -Seconds 60 }
[W.P]::SetThreadExecutionState([uint32]"0x80000000") | Out-Null
