param([Parameter(Mandatory = $true)][int]$WatchPid)
# Blocks Windows idle sleep while process $WatchPid lives. Idle sleep stops the
# WSL VM and every campaign in its container; a `wsl -e sleep` keepalive does
# not prevent it. Closing the lid or a manual sleep still wins. Usage:
#   $p = Start-Process -WindowStyle Hidden -PassThru wsl -ArgumentList '-e','bash','<campaign script>',...
#   Start-Process -WindowStyle Hidden powershell -ArgumentList '-NoProfile','-ExecutionPolicy','Bypass','-File','tools\keep_awake.ps1','-WatchPid',$p.Id
Add-Type -Namespace W -Name P -MemberDefinition '[DllImport("kernel32.dll")] public static extern uint SetThreadExecutionState(uint f);'
# ES_CONTINUOUS | ES_SYSTEM_REQUIRED
[W.P]::SetThreadExecutionState([uint32]"0x80000001") | Out-Null
while (Get-Process -Id $WatchPid -ErrorAction SilentlyContinue) { Start-Sleep -Seconds 60 }
[W.P]::SetThreadExecutionState([uint32]"0x80000000") | Out-Null
