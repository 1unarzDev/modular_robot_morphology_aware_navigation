# Runs the declared real-buildings study until its report exists, restarting it
# from the per-building checkpoint if the process dies. Every building's draw is
# seeded from its own id, so a resumed run yields what an uninterrupted one
# would; each restart is logged to run.log. Launch through WMI (CLAUDE.md):
#   Invoke-CimMethod Win32_Process -MethodName Create -Arguments @{CommandLine='powershell.exe -NoProfile -WindowStyle Hidden -ExecutionPolicy Bypass -File tools\supervise_real_buildings.ps1'; CurrentDirectory=$PWD.Path}
$ErrorActionPreference = "Continue"
$dir = "studies\real_buildings"
$output = "$dir\real_buildings_20261115.json"
$log = "$dir\run.log"
for ($attempt = 1; $attempt -le 20; $attempt++) {
    if (Test-Path $output) { break }
    Add-Content $log "SUPERVISOR $(Get-Date -Format o) start attempt $attempt"
    # Below normal priority: the host is the user's own machine (2026-11-15).
    cmd.exe /c "start `"`" /belownormal /b /wait .venv-host\Scripts\python.exe tools\run_real_buildings_host.py --buildings $dir\sample.txt --data-root C:\Users\risha\datasets\hm3d --master-seed 20261115 --workers 7 --output $output >> $log 2>&1"
    Add-Content $log "SUPERVISOR $(Get-Date -Format o) attempt $attempt exited with $LASTEXITCODE"
    if (-not (Test-Path $output)) { Start-Sleep -Seconds 30 }
}
Add-Content $log "SUPERVISOR $(Get-Date -Format o) finished; report present: $(Test-Path $output)"
