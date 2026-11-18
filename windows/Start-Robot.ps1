param([ValidateSet('425','1003')][string]$RobotProfile='425',[string]$RobotAddress='192.168.0.100',[switch]$NoBrowser)
$ErrorActionPreference='Stop'
try {
    Write-Host "Checking robot $RobotProfile at $RobotAddress ..."
    if (!(Get-Command ssh.exe -ErrorAction SilentlyContinue)) {throw 'Windows OpenSSH Client is missing. Install it from Windows Optional Features, then retry.'}
    & ping.exe -n 3 -w 1500 $RobotAddress
    if ($LASTEXITCODE -ne 0) {throw 'Ping failed. Connect to the robot Wi-Fi and check its IP first.'}
    $source=Get-Content -Raw -LiteralPath (Join-Path $PSScriptRoot 'remote_start.py')
    $hostsFile=Join-Path $PSScriptRoot "known_hosts_$RobotProfile"
    Write-Host 'First connection: verify the SSH fingerprint. Password input is hidden; enter your robot SSH password.'
    $source | & ssh.exe -o ConnectTimeout=8 -o ServerAliveInterval=10 -o ServerAliveCountMax=3 -o StrictHostKeyChecking=ask -o "UserKnownHostsFile=$hostsFile" "wheeltec@$RobotAddress" "python3 - $RobotProfile"
    if ($LASTEXITCODE -ne 0) {throw 'Remote check/start failed. See the message above. No forced restart was attempted.'}
    if(!$NoBrowser){Start-Process "http://${RobotAddress}:8890/"}
    Write-Host 'Panel opened. Check map/localization before starting a mission.'
} catch {Write-Host $_ -ForegroundColor Red;exit 1}
