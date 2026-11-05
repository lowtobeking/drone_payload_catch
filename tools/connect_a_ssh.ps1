# Connect to A through WLAN without changing network settings.
param([switch]$Check)
$ErrorActionPreference = 'Stop'
$ssh = Join-Path $env:WINDIR 'System32\OpenSSH\ssh.exe'
$key = Join-Path $env:USERPROFILE '.ssh\id_ed25519'
if (-not (Test-Path -LiteralPath $ssh) -or -not (Test-Path -LiteralPath $key)) {
    throw 'OpenSSH or the A identity key is missing.'
}
$wifi = [System.Net.NetworkInformation.NetworkInterface]::GetAllNetworkInterfaces() |
    Where-Object { $_.Name -eq 'WLAN' -and $_.OperationalStatus -eq 'Up' } |
    Select-Object -First 1
if (-not $wifi) { throw 'WLAN is not connected.' }
$localIp = $wifi.GetIPProperties().UnicastAddresses |
    Where-Object { $_.Address.AddressFamily -eq 'InterNetwork' -and
                   $_.Address.ToString().StartsWith('192.168.1.') } |
    Select-Object -First 1
if (-not $localIp) { throw 'WLAN has no 192.168.1.x IPv4 address.' }
$common = @('-b', $localIp.Address.ToString(), '-o', 'ConnectTimeout=6',
            '-o', 'StrictHostKeyChecking=yes', '-o', 'HostKeyAlias=192.168.155.186',
            '-i', $key)
foreach ($target in @('192.168.1.5', '192.168.1.101')) {
    $serial = & $ssh @common '-o' 'BatchMode=yes' "nvidia@$target" 'cat /proc/device-tree/serial-number' 2>$null
    if ($LASTEXITCODE -eq 0 -and ($serial -join '').Trim([char]0) -eq '1612425621719') {
        Write-Host "A verified at $target via WLAN $($localIp.Address)"
        if ($Check) { exit 0 }
        & $ssh @common "nvidia@$target"
        exit $LASTEXITCODE
    }
}
throw 'A did not answer on either verified address.'
