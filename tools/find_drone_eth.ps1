# find_drone_eth.ps1 -- locate the Jetson on a directly-connected Ethernet port
#
# Why: the Jetson uses a STATIC IP (no DHCP), so you must put your PC's
# Ethernet port on the same subnet before it can be reached.
#
# Run in an ELEVATED PowerShell (Administrator). Example:
#   powershell -ExecutionPolicy Bypass -File tools\find_drone_eth.ps1
#
# It will:
#   1. pick the physical Ethernet adapter that is Up,
#   2. for each candidate subnet: assign a temporary static IP, ping the
#      documented Jetson/FC addresses, and if none answer, scan the /24,
#   3. keep the IP that worked and try an SSH test,
#   4. print how to revert.
#
# It will NOT touch your WiFi or set any default gateway.

param(
    [switch]$NoScan   # skip the /24 scan, only ping the known addresses
)

$ErrorActionPreference = 'Continue'
$OutputEncoding = [System.Text.Encoding]::ASCII

function Say($msg, $color = 'Gray') { Write-Host $msg -ForegroundColor $color }

# ---- 1. find physical ethernet adapter ------------------------------------
$eth = Get-NetAdapter | Where-Object {
    $_.MediaType -eq '802.3' -and $_.Status -eq 'Up' -and
    $_.InterfaceDescription -notmatch 'Hyper-V|Virtual|Wi-Fi|Loopback|Bluetooth|WSL'
} | Select-Object -First 1

if (-not $eth) { Say "[x] No physical Ethernet adapter is Up. Plug the cable in first." 'Red'; exit 1 }

Say ("[i] Using adapter: {0}  ({1})" -f $eth.Name, $eth.InterfaceDescription) 'Cyan'
Say ("    Link: {0}   MAC: {1}" -f $eth.LinkSpeed, $eth.MacAddress)
$mbps = 0; [void][int]::TryParse(($eth.LinkSpeed -replace '[^0-9]',''), [ref]$mbps)
if ($mbps -le 10) { Say "    [!] Link is only $($eth.LinkSpeed) -- likely a bad cable (should be 1 Gbps or 100 Mbps)." 'Yellow' }

$idx = $eth.ifIndex

# ---- 1b. make sure the port auto-negotiates (not forced to 10 Mbps) -------
$sd = Get-NetAdapterAdvancedProperty -InterfaceDescription $eth.InterfaceDescription -ErrorAction SilentlyContinue |
      Where-Object { $_.RegistryKeyword -eq '*SpeedDuplex' }
if ($sd -and ($sd.RegistryValue -notcontains '0')) {
    Say ("    [!] Speed & Duplex is forced to '{0}'; restoring Auto Negotiation ..." -f $sd.DisplayValue) 'Yellow'
    Set-NetAdapterAdvancedProperty -InterfaceDescription $eth.InterfaceDescription `
        -RegistryKeyword '*SpeedDuplex' -RegistryValue 0 -ErrorAction SilentlyContinue
    Start-Sleep -Seconds 5
    $eth = Get-NetAdapter -InterfaceIndex $idx
    Say ("    now: {0}" -f $eth.LinkSpeed) 'Cyan'
}

# ---- 2. take the port off DHCP / clear old IPv4 ---------------------------
Set-NetIPInterface -InterfaceIndex $idx -Dhcp Disabled -ErrorAction SilentlyContinue
Get-NetIPAddress -InterfaceIndex $idx -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Remove-NetIPAddress -Confirm:$false -ErrorAction SilentlyContinue

$subnets = @(
    [pscustomobject]@{
        Name = 'mesh 192.168.1.0/24';  Self = '192.168.1.123'; Prefix = 24
        Targets = @('192.168.1.101','192.168.1.102','192.168.1.103','192.168.1.100')
        Base = '192.168.1'
    },
    [pscustomobject]@{
        Name = 'fc-link 192.168.77.0/24'; Self = '192.168.77.50'; Prefix = 24
        Targets = @('192.168.77.100','192.168.77.2')
        Base = '192.168.77'
    },
    [pscustomobject]@{
        Name = 'fc-link 10.41.10.0/24'; Self = '10.41.10.50'; Prefix = 24
        Targets = @('10.41.10.100','10.41.10.2')
        Base = '10.41.10'
    }
)

function Ping-One([string]$ip) {
    try {
        $p = New-Object System.Net.NetworkInformation.Ping
        return ($p.Send($ip, 400).Status -eq 'Success')
    } catch { return $false }
}

function Scan-Subnet([string]$base) {
    $hits = @()
    for ($i = 1; $i -le 254; $i++) {
        $ip = "$base.$i"
        try {
            $p = New-Object System.Net.NetworkInformation.Ping
            if ($p.Send($ip, 250).Status -eq 'Success') { $hits += $ip; Say "    hit: $ip" 'Green' }
        } catch {}
    }
    return $hits
}

$found = $null

foreach ($s in $subnets) {
    Say ""
    Say ("=== trying {0}  (PC = {1}) ===" -f $s.Name, $s.Self) 'Cyan'
    New-NetIPAddress -InterfaceIndex $idx -IPAddress $s.Self -PrefixLength $s.Prefix -ErrorAction SilentlyContinue | Out-Null
    Start-Sleep -Milliseconds 700

    $hit = $null
    foreach ($t in $s.Targets) {
        if (Ping-One $t) { Say "  known host answered: $t" 'Green'; $hit = $t; break }
    }

    if (-not $hit -and -not $NoScan) {
        Say "  no known host; scanning $($s.Base).0/24 ..." 'Yellow'
        $hits = Scan-Subnet $s.Base
        if ($hits.Count -gt 0) { $hit = $hits[0] }
    }

    if ($hit) { $found = [pscustomobject]@{ Subnet = $s; Ip = $hit }; break }

    Get-NetIPAddress -InterfaceIndex $idx -AddressFamily IPv4 -ErrorAction SilentlyContinue |
        Remove-NetIPAddress -Confirm:$false -ErrorAction SilentlyContinue
}

# ---- 3. report / SSH test --------------------------------------------------
if ($found) {
    Say ""
    Say ("[OK] Found a device at {0}  (via {1})" -f $found.Ip, $found.Subnet.Name) 'Green'
    Say ("     Your Ethernet port keeps static IP {0}." -f $found.Subnet.Self)

    $key = Join-Path $env:USERPROFILE '.ssh\id_ed25519'
    Say ""
    Say "[i] SSH test (user nvidia):"
    if (-not (Test-Path $key)) { Say "    private key not found: $key" 'Yellow' }
    else {
        $out = & ssh -i $key -o BatchMode=yes -o StrictHostKeyChecking=accept-new -o ConnectTimeout=6 "nvidia@$($found.Ip)" "hostname; uname -sr" 2>&1
        $code = $LASTEXITCODE
        $out | ForEach-Object { Say "    $_" }
        if ($code -eq 0) {
            Say "    => passwordless SSH OK, no password needed." 'Green'
        } else {
            Say "    => key auth failed; run once with password to install the key:" 'Yellow'
            Say "       ssh-copy-id -i `"$key.pub`" nvidia@$($found.Ip)" 'Yellow'
        }
    }
    Say ""
    Say "To revert the Ethernet port back to DHCP when done:"
    Say ("  Set-NetIPInterface -InterfaceIndex {0} -Dhcp Enabled" -f $idx)
    Say ("  Remove-NetIPAddress -InterfaceIndex {0} -AddressFamily IPv4 -Confirm:`$false" -f $idx)
} else {
    Say ""
    Say "[x] Nothing answered on any candidate subnet." 'Red'
    Say "    - Check it is really the Jetson's DATA port (not the FC's, not power)."
    Say "    - Check the cable (link is only $($eth.LinkSpeed)); try another cable."
    Say "    - Re-run WITHOUT -NoScan so it scans the whole /24."
    Get-NetIPAddress -InterfaceIndex $idx -AddressFamily IPv4 -ErrorAction SilentlyContinue |
        Remove-NetIPAddress -Confirm:$false -ErrorAction SilentlyContinue
    Set-NetIPInterface -InterfaceIndex $idx -Dhcp Enabled -ErrorAction SilentlyContinue
    Say "    Ethernet port restored to DHCP."
}
