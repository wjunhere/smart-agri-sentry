$port = New-Object System.IO.Ports.SerialPort 'COM9', 115200, 'None', 8, 'One'
$port.DtrEnable = $false
$port.RtsEnable = $false
$port.ReadTimeout = 4000
$port.Open()
Write-Output "Opened at 115200"

$data = ''
$t0 = Get-Date
try {
    while ( ((Get-Date) - $t0).TotalSeconds -lt 5 ) {
        $b = $port.ReadByte()
        $data += [char]$b
    }
} catch {}

$port.Close()
Write-Output "Read $($data.Length) chars at 115200"

if ($data.Length -eq 0) {
    # try 9600
    $port2 = New-Object System.IO.Ports.SerialPort 'COM9', 9600, 'None', 8, 'One'
    $port2.DtrEnable = $false
    $port2.RtsEnable = $false
    $port2.ReadTimeout = 4000
    $port2.Open()
    Write-Output "Opened at 9600"

    $data2 = ''
    $t0 = Get-Date
    try {
        while ( ((Get-Date) - $t0).TotalSeconds -lt 5 ) {
            $b = $port2.ReadByte()
            $data2 += [char]$b
        }
    } catch {}
    $port2.Close()
    Write-Output "Read $($data2.Length) chars at 9600"
    if ($data2) { Write-Output $data2 }
} else {
    Write-Output $data
}
