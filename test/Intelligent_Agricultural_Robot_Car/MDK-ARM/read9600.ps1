$port = New-Object System.IO.Ports.SerialPort 'COM9', 9600, 'None', 8, 'One'
$port.DtrEnable = $false
$port.RtsEnable = $false
$port.ReadTimeout = 5000
$port.Open()
Write-Output "Opened COM9 at 9600"

$data = ''
$t0 = Get-Date
try {
    while ( ((Get-Date) - $t0).TotalSeconds -lt 6 ) {
        $b = $port.ReadByte()
        $data += [char]$b
    }
} catch {}

$port.Close()
Write-Output "Read $($data.Length) chars at 9600"
Write-Output "=== DATA ==="
Write-Output $data
