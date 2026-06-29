$baud = 115200
$port = New-Object System.IO.Ports.SerialPort 'COM9', $baud, 'None', 8, 'One'
$port.DtrEnable = $true
$port.RtsEnable = $true
$port.Open()
$port.ReadTimeout = 8000
Start-Sleep -Milliseconds 100

$buffer = ''
try {
    while ($true) {
        $b = $port.ReadByte()
        $buffer += [char]$b
    }
} catch {
    # timeout
}

$port.Close()
Write-Output "=== CAPTURED AT $baud baud ==="
Write-Output $buffer
