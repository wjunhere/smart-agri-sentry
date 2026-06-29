$port = New-Object System.IO.Ports.SerialPort COM9,115200,None,8,One
$port.ReadTimeout = 3000
try {
    $port.Open()
    $port.WriteLine("STATUS")
    Start-Sleep -Milliseconds 300
    # Read all available response lines
    while ($port.BytesToRead -gt 0) {
        $line = $port.ReadLine()
        Write-Host $line
    }
} catch {
    Write-Host "ERROR: $_"
} finally {
    if ($port.IsOpen) { $port.Close() }
}
