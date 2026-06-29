Write-Output "=== Available COM ports ==="
[System.IO.Ports.SerialPort]::GetPortNames() | ForEach-Object { Write-Output "  $_" }

Write-Output "`n=== Processes with COM handle ==="
Get-Process | ForEach-Object {
    try {
        $handles = Get-Process -Id $_.Id | Select-Object -ExpandProperty Modules -ErrorAction SilentlyContinue
    } catch {}
}

Write-Output "`n=== Checking COM9 directly ==="
try {
    $port = New-Object System.IO.Ports.SerialPort 'COM9', 115200, 'None', 8, 'One'
    $port.ReadTimeout = 1000
    $port.Open()
    Write-Output "COM9 opened successfully at 115200"
    Start-Sleep -Milliseconds 100
    $data = ''
    try {
        while ($true) {
            $b = $port.ReadByte()
            $data += [char]$b
        }
    } catch {
        # timeout
    }
    $port.Close()
    if ($data) {
        Write-Output "Read $($data.Length) chars:"
        Write-Output $data
    } else {
        Write-Output "No data on COM9 at 115200"
    }
} catch {
    Write-Output "Failed to open COM9 at 115200: $($_.Exception.Message)"
    # Try 9600
    try {
        $port = New-Object System.IO.Ports.SerialPort 'COM9', 9600, 'None', 8, 'One'
        $port.ReadTimeout = 1000
        $port.Open()
        Write-Output "COM9 opened successfully at 9600"
        Start-Sleep -Milliseconds 100
        $data = ''
        try {
            while ($true) {
                $b = $port.ReadByte()
                $data += [char]$b
            }
        } catch {}
        $port.Close()
        if ($data) {
            Write-Output "Read $($data.Length) chars at 9600:"
            Write-Output $data
        } else {
            Write-Output "No data on COM9 at 9600"
        }
    } catch {
        Write-Output "Failed to open COM9 at 9600: $($_.Exception.Message)"
    }
}
