# Flash first, then try both baud rates

# Reset MCU
$cli = "C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe"
cmd /c "`"$cli`" -c SWD -Rst -NoPrompt 2>&1" | Out-Null

# Try 9600 first (boot markers are at 9600)
foreach ($baud in @(9600, 115200)) {
    try {
        $port = New-Object System.IO.Ports.SerialPort 'COM9', $baud, 'None', 8, 'One'
        $port.DtrEnable = $true
        $port.RtsEnable = $true
        $port.ReadTimeout = 3000
        $port.Open()
        Start-Sleep -Milliseconds 200

        $buf = ''
        try {
            while ($true) {
                $b = $port.ReadByte()
                $buf += [char]$b
            }
        } catch {}

        $port.Close()

        if ($buf.Length -gt 0) {
            Write-Output "=== $baud baud: $($buf.Length) chars ==="
            Write-Output $buf
            break
        } else {
            Write-Output "No data at $baud baud"
        }
    } catch {
        Write-Output "Error at $baud: $($_.Exception.Message)"
    }
}
