$baud = 115200
$port = New-Object System.IO.Ports.SerialPort 'COM9', $baud, 'None', 8, 'One'
$port.DtrEnable = $true
$port.RtsEnable = $true
$port.Open()
$port.ReadTimeout = 15000

# Flash the MCU (will reset at end and produce boot output)
$flashCmd = '"C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe" -c SWD -P "D:\stm_design\Intelligent_Agricultural_Robot_Car\MDK-ARM\Intelligent_Agricultural_Robot_Car\Intelligent_Agricultural_Robot_Car.hex" -Rst -NoPrompt 2>&1'
$null = cmd /c $flashCmd

# Wait a moment for boot to complete
Start-Sleep -Milliseconds 500

# Read all available data
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
