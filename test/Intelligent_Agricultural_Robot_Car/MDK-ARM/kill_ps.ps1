Get-Process powershell | Where-Object { $_.Id -ne $PID } | Stop-Process -Force
Write-Output "Done"
