param(
    [Parameter(Mandatory = $true)]
    [string]$Port,

    [int]$ScanTimeout = 30
)

$bridge = Join-Path $PSScriptRoot "..\firmware\STM32_Wireless_CAN_Gateway\tools\vehicle_ble_to_stm32_bridge.py"
python -u $bridge --port $Port --scan-timeout $ScanTimeout
