# Read, stage or restore the standard Windows Wi-Fi battery power policy.
# Never restart an adapter, change SSIDs/IPs, reactivate a power plan or install drivers.
[CmdletBinding(SupportsShouldProcess)]
param(
    [ValidateSet('Audit', 'Stage', 'Restore')][string]$Mode = 'Audit',
    [string]$AdapterName = 'WLAN',
    [string]$StatePath = ''
)
$ErrorActionPreference = 'Stop'
if (-not $StatePath) { $StatePath = Join-Path (Split-Path -Parent $MyInvocation.MyCommand.Path) '..\..\validation\windows-wifi-policy.json' }
$wifiSubgroup = '19cbb8fa-5279-450e-9fac-8a3d5fedd0c1'
$wifiPowerSetting = '12bbebe6-58d6-4636-95bb-3217ef867c1a'

function Invoke-WifiPowercfg([string[]]$Values) {
    $result = & powercfg.exe @Values
    if ($LASTEXITCODE -ne 0) { throw "powercfg failed: $LASTEXITCODE" }
    return ($result -join "`n")
}

function Get-WifiSnapshot {
    $adapter = Get-NetAdapter -Name $AdapterName
    if ($adapter.InterfaceDescription -ne 'Realtek 8922AE WiFi 7 PCI-E NIC') {
        throw 'This adjustment is scoped to the audited Realtek 8922AE adapter.'
    }
    $active = Invoke-WifiPowercfg -Values @('/getactivescheme')
    $scheme = [regex]::Match($active, '[0-9a-fA-F]{8}(-[0-9a-fA-F]{4}){3}-[0-9a-fA-F]{12}').Value
    if (-not $scheme) { throw 'Cannot identify the active power plan.' }
    $power = Invoke-WifiPowercfg -Values @('/query', $scheme, $wifiSubgroup, $wifiPowerSetting)
    $indices = [regex]::Matches($power, '0x([0-9a-fA-F]{8})')
    if ($indices.Count -ne 2) { throw 'Cannot identify AC/DC Wi-Fi power indices.' }
    return [ordered]@{
        time = [DateTimeOffset]::Now.ToString('o')
        adapter_name = $adapter.Name
        adapter_guid = [string]$adapter.InterfaceGuid
        adapter_description = $adapter.InterfaceDescription
        adapter_status = [string]$adapter.Status
        driver = $adapter.DriverInformation
        ipv4 = @(Get-NetIPAddress -InterfaceIndex $adapter.ifIndex -AddressFamily IPv4 -ErrorAction SilentlyContinue |
            Select-Object -ExpandProperty IPAddress)
        power_plan = $scheme
        ac_index = [Convert]::ToInt32($indices[0].Groups[1].Value, 16)
        dc_index = [Convert]::ToInt32($indices[1].Groups[1].Value, 16)
        driver_properties = @(Get-NetAdapterAdvancedProperty -Name $AdapterName -AllProperties |
            Where-Object { $_.RegistryKeyword -in @('LpsEn', 'RegRoamLevel', 'RegROAMSensitiveLevel', 'PreferBand') } |
            Select-Object RegistryKeyword, RegistryValue, DisplayValue)
    }
}

$before = Get-WifiSnapshot
if ($Mode -eq 'Audit') {
    $before | ConvertTo-Json -Depth 5
    return
}
$resolvedState = [IO.Path]::GetFullPath($StatePath)
if ($Mode -eq 'Stage') {
    if ($before.dc_index -eq 0) {
        [ordered]@{status='already_maximum_performance';current=$before} | ConvertTo-Json -Depth 6
        return
    }
    if (Test-Path -LiteralPath $resolvedState) {
        throw 'A prior backup exists. Review or restore it before staging another change.'
    }
    if ($PSCmdlet.ShouldProcess($before.power_plan, 'Back up and set Wi-Fi battery power index to 0; no activation or restart')) {
        $parent = Split-Path -Parent $resolvedState
        New-Item -ItemType Directory -Path $parent -Force | Out-Null
        $before | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $resolvedState -Encoding UTF8
        Invoke-WifiPowercfg -Values @('/setdcvalueindex', $before.power_plan, $wifiSubgroup, $wifiPowerSetting, '0') | Out-Null
    } else { return }
} else {
    $backup = Get-Content -LiteralPath $resolvedState -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($backup.adapter_guid -ne $before.adapter_guid -or
        $backup.power_plan -notmatch '^[0-9a-fA-F]{8}(-[0-9a-fA-F]{4}){3}-[0-9a-fA-F]{12}$' -or
        $backup.dc_index -notin @(0, 1, 2, 3)) {
        throw 'The backup does not match the audited adapter/power policy.'
    }
    if ($PSCmdlet.ShouldProcess($backup.power_plan, 'Restore the saved Wi-Fi battery power index; no activation or restart')) {
        Invoke-WifiPowercfg -Values @('/setdcvalueindex', $backup.power_plan, $wifiSubgroup, $wifiPowerSetting,
            [string]$backup.dc_index) | Out-Null
    } else { return }
}
$after = Get-WifiSnapshot
$expectedIndex = if ($Mode -eq 'Stage') { 0 } else { [int]$backup.dc_index }
if ($after.power_plan -eq $before.power_plan -and $after.dc_index -ne $expectedIndex) {
    throw 'Power policy verification failed; the backup has been retained.'
}
[ordered]@{
    status = if ($Mode -eq 'Stage') { 'staged_for_next_policy_reload' } else { 'saved_policy_restored' }
    backup = $resolvedState
    adapter_restart_performed = $false
    power_plan_reactivated = $false
    link_unchanged = ($before.adapter_guid -eq $after.adapter_guid -and
        $before.adapter_status -eq $after.adapter_status -and
        (($before.ipv4 | Sort-Object) -join ',') -eq (($after.ipv4 | Sort-Object) -join ','))
    before = $before
    after = $after
} | ConvertTo-Json -Depth 6
