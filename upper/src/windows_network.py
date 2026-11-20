"""Read-only Windows context after transport faults; never changes a network."""
import json
import os
import subprocess
import threading
import time
import struct
import socket

import diagnostics

_guard = threading.Lock()
_last_capture = float('-inf')


def _route_gateways(data):
    """MIB_IPFORWARDTABLE: DWORD count followed by 14-DWORD IPv4 rows."""
    if len(data) < 4:
        return []
    count = struct.unpack_from('=I', data)[0]
    if count > (len(data) - 4) // 56:
        return []
    rows = [struct.unpack_from('=14I', data, 4 + index * 56) for index in range(count)]
    return list(dict.fromkeys(socket.inet_ntoa(struct.pack('=I', row[3]))
        for row in sorted(rows, key=lambda item: item[9])
        if row[0] == row[1] == 0 and row[3] and row[5] != 2))


def gateways():
    """Read Windows IPv4 gateways without scanning or changing a connection."""
    if os.name != 'nt':
        return []
    import ctypes
    size = ctypes.c_uint32()
    query = ctypes.WinDLL('iphlpapi').GetIpForwardTable
    query.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint32), ctypes.c_int]
    query.restype = ctypes.c_uint32
    if query(None, ctypes.byref(size), False) != 122 or not 4 <= size.value <= 1024 * 1024:
        return []
    table = ctypes.create_string_buffer(size.value)
    return _route_gateways(table.raw) if query(table, ctypes.byref(size), False) == 0 else []


_SCRIPT = r"""
$ErrorActionPreference='Stop'
[Console]::OutputEncoding=New-Object System.Text.UTF8Encoding($false)
$since=(Get-Date).AddMinutes(-2)
$interfaces=@([System.Net.NetworkInformation.NetworkInterface]::GetAllNetworkInterfaces() |
  Where-Object {$_.OperationalStatus -eq 'Up'} | ForEach-Object {
    $properties=$_.GetIPProperties()
    [pscustomobject]@{name=$_.Name;description=$_.Description;
      ipv4=@($properties.UnicastAddresses | Where-Object {$_.Address.AddressFamily -eq 'InterNetwork'} | ForEach-Object {$_.Address.ToString()});
      gateways=@($properties.GatewayAddresses | ForEach-Object {$_.Address.ToString()})}
  })
$events=@(foreach($source in @(
  @{log='Microsoft-Windows-WLAN-AutoConfig/Operational';ids=@(8001,8002,8003)},
  @{log='System';ids=@(4003);provider='Microsoft-Windows-WLAN-AutoConfig'},
  @{log='Microsoft-Windows-Wcmsvc/Operational';ids=@(1009,1010)})) {
    $filter=@{LogName=$source.log;StartTime=$since;Id=$source.ids}
    if($source.provider){$filter.ProviderName=$source.provider}
    Get-WinEvent -FilterHashtable $filter -MaxEvents 12 -ErrorAction SilentlyContinue | ForEach-Object {
      $xml=[xml]$_.ToXml();$fields=@{}
      foreach($item in $xml.Event.EventData.Data){
        if([string]$item.Name -in @('SSID','Reason','ReasonCode','FailureReason','InterfaceDescription')) {
          $fields[[string]$item.Name]=[string]$item.'#text'
        }
      }
      [pscustomobject]@{time=$_.TimeCreated.ToUniversalTime().ToString('o');log=$source.log;
        id=$_.Id;record=$_.RecordId;fields=$fields;
        message=if($_.Id -eq 4003 -or $_.Id -in @(1009,1010)){($_.Message -replace '\s+',' ')}else{''}}
    }
})
[pscustomobject]@{interfaces=$interfaces;events=$events} | ConvertTo-Json -Depth 6 -Compress
"""


def _collect():
    try:
        result = subprocess.run(['powershell.exe', '-NoProfile', '-NonInteractive', '-Command', _SCRIPT],
            capture_output=True, text=True, encoding='utf-8', errors='replace', timeout=8,
            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        if result.returncode:
            diagnostics.event('windows_network_error', reason='event_query_failed', returncode=result.returncode)
            return
        snapshot = json.loads(result.stdout)
        if isinstance(snapshot, dict):
            diagnostics.event('windows_network', **snapshot)
    except (OSError, ValueError, subprocess.TimeoutExpired) as exc:
        diagnostics.event('windows_network_error', reason=type(exc).__name__)
    finally:
        _guard.release()


def capture():
    """At most one bounded background query per 30 seconds, with no request delay."""
    global _last_capture
    if os.name != 'nt' or not diagnostics.active() or not _guard.acquire(blocking=False):
        return
    now = time.monotonic()
    if now - _last_capture < 30:
        _guard.release()
        return
    _last_capture = now
    try:
        threading.Thread(target=_collect, daemon=True, name='wifi-context').start()
    except RuntimeError:
        _guard.release()
