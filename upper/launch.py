"""Start the local RobotFish service if needed, then open the UI in the browser.

Designed to be launched by pythonw.exe through start.cmd, so it must never
require a console. Failures surface as a Windows message box with the tail of
server.log, because a silent failure here looks identical to "nothing happened".
"""
from collections import deque
from pathlib import Path
import json
import os
import subprocess
import sys
import time
import urllib.request
import webbrowser

ROOT = Path(__file__).resolve().parent
PORT = int(os.environ.get('ROBOFISH_HOST_PORT', '8766'))
URL = f'http://127.0.0.1:{PORT}'
LOG = ROOT / 'server.log'
SERVICE_ID = 'robotfish-local'
REVISION = 'workstation-37-host-logic-fixes'
BOOT_TIMEOUT_SECONDS = 20.0


def alert(message):
    """Report a startup failure without depending on a console."""
    if os.name == 'nt' and sys.stderr is None:
        import ctypes
        ctypes.windll.user32.MessageBoxW(None, message, '机器鱼工作站', 0x10)
    print(message, file=sys.stderr)


def probe():
    """Return the running service identity, or None when nothing answers."""
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(URL + '/api/health', timeout=1) as response:
            return json.load(response)
    except Exception:
        return None


def log_tail(lines=8):
    if not LOG.exists():
        return '(server.log 尚未生成)'
    with LOG.open('r', encoding='utf-8', errors='replace') as handle:
        return ''.join(deque(handle, maxlen=lines)).strip() or '(日志为空)'


def spawn():
    LOG.touch(exist_ok=True)
    # sys.executable is pythonw.exe when started from start.cmd, so the child
    # inherits a windowless interpreter. File handles keep it alive after exit.
    with LOG.open('a', encoding='utf-8') as sink:
        return subprocess.Popen(
            [sys.executable, str(ROOT / 'server.py')], cwd=ROOT,
            stdout=sink, stderr=sink,
            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))


def local_request(path, body=None):
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    headers = {'Content-Type': 'application/json', 'Origin': URL}
    if body is not None:
        headers['X-AmphiLink-Token'] = local_request('session')['token']
    request = urllib.request.Request(URL + '/api/' + path, headers=headers,
        data=json.dumps(body).encode() if body is not None else None)
    with opener.open(request, timeout=8) as response:
        return json.load(response)


def same_instance(identity):
    return (isinstance(identity, dict) and identity.get('service') == SERVICE_ID and
            os.path.normcase(identity.get('instance', '')) == os.path.normcase(str(ROOT)))


def legacy_stop():
    """Old releases have no exit API: verify this exact server process first."""
    if os.name != 'nt':
        raise RuntimeError('旧版服务不支持退出接口，请手动结束本目录的 server.py')
    state = local_request('state')
    if state.get('connected') or state.get('session_active'):
        raise RuntimeError('旧版仍连接调试目标，请先断开调试会话再结束')
    try:
        view = local_request('station')
        control = view.get('control', {})
        if control.get('connected') and control.get('session'):
            local_request('station/control', {'action':'stop','session':control['session'],
                'sequence':1,'client_id':'exit-' + str(time.monotonic_ns())})
    except urllib.error.URLError:
        pass  # The explicitly requested local exit must also work when the fish is offline.
    output = subprocess.check_output(['netstat', '-ano', '-p', 'tcp'],
        creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0)).decode('ascii', errors='ignore')
    pids = {int(parts[-1]) for line in output.splitlines() if len(parts := line.split()) == 5
            and parts[1] == f'127.0.0.1:{PORT}' and parts[3] == 'LISTENING'}
    if len(pids) != 1:
        raise RuntimeError('无法唯一确认本机上位机监听进程，未结束任何程序')
    script = r'''
$ErrorActionPreference = 'Stop'
$taskProcess = Get-CimInstance Win32_Process -Filter ('ProcessId = ' + $env:ROBOTFISH_STOP_PID)
$taskExpected = [regex]::Escape($env:ROBOTFISH_SERVER_SCRIPT) + '(?=["\s]|$)'
if ($taskProcess.Name -notin @('python.exe','pythonw.exe') -or
    $taskProcess.CommandLine -notmatch $taskExpected) { throw 'Process is not this RobotFish server.py' }
Stop-Process -Id ([int]$env:ROBOTFISH_STOP_PID) -ErrorAction Stop
'''
    env = dict(os.environ, ROBOTFISH_STOP_PID=str(pids.pop()), ROBOTFISH_SERVER_SCRIPT=str(ROOT / 'server.py'))
    subprocess.run(['powershell.exe', '-NoProfile', '-Command', script], env=env,
        check=True, timeout=10, capture_output=True,
        creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))


def stop_service(identity):
    if not same_instance(identity):
        raise RuntimeError('端口属于其他程序或其他目录，未结束该进程')
    saved, raw = local_request('data'), local_request('telemetry')
    unsaved = saved.get('recording') or saved.get('count') or raw.get('count')
    if unsaved:
        import ctypes
        if os.name != 'nt' or ctypes.windll.user32.MessageBoxW(None,
                f"当前有 {saved.get('count',0)} 条状态记录、{raw.get('count',0)} 条原始记录。\n"
                '结束后台会丢失未导出的数据。确认放弃并结束？', '机器鱼工作站', 0x24) != 6:
            raise RuntimeError('已取消退出，请先导出实验数据')
    if identity.get('shutdown_supported') is True:
        local_request('shutdown', {'discard_records': bool(unsaved)})
    else:
        legacy_stop()
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        if probe() is None:
            return
        time.sleep(.1)
    raise RuntimeError('旧后台尚未退出，未重复启动新进程')


def main():
    open_browser = '--no-browser' not in sys.argv[1:]
    identity = probe()

    if identity is not None and same_instance(identity) and ('--stop' in sys.argv[1:] or
            '--restart' in sys.argv[1:] or identity.get('revision') != REVISION):
        try:
            stop_service(identity)
            identity = None
        except Exception as error:
            alert('无法结束旧上位机：' + str(error))
            return 1
    if '--stop' in sys.argv[1:]:
        if identity is not None:
            alert('端口属于其他程序或其他目录，未结束该进程')
            return 1
        return 0

    if identity is None:
        process = spawn()
        deadline = time.monotonic() + BOOT_TIMEOUT_SECONDS
        while time.monotonic() < deadline:
            identity = probe()
            if identity:
                break
            if process.poll() is not None:
                alert('本地服务启动后立即退出。\n\n最后日志：\n' + log_tail())
                return 1
            time.sleep(0.2)
        else:
            alert(f'本地服务在 {BOOT_TIMEOUT_SECONDS:.0f} 秒内没有就绪。\n\n'
                  f'可能原因：端口 {PORT} 被占用、缺少依赖模块、或 openocd 目录不完整。\n\n'
                  '最后日志：\n' + log_tail())
            return 1

    if not isinstance(identity,dict) or identity.get('service') != SERVICE_ID:
        alert(f'端口 {PORT} 已被另一个程序占用，无法启动上位机本地服务。\n\n'
              '请关闭占用该端口的程序后重试。')
        return 1

    if identity.get('revision') != REVISION or not same_instance(identity):
        alert('本机正在运行旧版或其他目录的上位机。\n\n请先保存实验数据并结束旧服务，再重新双击图标。\n当前入口：'+str(ROOT))
        return 1

    if open_browser:
        webbrowser.open(URL)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
