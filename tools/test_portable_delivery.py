"""Exercise the assembled delivery offline, without cameras, serial ports or fish commands."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
import time
import urllib.request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True, type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    spec = importlib.util.spec_from_file_location('portable', root / 'portable.py')
    portable = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(portable)
    env = portable.initialize(force=True)
    python = root / 'runtime/python.exe'
    report = {'offline_training': False, 'host_start_stop': False, 'editor_cli': False,
              'real_camera_or_serial_opened': False, 'real_fish_commands_sent': False}
    launch = json.loads((root / '.vscode/launch.json').read_text(encoding='utf-8'))
    workspace = json.loads((root / 'RobotFish.code-workspace').read_text(encoding='utf-8'))
    assert workspace['folders'][0]['path'] == '.'
    assert len(launch['configurations']) == 5
    operations = {configuration['args'][0] for configuration in launch['configurations']}
    assert operations == {'start', 'build', 'flash', 'ota', 'setup'}
    for configuration in launch['configurations']:
        assert configuration['type'] == 'debugpy'
        assert Path(configuration['python'].replace('${workspaceFolder}', str(root))).is_file()
        assert Path(configuration['program'].replace('${workspaceFolder}', str(root))).is_file()
    report['launch_configurations'] = len(launch['configurations'])
    report['public_operations'] = sorted(operations)
    try:
        portable.firmware_command('flash', [''])
        raise AssertionError('Blank flash port was not rejected')
    except RuntimeError as error:
        assert 'port' in str(error) or '串口' in str(error)
    report['blank_flash_port_rejected'] = True

    def run(command, capture=False):
        result = subprocess.run([str(p) for p in command], cwd=root / 'project', env=env,
                                capture_output=capture, text=True, encoding='utf-8',
                                errors='replace', timeout=180)
        if result.returncode:
            if capture: print(result.stdout, result.stderr)
            raise RuntimeError('Validation command failed: ' + str(command))
        return result

    run([python, '-B', '-m', 'pip', 'check'])
    code = '''import os,runpy,socket,sys
original=socket.getaddrinfo
def local_only(host,*args,**kwargs):
    if host not in ('127.0.0.1','localhost','::1',None):
        raise OSError('External network disabled by portable validation')
    return original(host,*args,**kwargs)
socket.getaddrinfo=local_only
sys.argv=[sys.argv[1],'smoke']
runpy.run_path(sys.argv[0],run_name='__main__')
'''
    run([python, '-B', '-c', code, root / 'portable_training.py'])
    report['offline_training'] = True
    editor = run(['cmd.exe', '/d', '/c', root / 'editor/bin/code.cmd', '--version'], capture=True)
    assert editor.stdout.strip(), 'Editor CLI returned no version'
    report['editor_cli'] = editor.stdout.strip().splitlines()
    with socket.socket() as handle:
        handle.bind(('127.0.0.1', 0))
        port = handle.getsockname()[1]
    env['ROBOFISH_HOST_PORT'] = str(port)
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    address = 'http://127.0.0.1:' + str(port)
    try:
        run([python, '-I', '-B', root / 'portable.py', 'start', '--no-browser'])
        with opener.open(address + '/api/health', timeout=4) as response:
            health = json.load(response)
        assert Path(health['instance']).resolve() == root / 'project/upper'
        for path in ('/', '/vision.js', '/dynamics.html', '/style.css'):
            with opener.open(address + path, timeout=4) as response:
                assert response.status == 200 and response.read()
        with opener.open(address + '/api/vision/config', timeout=4) as response:
            config = json.load(response)
        assert config['default_model'] == 'vision/models/fish_body_yolo/best.pt'
    finally:
        run([python, '-I', '-B', root / 'portable.py', 'stop', '--no-browser'])
    report['host_start_stop'] = True
    debug_log = root / 'state/debug-host-validation.log'
    with debug_log.open('w', encoding='utf-8') as log:
        backend = subprocess.Popen([str(python), '-I', '-B', str(root / 'portable.py'), 'debug-host'],
                                   cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            for _ in range(100):
                try:
                    with opener.open(address + '/api/health', timeout=.3) as response:
                        health = json.load(response)
                    break
                except OSError:
                    if backend.poll() is not None:
                        raise RuntimeError('Debug backend exited before readiness')
                    time.sleep(.1)
            else:
                raise RuntimeError('Debug backend did not become ready')
            assert Path(health['instance']).resolve() == root / 'project/upper'
            run([python, '-I', '-B', root / 'portable.py', 'stop', '--no-browser'])
            assert backend.wait(timeout=15) == 0
            report['debug_host_start_stop'] = True
        finally:
            if backend.poll() is None:
                backend.terminate()
                backend.wait(timeout=10)
    (root / 'state/delivery-validation.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
