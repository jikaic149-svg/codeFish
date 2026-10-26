"""Folder-local Windows host, training, firmware and development launchers."""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import re
import runpy
import shutil
import subprocess
import sys
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT / 'project'
STATE = ROOT / 'state'
PYTHON = ROOT / 'runtime/python.exe'
IDF_PYTHON = ROOT / 'idf-env/Scripts/python.exe'
IDF = ROOT / 'tools/esp-idf'
BUILD = STATE / 'build/firmware'
ACTION_LABELS = {'start': '启动上位机', 'build': '编译固件', 'flash': 'USB 烧录（自动串口）',
                 'ota': 'OTA 升级（自动发现）', 'setup': '环境检查与补全'}


def execute(command, env, cwd=PROJECT, capture=False, log_path=None):
    command = [str(p) for p in command]
    if log_path is not None:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open('w', encoding='utf-8') as log:
            process = subprocess.Popen(command, cwd=cwd, env=env, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, text=True, encoding='utf-8', errors='replace')
            try:
                for line in process.stdout:
                    print(line, end='', flush=True)
                    log.write(line)
                code = process.wait()
            finally:
                if process.poll() is None:
                    process.terminate()
                    process.wait()
        if code:
            raise subprocess.CalledProcessError(code, command)
        return subprocess.CompletedProcess(command, code)
    return subprocess.run(command, cwd=cwd, env=env, check=True,
                          text=True, encoding='utf-8', errors='replace',
                          stdout=subprocess.PIPE if capture else None,
                          stderr=subprocess.STDOUT if capture else None)


def repair_environment():
    print('正在校验包内离线依赖备份并补齐缺失文件……', flush=True)
    manifest = json.loads((ROOT / 'manifest.json').read_text(encoding='utf-8'))
    backup = manifest.get('offline_environment')
    if not backup:
        raise RuntimeError('缺少离线环境备份，请重新复制完整交付文件夹。')
    archive_path = (ROOT / backup['path']).resolve()
    if not archive_path.is_relative_to(ROOT.resolve()):
        raise RuntimeError('离线环境备份路径不在包内。')
    with archive_path.open('rb') as stream:
        if hashlib.file_digest(stream, 'sha256').hexdigest() != backup['sha256']:
            raise RuntimeError('离线环境备份校验失败，请重新复制offline目录。')
    restored = []
    with zipfile.ZipFile(archive_path) as archive:
        members = []
        for item in archive.infolist():
            if item.is_dir():
                continue
            target = (ROOT / item.filename).resolve()
            if not target.is_relative_to(ROOT.resolve()):
                raise RuntimeError('离线备份含有越界路径。')
            parts = target.relative_to(ROOT.resolve()).parts
            if not (parts[0] in ('runtime', 'fonts') or parts[:3] == ('idf-env', 'Lib', 'site-packages')):
                raise RuntimeError('离线备份含有非环境文件。')
            members.append((item, target))
        for item, target in members:
            if target.is_file() and (target.stat().st_size > 0 or item.file_size == 0):
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(archive.read(item))
            restored.append(item.filename)
    return restored


def conclude(action, status, summary, **details):
    states = {'success': '成功', 'failed': '失败', 'cancelled': '已取消', 'unconfirmed': '未确认完成'}
    report = {'action': action, 'status': status, 'summary': summary,
              'completed_utc': datetime.now(timezone.utc).isoformat(), **details}
    print('\n【' + ACTION_LABELS.get(action, action) + '】' + states[status])
    print('结论：' + summary)
    for output in details.get('outputs', []):
        print('结果：' + str(output))
    destination = STATE / 'results' / (action + '.json')
    try:
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(report, ensure_ascii=False, indent=2, default=str), encoding='utf-8')
        print('记录：' + str(destination))
    except OSError as error:
        print('结论记录未能保存：' + str(error))
    return {'success': 0, 'failed': 1, 'cancelled': 130, 'unconfirmed': 2}[status]


def environment(idf=False):
    env = dict(os.environ)
    for name in ('PYTHONHOME', 'PYTHONPATH', 'VIRTUAL_ENV', 'CONDA_PREFIX', 'ELECTRON_RUN_AS_NODE'):
        env.pop(name, None)
    directories = {
        'APPDATA': 'appdata', 'LOCALAPPDATA': 'localappdata', 'TEMP': 'temp', 'TMP': 'temp',
        'PIP_CACHE_DIR': 'cache/pip', 'PYTHONUSERBASE': 'python-user',
        'XDG_CACHE_HOME': 'cache', 'XDG_CONFIG_HOME': 'config', 'XDG_DATA_HOME': 'data',
        'YOLO_CONFIG_DIR': 'config', 'MPLCONFIGDIR': 'config/matplotlib',
        'TORCH_HOME': 'cache/torch', 'TORCHINDUCTOR_CACHE_DIR': 'cache/torchinductor',
        'HF_HOME': 'cache/huggingface', 'CCACHE_DIR': 'cache/ccache',
        'PLAYWRIGHT_BROWSERS_PATH': 'browsers', 'ROBOFISH_EXPERIMENTS_DIR': 'experiments',
    }
    for name, relative in directories.items():
        path = STATE / relative
        path.mkdir(parents=True, exist_ok=True)
        env[name] = str(path)
    windir = Path(env.get('SystemRoot', 'C:/Windows'))
    bins = [ROOT / 'bin', ROOT / 'runtime', ROOT / 'runtime/Scripts',
            ROOT / 'tools/runtime-dlls', ROOT / 'tools/ucrt64/bin', ROOT / 'tools/cmake/bin',
            ROOT / 'tools/ninja', ROOT / 'tools/git/cmd', ROOT / 'tools/node',
            windir / 'System32', windir, windir / 'System32/Wbem',
            windir / 'System32/WindowsPowerShell/v1.0']
    tools = ROOT / 'tools/idf-tools/tools'
    for name in ('xtensa-esp-elf', 'xtensa-esp-elf-gdb', 'riscv32-esp-elf',
                 'riscv32-esp-elf-gdb', 'esp32ulp-elf', 'openocd-esp32', 'ccache', 'dfu-util', 'idf-exe'):
        base = tools / name
        for path in base.glob('*/*/bin'):
            bins.append(path)
        for path in base.glob('*/*'):
            if path.is_dir() and any(path.glob('*.exe')):
                bins.append(path)
        for path in base.glob('*'):
            if path.is_dir() and any(path.glob('*.exe')):
                bins.append(path)
    openocd = next((tools / 'openocd-esp32').glob('*/*/bin/openocd.exe'), None)
    if openocd is None:
        raise RuntimeError('包内OpenOCD工具缺失，请重新复制tools目录；本操作不会安装系统驱动。')
    roms = next((tools / 'esp-rom-elfs').iterdir())
    env.update(
        PATH=os.pathsep.join(str(p) for p in ([IDF_PYTHON.parent] if idf else []) + bins),
        PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1', PYTHONIOENCODING='utf-8',
        PIP_CONFIG_FILE=os.devnull, GIT_CONFIG_GLOBAL=str(STATE / 'gitconfig'), GIT_CONFIG_NOSYSTEM='1',
        ROBOFISH_WORKSPACE=str(PROJECT), ROBOFISH_VISION_PYTHON=str(PYTHON),
        ROBOFISH_IDF_PYTHON=str(IDF_PYTHON), ROBOFISH_CODE=str(ROOT / 'editor/Code.exe'),
        ROBOFISH_VISION_SAMPLES=str(ROOT / 'samples'), ROBOFISH_HOST_PORT=env.get('ROBOFISH_HOST_PORT', '8767'),
        ROBOFISH_CXX=str(ROOT / 'tools/ucrt64/bin/g++.exe'),
        IDF_PATH=str(IDF), IDF_TOOLS_PATH=str(ROOT / 'tools/idf-tools'),
        IDF_PYTHON_ENV_PATH=str(ROOT / 'idf-env'), IDF_COMPONENT_MANAGER='0', IDF_CCACHE_ENABLE='0',
        OPENOCD_SCRIPTS=str(openocd.parent.parent / 'share/openocd/scripts'),
        ESP_ROM_ELF_DIR=str(roms), ESP_IDF_VERSION='5.3',
        CUDA_VISIBLE_DEVICES='-1', YOLO_AUTOINSTALL='false', YOLO_OFFLINE='true',
        VSCODE_PORTABLE=str(ROOT / 'editor/data'),
        TCL_LIBRARY=str(ROOT / 'runtime/tcl/tcl8.6'), TK_LIBRARY=str(ROOT / 'runtime/tcl/tk8.6'),
    )
    # The old STM32 maintenance pane needs a different custom OpenOCD; do not misidentify ESP32 tools as that pane.
    return env


def initialize(force=False):
    env = environment()
    marker = STATE / 'initialized.json'
    saved = json.loads(marker.read_text(encoding='utf-8')) if marker.is_file() else {}
    font = ROOT / 'fonts/NotoSansSC-VF.ttf'
    if font.is_file():
        font_dir = Path(env['YOLO_CONFIG_DIR']) / 'Ultralytics'
        font_dir.mkdir(parents=True, exist_ok=True)
        for name in ('Arial.ttf', 'Arial.Unicode.ttf'):
            target = font_dir / name
            if not target.is_file():
                shutil.copy2(font, target)
    relocated = saved.get('root') != str(ROOT)
    if force or relocated or not IDF_PYTHON.is_file():
        execute([PYTHON, '-B', '-m', 'venv', '--without-pip', ROOT / 'idf-env'], env)
        if relocated and BUILD.exists():
            target = BUILD.resolve()
            assert target == (ROOT / 'state/build/firmware').resolve() and target.is_relative_to(STATE.resolve())
            assert not BUILD.is_symlink()
            shutil.rmtree(target)
        (ROOT / 'bin').mkdir(exist_ok=True)
        for name, action in (('idf', 'idf'), ('yolo', 'yolo')):
            (ROOT / 'bin' / (name + '.cmd')).write_text(
                '@echo off\n"%~dp0..\\runtime\\python.exe" -I -B "%~dp0..\\portable.py" '
                + action + ' %*\n', encoding='ascii')
        for name, executable, module in (('pip', 'runtime/python.exe', 'pip'),
                                          ('esptool', 'idf-env/Scripts/python.exe', 'esptool')):
            (ROOT / 'bin' / (name + '.cmd')).write_text('@echo off\n"%~dp0..\\'
                + executable.replace('/', '\\') + '" -B -m ' + module + ' %*\n', encoding='ascii')
        user = ROOT / 'editor/data/user-data/User'
        user.mkdir(parents=True, exist_ok=True)
        settings = {
            'telemetry.telemetryLevel': 'off', 'update.mode': 'none',
            'extensions.autoCheckUpdates': False, 'extensions.autoUpdate': False,
            'python.defaultInterpreterPath': str(PYTHON), 'python.terminal.activateEnvironment': False,
            'git.path': str(ROOT / 'tools/git/cmd/git.exe'),
            'cmake.cmakePath': str(ROOT / 'tools/cmake/bin/cmake.exe'),
            'C_Cpp.default.compilerPath': str(ROOT / 'tools/ucrt64/bin/g++.exe'),
            'terminal.integrated.defaultProfile.windows': 'Command Prompt',
            'terminal.integrated.env.windows': {key: env[key] for key in (
                'PATH', 'APPDATA', 'LOCALAPPDATA', 'TEMP', 'TMP', 'PIP_CACHE_DIR',
                'PIP_CONFIG_FILE', 'PYTHONNOUSERSITE', 'PYTHONDONTWRITEBYTECODE', 'PYTHONUSERBASE',
                'XDG_CACHE_HOME', 'XDG_CONFIG_HOME', 'XDG_DATA_HOME', 'YOLO_CONFIG_DIR',
                'MPLCONFIGDIR', 'TORCH_HOME', 'HF_HOME', 'CCACHE_DIR', 'PLAYWRIGHT_BROWSERS_PATH',
                'GIT_CONFIG_GLOBAL', 'GIT_CONFIG_NOSYSTEM', 'ROBOFISH_WORKSPACE',
                'ROBOFISH_VISION_PYTHON', 'ROBOFISH_IDF_PYTHON', 'ROBOFISH_CODE',
                'ROBOFISH_VISION_SAMPLES', 'ROBOFISH_EXPERIMENTS_DIR', 'ROBOFISH_HOST_PORT',
                'ROBOFISH_CXX', 'IDF_PATH', 'IDF_TOOLS_PATH', 'IDF_PYTHON_ENV_PATH',
                'IDF_COMPONENT_MANAGER', 'OPENOCD_SCRIPTS', 'ESP_ROM_ELF_DIR',
                'CUDA_VISIBLE_DEVICES', 'YOLO_AUTOINSTALL', 'YOLO_OFFLINE')},
        }
        (user / 'settings.json').write_text(json.dumps(settings, indent=2), encoding='utf-8')
        configs = []
        for action, name in ACTION_LABELS.items():
            configs.append({'name': name, 'type': 'debugpy', 'request': 'launch',
                'python': '${workspaceFolder}/runtime/python.exe',
                'program': '${workspaceFolder}/portable.py', 'args': [action],
                'cwd': '${workspaceFolder}', 'console': 'integratedTerminal',
                'justMyCode': True, 'subProcess': False,
                'env': {'PYTHONHOME': '', 'PYTHONPATH': '', 'PYTHONNOUSERSITE': '1',
                        'PYTHONDONTWRITEBYTECODE': '1', 'PYTHONIOENCODING': 'utf-8'}})
        launch = {'version': '0.2.0', 'configurations': configs}
        (ROOT / '.vscode').mkdir(exist_ok=True)
        launch_file = ROOT / '.vscode/launch.json'
        if not launch_file.exists():
            launch_file.write_text(json.dumps(launch, ensure_ascii=False, indent=2), encoding='utf-8')
        workspace = {'folders': [{'path': '.', 'name': 'RobotFish Dev'}], 'settings': {
            'files.exclude': {folder: True for folder in ('runtime', 'idf-env', 'tools', 'editor', 'state', 'fonts', 'bin')}
        }, 'tasks': {
            'version': '2.0.0', 'tasks': [
                {'label': 'RobotFish: ' + name, 'type': 'process', 'command': '${workspaceFolder}/runtime/python.exe',
                 'args': ['-I', '-B', '${workspaceFolder}/portable.py', action],
                 'options': {'cwd': '${workspaceFolder}/project'}, 'problemMatcher': []}
                for action, name in ACTION_LABELS.items()]
        }}
        (ROOT / 'RobotFish.code-workspace').write_text(json.dumps(workspace, indent=2), encoding='utf-8')
        marker.write_text(json.dumps({'root': str(ROOT), 'python': '3.13.14', 'idf': '5.3.2'}), encoding='utf-8')
    return env


def select_usb_port():
    from serial.tools.list_ports import comports
    ports = [item for item in comports() if re.fullmatch(r'COM[1-9][0-9]*', item.device, re.I)]
    native = [item for item in ports if item.vid == 0x303A]
    candidates = native or [item for item in ports if item.vid in (0x10C4, 0x1A86, 0x0403)
                            or 'esp32' in (item.description or '').lower()]
    candidates.sort(key=lambda item: int(item.device[3:]))
    if not candidates:
        raise RuntimeError('未检测到ESP32 USB或常见USB串口；请接好数据线并确认设备管理器已有串口驱动。烧录未开始。')
    if len(candidates) == 1:
        port = candidates[0].device.upper()
        print('自动检测到 USB 串口：' + port + '（' + (candidates[0].description or 'USB串口') + '）', flush=True)
        return port
    print('检测到多个候选串口，请选择目标设备：')
    for index, item in enumerate(candidates, 1):
        print(str(index) + '. ' + item.device + '  ' + (item.description or '') + '  ' + (item.serial_number or ''))
    try:
        choice = input('输入序号，留空取消：').strip()
    except EOFError:
        choice = ''
    if not choice:
        return None
    if not choice.isdigit() or not 1 <= int(choice) <= len(candidates):
        raise RuntimeError('串口序号无效，烧录未开始。')
    return candidates[int(choice) - 1].device.upper()


def ota_upgrade(extra):
    sys.path.insert(0, str(PROJECT / 'upper'))
    import device_client
    import robotfish
    if len(extra) > 1:
        raise RuntimeError('OTA最多接受一个指定设备地址。')
    if extra:
        host_address = device_client.display_host(extra[0])
        identity = device_client.CLIENT.identity(host_address)
        if not robotfish.validate_identity(identity):
            raise RuntimeError('指定地址不是RobotFish ESP32-S3，未上传固件。')
    else:
        print('正在当前网络自动发现 RobotFish……', flush=True)
        identity = device_client.CLIENT.discover_robotfish()
        if not identity:
            raise RuntimeError('当前网络未发现RobotFish；请先连接机器鱼Wi-Fi，再运行OTA。未上传固件。')
        host_address = identity['host']
    print('已发现 RobotFish：' + host_address + '；先编译当前源码。', flush=True)
    built = firmware_command('build', [])
    image = Path(built['outputs'][0]).read_bytes()
    log_path = STATE / 'logs/ota.log'
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open('w', encoding='utf-8') as log:
        def message(text, *args):
            print(text, flush=True)
            log.write(str(text) + '\n')
            log.flush()
        try:
            installed = robotfish.install(host_address,
                os.environ.get('ROBOFISH_OTA_PASSWORD', robotfish.DEFAULT_CONTROL_PASSWORD), image, message, message)
        except Exception as error:
            message(type(error).__name__ + '：' + str(error))
            raise
    if not installed.get('complete'):
        return {'status': 'unconfirmed', 'summary': '未确认OTA升级完成，请查看日志及设备状态。',
                'host': host_address, 'outputs': [str(log_path)]}
    return {'status': 'success', 'summary': 'OTA上传后已确认版本 ' + installed['version'] +
            '、切换到 ' + installed['slot'] + ' 并健康启动；运动效果未作实机验收。',
            'host': host_address, 'version': installed['version'], 'slot': installed['slot'],
            'sha256': built['sha256'], 'outputs': [built['outputs'][0], str(log_path)]}


def firmware_command(action, extra):
    env = environment(idf=True)
    command = [IDF_PYTHON, '-B', IDF / 'tools/idf.py', '-B', BUILD, '-DIDF_COMPONENT_MANAGER=0']
    manifest = json.loads((ROOT / 'manifest.json').read_text(encoding='utf-8'))
    command += ['-D' + key + '=' + value for key, value in manifest.get('cmake_defines', {}).items()]
    if action == 'flash':
        if not extra or not re.fullmatch(r'COM[1-9][0-9]*', extra[0], re.I):
            raise RuntimeError('串口无效；请填写实际COM端口（例如COM7）。烧录未开始。')
        command += ['-p', extra[0], 'flash', *extra[1:]]
    elif action == 'build':
        command += ['build', *extra]
    else:
        command += extra or ['--help']
    log_path = STATE / 'logs' / (action + '.log')
    execute(command, env, log_path=log_path)
    if action == 'build':
        description = json.loads((BUILD / 'project_description.json').read_text(encoding='utf-8'))
        version = description.get('project_version', 'unknown')
        short_version = re.sub(r'[^A-Za-z0-9_.-]', '_', version.split('-', 1)[0])
        target = ROOT / 'firmware' / ('RobotFish-ESP32S3-' + short_version + '-rebuilt.bin')
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(BUILD / 'robotfish.bin', target)
        for suffix in ('.elf', '.map'):
            shutil.copy2(BUILD / ('robotfish' + suffix), target.with_suffix(suffix))
        for relative, name in (('bootloader/bootloader.bin', 'bootloader.bin'),
                               ('partition_table/partition-table.bin', 'partition-table.bin'),
                               ('ota_data_initial.bin', 'ota_data_initial.bin')):
            shutil.copy2(BUILD / relative, ROOT / 'firmware' / name)
        return {'status': 'success', 'summary': '固件编译及分区大小检查通过，版本 ' + version + '；未执行烧录。',
                'outputs': [str(target)], 'version': version, 'bytes': target.stat().st_size,
                'sha256': hashlib.sha256(target.read_bytes()).hexdigest(), 'log': str(log_path)}
    if action == 'flash':
        metadata = json.loads((BUILD / 'flasher_args.json').read_text(encoding='utf-8'))
        expected = len(metadata.get('flash_files', {}))
        verified = log_path.read_text(encoding='utf-8').lower().count('hash of data verified')
        confirmed = expected > 0 and verified >= expected
        return {'status': 'success' if confirmed else 'unconfirmed',
                'summary': ('写入与全部镜像回读校验通过；固件实际启动及运动效果尚未验证。' if confirmed else
                            '烧录工具已正常返回，但未从日志确认全部镜像回读校验；本次不能标记为校验通过。'),
                'port': extra[0], 'expected_images': expected, 'verified_images': verified,
                'outputs': [str(log_path)], 'hardware_runtime_verified': False}
    return {'status': 'success', 'summary': 'IDF命令已正常结束。', 'outputs': [str(log_path)]}


def check(env):
    import cv2
    import numpy
    import flask
    import serial
    import torch
    import torchvision
    from ultralytics import YOLO
    from ultralytics.utils import SETTINGS
    settings = {'datasets_dir': str(ROOT / 'datasets'), 'runs_dir': str(STATE / 'training'),
                'weights_dir': str(PROJECT / 'vision/fish_body_yolo'), 'sync': False,
                'hub': False, 'wandb': False, 'comet': False, 'mlflow': False}
    SETTINGS.update({key: value for key, value in settings.items() if key in SETTINGS})
    assert torch.version.cuda is None, 'The delivery must use the CPU build'
    assert Path(sys.executable).resolve().is_relative_to(ROOT), sys.executable
    assert all(not p or Path(p).resolve().is_relative_to(ROOT) or 'debugpy' in p.lower() for p in sys.path), sys.path
    assert not (PROJECT / 'docs').exists()
    torch.set_num_threads(2)
    counts = {}
    for split in ('train', 'val', 'test'):
        images = sorted((ROOT / 'datasets/fish/images' / split).glob('*'))
        counts[split] = len(images)
        assert images
        for image in images:
            label = ROOT / 'datasets/fish/labels' / split / (image.stem + '.txt')
            assert label.is_file()
            for row in label.read_text().splitlines():
                values = row.split()
                assert len(values) == 5 and int(values[0]) == 0
                assert all(0 <= float(v) <= 1 for v in values[1:])
    sample = sorted((ROOT / 'datasets/fish/images/test').glob('*'))[0]
    image = cv2.imread(str(sample))
    assert image is not None
    model = YOLO(str(PROJECT / 'vision/fish_body_yolo/best.pt'))
    prediction = model.predict(image, imgsz=960, device='cpu', verbose=False)[0]
    report = {'ok': True, 'root': str(ROOT), 'python': sys.version.split()[0],
              'torch': torch.__version__, 'opencv': cv2.__version__, 'numpy': numpy.__version__,
              'ultralytics': importlib.metadata.version('ultralytics'),
              'detections': len(prediction.boxes), 'dataset': counts, 'docs_included': False,
              'recipient_hardware_tested': False}
    versions = {}
    commands = {'git': [ROOT / 'tools/git/cmd/git.exe', '--version'],
                'cmake': [ROOT / 'tools/cmake/bin/cmake.exe', '--version'],
                'ninja': [ROOT / 'tools/ninja/ninja.exe', '--version'],
                'cxx': [ROOT / 'tools/ucrt64/bin/g++.exe', '--version'],
                'node': [ROOT / 'tools/node/node.exe', '--version'],
                'idf_python': [IDF_PYTHON, '-B', '-c', 'import esptool,cryptography,serial; print("IDF Python packages OK")']}
    tools = ROOT / 'tools/idf-tools/tools'
    commands['openocd'] = [next((tools / 'openocd-esp32').glob('*/*/bin/openocd.exe')), '--version']
    commands['gdb'] = [next((tools / 'xtensa-esp-elf-gdb').glob('*/*/bin/xtensa-esp32s3-elf-gdb.exe')), '--version']
    for name, command in commands.items():
        versions[name] = execute(command, environment(idf=name == 'idf_python'), capture=True).stdout.splitlines()[0]
    report['tools'] = versions
    (STATE / 'check.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    return report


def tests(env):
    compiler = ROOT / 'tools/ucrt64/bin/g++.exe'
    print('NOTE: historical tests/attitude/regression.cpp assumes Kp=5 and sub-deadband convergence; retained outside this targeted regression set.')
    for script in ('motion/run.ps1', 'control/run_calibration.ps1',
                   'wifi_http/run.ps1', 'radio_board/run.ps1'):
        execute(['powershell.exe', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File',
                 PROJECT / 'tests' / script, '-Compiler', compiler], environment(idf=True))
    for name in ('test_curve_speed.py', 'test_path_guidance.py', 'test_measured_speed_profile.py',
                 'test_observer_continuity.py', 'test_vision_continuity.py',
                 'test_dynamics_calibration.py', 'test_dynamics_control.py', 'test_video_recording.py'):
        execute([PYTHON, '-B', '-m', 'unittest', 'discover', '-s', 'upper/tests', '-p', name, '-v'], env)
    execute([ROOT / 'tools/node/node.exe', 'upper/tests/test_dynamics_steps.js'], env)
    execute([ROOT / 'tools/node/node.exe', 'tests/control/web_status.js'], env)
    print('PASS: portable native, Python and JavaScript regression checks')


def host(action, extra, env):
    command = [PYTHON, '-B', PROJECT / 'upper/launch.py', '--no-browser']
    if action == 'stop': command.append('--stop')
    execute(command + extra, env)
    if action != 'start' or '--no-browser' in extra:
        return
    browser = None
    for variable in ('PROGRAMFILES(X86)', 'PROGRAMFILES'):
        folder = env.get(variable)
        if folder:
            candidate = Path(folder) / 'Microsoft/Edge/Application/msedge.exe'
            if candidate.is_file(): browser = candidate; break
    if browser is None:
        os.startfile('http://127.0.0.1:' + env['ROBOFISH_HOST_PORT'] + '/')
        return
    log_path = STATE / 'logs/browser.log'
    log_path.parent.mkdir(parents=True, exist_ok=True)
    browser_env = dict(env, ROBOFISH_BROWSER_EXE=str(browser), ROBOFISH_BROWSER_ROOT=str(ROOT),
        ROBOFISH_BROWSER_ARGS=subprocess.list2cmdline([
            '--user-data-dir=' + str(STATE / 'browser'), '--no-first-run', '--no-default-browser-check',
            '--disable-gpu', '--disable-background-mode', '--enable-logging', '--log-level=2',
            '--log-file=' + str(log_path), 'http://127.0.0.1:' + env['ROBOFISH_HOST_PORT'] + '/']))
    # Let the Windows desktop own the browser instead of the short-lived debug/terminal process.
    execute(['powershell.exe', '-NoProfile', '-NonInteractive', '-Command',
             "$ErrorActionPreference='Stop'\n"
             "$desktopShell=New-Object -ComObject Shell.Application\n"
             "$desktopShell.ShellExecute($env:ROBOFISH_BROWSER_EXE,$env:ROBOFISH_BROWSER_ARGS,"
             "$env:ROBOFISH_BROWSER_ROOT,'open',1)"], browser_env, cwd=ROOT, capture=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('init', 'setup', 'start', 'stop', 'debug-host', 'shell', 'editor', 'build',
                        'flash', 'ota', 'idf', 'check', 'test', 'delivery', 'train', 'validate', 'smoke', 'yolo'))
    args, extra = parser.parse_known_args()
    if args.action in ACTION_LABELS:
        print('正在' + ACTION_LABELS[args.action] + '，请等待……', flush=True)
    repaired = repair_environment() if args.action == 'setup' else []
    env = initialize(force=args.action in ('init', 'setup'))
    os.environ.clear()
    os.environ.update(env)
    result = {'status': 'success', 'summary': '所选命令已正常结束。'}
    if args.action == 'init':
        result['summary'] = '包内环境路径和配置已初始化；尚未执行完整依赖或硬件检查。'
    elif args.action in ('build', 'flash', 'idf'):
        if args.action == 'flash' and not extra:
            port = select_usb_port()
            if port is None:
                return args.action, {'status': 'cancelled', 'summary': '已取消选择USB串口，烧录未开始。'}
            extra = [port]
        result = firmware_command(args.action, extra)
    elif args.action == 'ota':
        result = ota_upgrade(extra)
    elif args.action in ('check', 'setup'):
        checked = check(env)
        result.update(summary=('软件环境检查通过；补齐' + str(len(repaired)) + '个缺失文件并更新包内配置。' if args.action == 'setup'
                               else 'Python、工具链、数据集及CPU模型推理检查通过。') + '摄像头、串口和驱动未作实机验证。',
                      restored_files=repaired, checks=checked, outputs=[str(STATE / 'check.json')])
    elif args.action == 'test':
        tests(env)
        result['summary'] = '当前关键原生、Python和JavaScript回归集合全部通过；不包含已标注的旧姿态测试。'
    elif args.action == 'delivery':
        execute([PYTHON, '-B', ROOT / 'verify_delivery.py', '--root', ROOT], env, ROOT)
    elif args.action in ('start', 'stop'):
        host(args.action, extra, env)
        result['summary'] = ('上位机本机后台已就绪：http://127.0.0.1:' + env['ROBOFISH_HOST_PORT'] +
                             '/；设备连接状态以页面显示为准。' if args.action == 'start'
                             else '本包上位机后台已停止或确认未在运行。')
    elif args.action == 'debug-host':
        os.chdir(PROJECT / 'upper')
        sys.path.insert(0, str(PROJECT / 'upper'))
        runpy.run_path(str(PROJECT / 'upper/server.py'), run_name='__main__')
        result['summary'] = '调试后台已正常退出。'
    elif args.action == 'shell':
        execute(['cmd.exe', '/K', 'title RobotFish Portable Development'], env)
        result['summary'] = '包内开发终端已正常退出。'
    elif args.action == 'editor':
        subprocess.Popen([str(ROOT / 'editor/Code.exe'), '--user-data-dir', str(ROOT / 'editor/data/user-data'),
                          '--extensions-dir', str(ROOT / 'editor/data/extensions'),
                          '--new-window', str(ROOT / 'RobotFish.code-workspace')],
                         cwd=PROJECT, env=env, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        result['summary'] = '已提交打开便携编辑器的请求；工作区为RobotFish.code-workspace。'
    elif args.action in ('train', 'validate', 'smoke'):
        execute([PYTHON, '-B', ROOT / 'portable_training.py', args.action, *extra], env, ROOT)
    elif args.action == 'yolo':
        execute([PYTHON, '-B', '-c', 'from ultralytics.cfg import entrypoint; entrypoint()', *extra], env)
    return args.action, result


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    sys.stderr.reconfigure(encoding='utf-8')
    action = sys.argv[1] if len(sys.argv) > 1 else 'operation'
    try:
        action, result = main()
    except KeyboardInterrupt:
        result = {'status': 'cancelled', 'summary': '操作已被中断，本次未确认完成。'}
    except subprocess.CalledProcessError as error:
        if error.output:
            print(error.output)
        cancelled = error.returncode in (130, -2, 3221225786, -1073741510)
        program = Path(str(error.cmd[0])).name if isinstance(error.cmd, (list, tuple)) else '子程序'
        result = {'status': 'cancelled' if cancelled else 'failed',
                  'summary': '操作被中断，未确认完成。' if cancelled else
                             program + '执行失败（退出码' + str(error.returncode) + '）；流程未完成，请查看上方首个错误。',
                  'returncode': error.returncode}
        log = STATE / 'logs' / (action + '.log')
        if log.is_file(): result['outputs'] = [str(log)]
    except Exception as error:
        result = {'status': 'failed', 'summary': '操作未完成：' + type(error).__name__ + '：' + str(error)}
    raise SystemExit(conclude(action, **result))
