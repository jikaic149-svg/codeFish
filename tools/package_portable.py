"""Assemble the project's Windows development folder from installed, versioned tools."""
from concurrent.futures import ThreadPoolExecutor
import argparse
import hashlib
import json
import os
import re
from pathlib import Path
import shutil
import stat
import subprocess
import zipfile

SOURCE = Path(__file__).resolve().parents[1]
BASE = Path('D:/python/3.13')
IDF = Path('D:/ESP32/ESP-IDF-v5.3.2')
IDF_TOOLS = Path('D:/ESP32/.espressif')
CPU_ZIP = SOURCE.parents[0] / 'codefish/build/releases/Xihu-VISION-English-Portable-Win64-20261020.zip'


def ignore(_directory, names):
    return [n for n in names if n in {'.git', '__pycache__', '.pytest_cache', '.mypy_cache'}
            or n.endswith(('.pyc', '.pyo', '-backup'))]


def copy_tree(source, destination):
    print('COPY', str(source), flush=True)
    def copy_file(src, dst):
        source_info = Path(src).stat()
        if Path(dst).is_file():
            target_info = Path(dst).stat()
            if source_info.st_size == target_info.st_size and source_info.st_mtime_ns == target_info.st_mtime_ns:
                return dst
            Path(dst).chmod(stat.S_IREAD | stat.S_IWRITE)
        return shutil.copy2(src, dst)
    shutil.copytree(source, destination, ignore=ignore, dirs_exist_ok=True, copy_function=copy_file)


def copy_package(source, destination, *names):
    for name in names:
        for item in source.glob(name):
            if item.is_dir():
                copy_tree(item, destination / item.name)
            elif item.is_file():
                destination.mkdir(parents=True, exist_ok=True)
                shutil.copy2(item, destination / item.name)


def create_environment_backup(root):
    """Keep a clean offline source for missing Python dependencies; never include account state."""
    archive = root / 'offline/environment.zip'
    archive.parent.mkdir(exist_ok=True)
    pending = archive.with_suffix('.zip.new')
    print('PREPARE offline Python dependency backup', flush=True)
    with zipfile.ZipFile(pending, 'w', zipfile.ZIP_DEFLATED, compresslevel=1) as output:
        for folder in ('runtime', 'idf-env/Lib/site-packages', 'fonts'):
            for directory, dirs, files in os.walk(root / folder):
                dirs[:] = [name for name in dirs if name != '__pycache__']
                for name in files:
                    if name.endswith(('.pyc', '.pyo')):
                        continue
                    file = Path(directory) / name
                    output.write(file, file.relative_to(root).as_posix())
    os.replace(pending, archive)
    with archive.open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    return {'path': archive.relative_to(root).as_posix(), 'sha256': digest, 'bytes': archive.stat().st_size}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--resume', action='store_true', help='Continue assembling this tool\'s incomplete delivery')
    args = parser.parse_args()
    output = args.output.resolve()
    assert output.name.startswith('RobotFish-Dev'), output
    if args.resume:
        assert (output / 'project/CMakeLists.txt').is_file() and (output / 'runtime/python.exe').is_file()
    else:
        assert not output.exists(), output
    assert not output.is_relative_to(SOURCE), 'Use a dedicated delivery folder outside the source tree'
    assert CPU_ZIP.is_file(), CPU_ZIP
    output.mkdir(parents=True, exist_ok=args.resume)
    project = output / 'project'
    project.mkdir(exist_ok=True)
    for folder in ('main', 'upper', 'vision'):
        def project_ignore(directory, names):
            excluded = set(ignore(directory, names))
            excluded.update(n for n in names if n in {'logs', 'data', 'runs', 'experiments', 'Ultralytics'}
                            or n.endswith('.local.json') or n == 'server.log')
            return list(excluded)
        shutil.copytree(SOURCE / folder, project / folder, ignore=project_ignore, dirs_exist_ok=True)
    for name in ('.gitattributes', '.gitignore', 'CMakeLists.txt', 'partitions.csv',
                 'sdkconfig', 'sdkconfig.defaults', 'README.md', 'process.md', 'build.ps1'):
        shutil.copy2(SOURCE / name, project / name)
    # The current model metadata and previews are small; old training console logs are not runtime input.
    for p in (project / 'vision/models/fish_body_yolo').glob('*.log'):
        p.unlink()
    (project / 'README.md').write_text(
        '> Portable delivery: docs/ was excluded at the user\'s request. Open ../Editor.cmd and use ../.vscode/launch.json; the parent README.txt describes portable paths. The original source-project notes follow.\n\n'
        + (project / 'README.md').read_text(encoding='utf-8'), encoding='utf-8')

    runtime = output / 'runtime'
    runtime.mkdir(exist_ok=True)
    for item in BASE.iterdir():
        if item.is_file() and (item.suffix in ('.exe', '.dll') or item.name == 'LICENSE.txt'):
            shutil.copy2(item, runtime / item.name)
    for folder in ('DLLs', 'Lib', 'include', 'libs', 'tcl'):
        def base_ignore(directory, names):
            return list(set(ignore(directory, names)) | ({'site-packages'} if Path(directory) == BASE / 'Lib' else set()))
        shutil.copytree(BASE / folder, runtime / folder, ignore=base_ignore, dirs_exist_ok=True)
    site = runtime / 'Lib/site-packages'
    site.mkdir(exist_ok=True)
    print('EXTRACT verified CPU Python packages', flush=True)
    with zipfile.ZipFile(CPU_ZIP) as archive:
        for info in archive.infolist():
            parts = Path(info.filename).parts
            if len(parts) > 1 and parts[1] == 'fonts' and not info.is_dir():
                target = output / 'fonts' / Path(info.filename).name
                target.parent.mkdir(exist_ok=True)
                target.write_bytes(archive.read(info))
            if 'runtime' not in parts:
                continue
            relative = Path(*parts[parts.index('runtime') + 1:])
            if not relative.parts:
                continue
            package_file = relative.parts[:2] == ('Lib', 'site-packages')
            extra_dll = len(relative.parts) == 1 and relative.suffix == '.dll' and not relative.name.startswith('python')
            if not (package_file or extra_dll) or info.is_dir() or '__pycache__' in relative.parts:
                continue
            target = (runtime / relative).resolve()
            assert target.is_relative_to(runtime.resolve()), target
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(archive.read(info))
    base_site = BASE / 'Lib/site-packages'
    copy_package(base_site, site, 'pip', 'pip-*.dist-info', 'serial', 'pyserial-*.dist-info',
                 'playwright', 'playwright-*.dist-info', 'greenlet', 'greenlet-*.dist-info',
                 'pyee', 'pyee-*.dist-info')
    copy_package(Path('D:/python/venvs/yolo/ultralytics/8.4.118/Lib/site-packages'), site, 'numpy-*.dist-info')
    idf_site = output / 'idf-env/Lib/site-packages'
    copy_tree(IDF_TOOLS / 'python_env/idf5.3_py3.13_env/Lib/site-packages', idf_site)
    for file in runtime.glob('*.dll'):
        # Compiler/debugger helpers also need the MSVC runtime on a clean Windows installation.
        if file.name.startswith(('vcruntime', 'msvcp', 'concrt')):
            target = output / 'tools/runtime-dlls' / file.name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(file, target)

    jobs = [(IDF, output / 'tools/esp-idf'), (IDF_TOOLS / 'tools', output / 'tools/idf-tools/tools'),
            (Path('D:/CMAKE'), output / 'tools/cmake'), (Path('D:/Ninja'), output / 'tools/ninja'),
            (Path('C:/msys64/ucrt64'), output / 'tools/ucrt64')]
    with ThreadPoolExecutor(max_workers=3) as pool:
        for result in pool.map(lambda job: copy_tree(*job), jobs):
            pass
    git = output / 'tools/git'
    for folder in ('bin', 'cmd', 'mingw64', 'usr', 'etc'):
        copy_tree(Path('D:/Git') / folder, git / folder)
    shutil.copy2('D:/Git/LICENSE.txt', git / 'LICENSE.txt')
    (output / 'tools/node').mkdir(parents=True, exist_ok=True)
    shutil.copy2('D:/Node.js/node-v24.19.0-win-x64/node.exe', output / 'tools/node/node.exe')
    for name in ('LICENSE',):
        p = Path('D:/Node.js/node-v24.19.0-win-x64') / name
        if p.is_file(): shutil.copy2(p, output / 'tools/node' / name)
    for name in ('espidf.constraints.v5.3.txt',):
        shutil.copy2(IDF_TOOLS / name, output / 'tools/idf-tools' / name)
    (output / 'tools/esp-idf/version.txt').write_text('v5.3.2\n', encoding='ascii')
    (output / 'tools/idf-tools/idf-env.json').write_text(json.dumps({
        'idfSelectedId': 'portable', 'idfInstalled': {'portable': {
            'version': '5.3', 'path': str(output / 'tools/esp-idf'),
            'features': ['core'], 'targets': ['esp32s3']}}}), encoding='utf-8')

    editor = output / 'editor'
    editor.mkdir(exist_ok=True)
    shutil.copy2('D:/Microsoft VS Code/Code.exe', editor / 'Code.exe')
    for folder in ('bin', 'e646868227'):
        copy_tree(Path('D:/Microsoft VS Code') / folder, editor / folder)
    (editor / 'data/extensions').mkdir(parents=True, exist_ok=True)
    ext_source = Path.home() / '.vscode/extensions'
    for pattern in ('ms-python.python-*', 'ms-python.debugpy-*', 'ms-python.vscode-pylance-*',
                    'ms-python.vscode-python-envs-*', 'ms-vscode.cpptools-*', 'ms-vscode.cmake-tools-*'):
        matches = sorted(ext_source.glob(pattern))
        if matches:
            copy_tree(matches[-1], editor / 'data/extensions' / matches[-1].name)

    dataset = output / 'datasets/fish'
    augmented = SOURCE / 'vision/experiments/fish_body_yolo/runs/yolov8n-960-robust-v3/augmentation_dataset'
    for split in ('train', 'val', 'test'):
        image_dir = dataset / 'images' / split
        label_dir = dataset / 'labels' / split
        image_dir.mkdir(parents=True, exist_ok=True)
        label_dir.mkdir(parents=True, exist_ok=True)
        for line in (augmented / 'splits' / (split + '.txt')).read_text(encoding='utf-8-sig').splitlines():
            image = Path(line.strip())
            if not image.is_absolute(): image = (augmented / 'splits' / image).resolve()
            assert image.is_file(), image
            parts = list(image.parts)
            pos = len(parts) - 1 - parts[::-1].index('images')
            parts[pos] = 'labels'
            label = Path(*parts).with_suffix('.txt')
            assert label.is_file(), label
            if (image_dir / image.name).exists():
                assert args.resume and (image_dir / image.name).read_bytes() == image.read_bytes(), image.name
            shutil.copy2(image, image_dir / image.name)
            shutil.copy2(label, label_dir / label.name)
    (dataset / 'dataset.yaml').write_text('train: images/train\nval: images/val\ntest: images/test\nnames:\n  0: fish\n', encoding='ascii')
    for name in ('manifest.yaml',):
        shutil.copy2(augmented.parent / name, dataset / ('source-' + name))
    samples = output / 'samples'
    samples.mkdir(exist_ok=True)
    video = SOURCE.parent / '\u5c4f\u5e55\u5f55\u5236 2026-10-18 115851.mp4'
    if video.is_file(): shutil.copy2(video, samples / 'fish-reference.mp4')

    firmware = output / 'firmware'
    firmware.mkdir(exist_ok=True)
    for old, new in [('robotfish.bin', 'RobotFish-ESP32S3-2.7.7-steering-priority.bin'),
                     ('robotfish.elf', 'RobotFish-ESP32S3-2.7.7-steering-priority.elf'),
                     ('bootloader/bootloader.bin', 'bootloader.bin'),
                     ('partition_table/partition-table.bin', 'partition-table.bin'),
                     ('ota_data_initial.bin', 'ota_data_initial.bin')]:
        shutil.copy2(SOURCE / 'build/firmware' / old, firmware / new)
    shutil.copy2(SOURCE / 'tools/portable.py', output / 'portable.py')
    shutil.copy2(SOURCE / 'tools/portable_training.py', output / 'portable_training.py')
    shutil.copy2(SOURCE / 'tools/test_portable_delivery.py', output / 'verify_delivery.py')
    shutil.copy2(SOURCE / 'tools/portable_readme.txt', output / 'README.txt')
    shutil.copy2(SOURCE / 'tools/portable_environment.json', output / 'environment-versions.json')
    for name, action in {'Editor': 'editor'}.items():
        (output / (name + '.cmd')).write_text('@echo off\nsetlocal\nset "PYTHONHOME="\nset "PYTHONPATH="\n"%~dp0runtime\\python.exe" -I -B "%~dp0portable.py" '
            + action + ' %*\nif errorlevel 1 pause\n', encoding='ascii')
    for filename, action in (('start.cmd', 'start'), ('stop.cmd', 'stop')):
        (project / 'upper' / filename).write_text('@echo off\nsetlocal\nset "PYTHONHOME="\nset "PYTHONPATH="\n"%~dp0..\\..\\runtime\\python.exe" -I -B "%~dp0..\\..\\portable.py" '
            + action + ' %*\nif errorlevel 1 pause\n', encoding='ascii')
    (project / 'build.ps1').write_text(
        "$ErrorActionPreference = 'Stop'\n& (Join-Path $PSScriptRoot '../runtime/python.exe') -I -B (Join-Path $PSScriptRoot '../portable.py') build\nexit $LASTEXITCODE\n", encoding='ascii')
    cache = (SOURCE / 'build/firmware/CMakeCache.txt').read_text(encoding='utf-8')
    defines = dict(re.findall(r'(?m)^(ROBOTFISH_[A-Z_]+):BOOL=(ON|OFF)$', cache))
    manifest = {'format': 1, 'platform': 'Windows 10/11 x64', 'firmware': '2.7.7-steering-priority',
                'entry': '.vscode/launch.json',
                'firmware_tests_removed': True,
                'python': '3.13.14', 'torch': '2.11.0+cpu', 'ultralytics': '8.4.118',
                'cmake_defines': defines,
                'idf': '5.3.2', 'excluded': ['project/docs', 'original build caches', 'credentials',
                     'local camera calibration', 'personal experiment logs', 'Git account configuration'],
                'source_files': {}, 'dataset_counts': {s: len(list((dataset / 'images' / s).glob('*'))) for s in ('train', 'val', 'test')}}
    manifest['offline_environment'] = create_environment_backup(output)
    manifest['environment_versions'] = 'environment-versions.json'
    for p in sorted(project.rglob('*')):
        if p.is_file(): manifest['source_files'][p.relative_to(project).as_posix()] = hashlib.sha256(p.read_bytes()).hexdigest()
    (output / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    assert not (project / 'docs').exists()
    print('ASSEMBLED', output, flush=True)


if __name__ == '__main__':
    main()
