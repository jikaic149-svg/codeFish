"""Check conclusions and offline repair without compiling or contacting hardware."""
from contextlib import redirect_stdout
import hashlib
from http.server import ThreadingHTTPServer
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zipfile

spec = importlib.util.spec_from_file_location('portable_actions', Path(__file__).with_name('portable.py'))
portable = importlib.util.module_from_spec(spec)
spec.loader.exec_module(portable)
fixture_spec = importlib.util.spec_from_file_location('ota_fixture',
    Path(__file__).resolve().parents[1] / 'upper/tests/test_robotfish.py')
fixture = importlib.util.module_from_spec(fixture_spec)
fixture_spec.loader.exec_module(fixture)


class PortableActionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='portable-action-test-', dir=Path(__file__).resolve().parent)
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.state = self.root / 'state'
        self.build = self.state / 'build/firmware'
        self.build.mkdir(parents=True)
        for name, value in (('ROOT', self.root), ('STATE', self.state), ('BUILD', self.build)):
            scope = patch.object(portable, name, value)
            scope.start(); self.addCleanup(scope.stop)
        self.env = patch.object(portable, 'environment', return_value={})
        self.env.start(); self.addCleanup(self.env.stop)
        (self.root / 'manifest.json').write_text('{"cmake_defines":{}}')

    def backup(self, files):
        archive = self.root / 'offline/environment.zip'
        archive.parent.mkdir(exist_ok=True)
        with zipfile.ZipFile(archive, 'w') as output:
            for name, data in files.items():output.writestr(name, data)
        manifest = {'offline_environment': {'path': 'offline/environment.zip',
                    'sha256': hashlib.sha256(archive.read_bytes()).hexdigest()}}
        (self.root / 'manifest.json').write_text(json.dumps(manifest))
        return archive

    def test_repair_completes_missing_files_without_overwriting_existing_data(self):
        self.backup({'runtime/Lib/missing.py': 'restored', 'runtime/Lib/kept.py': 'original',
                     'idf-env/Lib/site-packages/dependency.py': 'idf'})
        kept = self.root / 'runtime/Lib/kept.py'
        kept.parent.mkdir(parents=True); kept.write_text('locally updated')
        restored = portable.repair_environment()
        self.assertEqual(len(restored), 2)
        self.assertEqual(kept.read_text(), 'locally updated')
        self.assertEqual((kept.parent / 'missing.py').read_text(), 'restored')

    def test_repair_rejects_corrupt_backup_before_writing(self):
        archive = self.backup({'runtime/Lib/missing.py': 'data'})
        with archive.open('ab') as stream:stream.write(b'changed')
        with self.assertRaisesRegex(RuntimeError, '校验失败'):portable.repair_environment()
        self.assertFalse((self.root / 'runtime').exists())

    def test_repair_rejects_paths_outside_the_package(self):
        self.backup({'runtime/Lib/missing.py': 'data', '../outside.py': 'invalid'})
        with self.assertRaisesRegex(RuntimeError, '越界'):portable.repair_environment()
        self.assertFalse((self.root / 'runtime').exists())

    def test_build_conclusion_uses_actual_version_and_image(self):
        for relative, data in {'robotfish.bin': b'new-image', 'robotfish.elf': b'elf',
                'robotfish.map': b'map', 'bootloader/bootloader.bin': b'boot',
                'partition_table/partition-table.bin': b'part', 'ota_data_initial.bin': b'ota'}.items():
            target = self.build / relative; target.parent.mkdir(parents=True, exist_ok=True); target.write_bytes(data)
        (self.build / 'project_description.json').write_text('{"project_version":"9.1.2-test"}')
        with patch.object(portable, 'execute') as command:
            result = portable.firmware_command('build', [])
        self.assertEqual(result['status'], 'success')
        self.assertEqual(result['version'], '9.1.2-test')
        self.assertEqual(result['sha256'], hashlib.sha256(b'new-image').hexdigest())
        self.assertEqual(Path(result['outputs'][0]).read_bytes(), b'new-image')
        command.assert_called_once()

    def test_flash_requires_all_images_to_be_verified(self):
        (self.build / 'flasher_args.json').write_text('{"flash_files":{"0x0":"boot.bin","0x10000":"app.bin"}}')
        for verified, expected in ((1, 'unconfirmed'), (2, 'success')):
            def command(*args, **kwargs):
                log = kwargs['log_path']; log.parent.mkdir(parents=True, exist_ok=True)
                log.write_text('Hash of data verified.\n' * verified)
            with patch.object(portable, 'execute', side_effect=command):
                result = portable.firmware_command('flash', ['COM7'])
            self.assertEqual(result['status'], expected)
            self.assertFalse(result['hardware_runtime_verified'])

    def test_invalid_port_never_starts_the_flash_command(self):
        with patch.object(portable, 'execute') as command:
            with self.assertRaises(RuntimeError):portable.firmware_command('flash', [''])
            command.assert_not_called()

    def test_cancelled_port_prompt_never_starts_flashing(self):
        with patch.dict(os.environ), patch.object(portable, 'initialize', return_value={}), \
                patch.object(portable, 'firmware_command') as flash, \
                patch('sys.argv', ['portable.py', 'flash']), patch.object(portable, 'select_usb_port', return_value=None):
            action, result = portable.main()
        self.assertEqual(action, 'flash'); self.assertEqual(result['status'], 'cancelled')
        flash.assert_not_called()

    def test_usb_detection_prefers_native_esp32_over_radio_adapter(self):
        ports = [SimpleNamespace(device='COM7', vid=0x10C4, description='Radio adapter'),
                 SimpleNamespace(device='COM9', vid=0x303A, description='USB JTAG/serial')]
        with patch('serial.tools.list_ports.comports', return_value=ports), patch('builtins.input') as prompt:
            self.assertEqual(portable.select_usb_port(), 'COM9')
            prompt.assert_not_called()

    def test_multiple_usb_candidates_require_selection_and_allow_cancel(self):
        ports = [SimpleNamespace(device='COM9', vid=0x303A, description='ESP32', serial_number='a'),
                 SimpleNamespace(device='COM7', vid=0x303A, description='ESP32', serial_number='b')]
        with patch('serial.tools.list_ports.comports', return_value=ports):
            with patch('builtins.input', return_value='2'):
                self.assertEqual(portable.select_usb_port(), 'COM9')
            with patch('builtins.input', return_value=''):
                self.assertIsNone(portable.select_usb_port())
            with patch('builtins.input', return_value='3'), self.assertRaises(RuntimeError):
                portable.select_usb_port()

    def test_no_usb_candidate_never_invokes_build_or_flash(self):
        with patch.dict(os.environ), patch.object(portable, 'initialize', return_value={}), \
                patch('serial.tools.list_ports.comports', return_value=[]), \
                patch('sys.argv', ['portable.py', 'flash']), patch.object(portable, 'firmware_command') as flash:
            with self.assertRaisesRegex(RuntimeError, '未检测到'):portable.main()
            flash.assert_not_called()

    def test_browser_uses_windows_uppercase_environment_and_package_profile(self):
        browser = self.root / 'programs/Microsoft/Edge/Application/msedge.exe'
        browser.parent.mkdir(parents=True); browser.touch()
        env = {'PROGRAMFILES(X86)': str(self.root / 'programs'), 'ROBOFISH_HOST_PORT': '8767'}
        with patch.object(portable, 'execute') as opened:
            portable.host('start', [], env)
        command = opened.call_args.args[0]
        self.assertEqual(command[0], 'powershell.exe')
        self.assertIn('Shell.Application', command[-1])
        self.assertIn('ShellExecute', command[-1])
        browser_env = opened.call_args.args[1]
        self.assertEqual(browser_env['ROBOFISH_BROWSER_EXE'], str(browser))
        self.assertIn('--user-data-dir=' + str(self.state / 'browser'), browser_env['ROBOFISH_BROWSER_ARGS'])
        self.assertIn('--disable-gpu', browser_env['ROBOFISH_BROWSER_ARGS'])
        self.assertIn('--disable-background-mode', browser_env['ROBOFISH_BROWSER_ARGS'])
        self.assertTrue(browser_env['ROBOFISH_BROWSER_ARGS'].endswith('http://127.0.0.1:8767/'))

    def test_browser_fallback_requests_default_browser_and_no_browser_skips_it(self):
        env = {'ROBOFISH_HOST_PORT': '8767'}
        with patch.object(portable, 'execute'), patch.object(portable.os, 'startfile') as opened:
            portable.host('start', [], env)
            opened.assert_called_once_with('http://127.0.0.1:8767/')
            portable.host('start', ['--no-browser'], env)
            opened.assert_called_once()

    def test_ota_auto_discovery_upload_and_confirmed_restart_over_loopback(self):
        fixture.Target.reset()
        server = ThreadingHTTPServer(('127.0.0.1', 0), fixture.Target)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(thread.join)
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        host = '127.0.0.1:' + str(server.server_port)
        image = self.root / 'app.bin'; image.write_bytes(fixture.image())
        built = {'outputs': [str(image)], 'sha256': hashlib.sha256(image.read_bytes()).hexdigest()}
        with patch.object(fixture.device_client.CLIENT, 'discover_robotfish', return_value=dict(fixture.HEALTH, host=host)), \
                patch.object(portable, 'firmware_command', return_value=built) as build, \
                patch.dict(os.environ, {'ROBOFISH_OTA_PASSWORD': 'test-user-input'}), \
                patch.object(fixture.robotfish.time, 'sleep'):
            result = portable.ota_upgrade([])
        build.assert_called_once_with('build', [])
        self.assertEqual(result['status'], 'success')
        self.assertEqual(result['host'], host)
        self.assertEqual(result['slot'], 'app1')
        self.assertEqual(fixture.Target.uploads, [fixture.image()])
        self.assertEqual(fixture.Target.paths, ['/get_ip', '/ota_upload', '/get_ip'])
        self.assertTrue((self.state / 'logs/ota.log').is_file())

    def test_ota_missing_device_does_not_build_or_upload(self):
        with patch.object(fixture.device_client.CLIENT, 'discover_robotfish', return_value=None), \
                patch.object(portable, 'firmware_command') as build, \
                patch.object(fixture.robotfish, 'install') as upload:
            with self.assertRaisesRegex(RuntimeError, '未发现'):portable.ota_upgrade([])
        build.assert_not_called(); upload.assert_not_called()

    def test_ota_upload_without_confirmation_cannot_report_success(self):
        image = self.root / 'app.bin'; image.write_bytes(fixture.image())
        with patch.object(fixture.device_client.CLIENT, 'discover_robotfish', return_value={'host': '192.168.4.1'}), \
                patch.object(portable, 'firmware_command', return_value={'outputs': [str(image)], 'sha256': 'test'}), \
                patch.object(fixture.robotfish, 'install', return_value={'complete': False}):
            self.assertEqual(portable.ota_upgrade([])['status'], 'unconfirmed')

    def test_result_record_keeps_failure_and_cancelled_states(self):
        for status, expected in (('failed', 1), ('cancelled', 130), ('unconfirmed', 2), ('success', 0)):
            with redirect_stdout(io.StringIO()):code = portable.conclude('flash', status, 'accurate scope')
            record = json.loads((self.state / 'results/flash.json').read_text(encoding='utf-8'))
            self.assertEqual(code, expected); self.assertEqual(record['status'], status)


if __name__ == '__main__':unittest.main()
