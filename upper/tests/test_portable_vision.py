"""Portable GPU selection without loading CUDA or opening any hardware."""
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
import portable


class PortableVisionTests(unittest.TestCase):
    def test_auto_gpu_unhides_cuda_even_from_an_old_cpu_editor_terminal(self):
        with tempfile.TemporaryDirectory() as folder:
            python = Path(folder) / 'python.exe'
            python.touch()
            old = {'ROBOFISH_VISION_PYTHON': str(portable.PYTHON), 'CUDA_VISIBLE_DEVICES': '-1'}
            with patch.dict(os.environ, old, clear=True), \
                    patch.object(portable, 'LOCAL_VISION_PYTHON', python), \
                    patch.object(portable, 'STATE', Path(folder) / 'state'):
                env = portable.environment()
                self.assertEqual(env['ROBOFISH_VISION_PYTHON'], str(python))
                self.assertEqual(env['ROBOFISH_VISION_DEVICE'], 'auto')
                self.assertNotIn('CUDA_VISIBLE_DEVICES', env)
                cpu = portable.environment(vision_device='cpu')
                self.assertEqual(cpu['ROBOFISH_VISION_PYTHON'], str(portable.PYTHON))
                self.assertEqual(cpu['CUDA_VISIBLE_DEVICES'], '-1')

    def test_absent_local_gpu_falls_back_but_explicit_configuration_must_exist(self):
        with tempfile.TemporaryDirectory() as folder, patch.dict(os.environ, {}, clear=True), \
                patch.object(portable, 'STATE', Path(folder) / 'state'), \
                patch.object(portable, 'LOCAL_VISION_PYTHON', Path(folder) / 'missing.exe'):
            env = portable.environment()
            self.assertEqual(env['ROBOFISH_VISION_PYTHON'], str(portable.PYTHON))
            self.assertEqual(env['CUDA_VISIBLE_DEVICES'], '-1')
            with self.assertRaises(RuntimeError):
                portable.environment(vision_device='cuda')
            with patch.dict(os.environ, {'ROBOFISH_VISION_PYTHON': str(Path(folder) / 'custom.exe')}):
                with self.assertRaises(RuntimeError):
                    portable.environment()
            with self.assertRaises(ValueError):
                portable.environment(vision_device='unknown')

    def test_gpu_browser_allows_acceleration_and_cpu_keeps_portable_compatibility(self):
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(portable, 'STATE', Path(folder) / 'state'), \
                patch.object(portable, 'execute') as execute:
            edge = Path(folder) / 'Microsoft/Edge/Application/msedge.exe'
            edge.parent.mkdir(parents=True)
            edge.touch()
            for python in (portable.PYTHON, Path(folder) / 'gpu-python.exe'):
                env = {'PROGRAMFILES': folder, 'ROBOFISH_HOST_PORT': '8767',
                       'ROBOFISH_VISION_PYTHON': str(python)}
                portable.host('start', [], env)
                args = execute.call_args.args[1]['ROBOFISH_BROWSER_ARGS']
                self.assertEqual('--disable-gpu' in args, python == portable.PYTHON)


if __name__ == '__main__':
    unittest.main()
