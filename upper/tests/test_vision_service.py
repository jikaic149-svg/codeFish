"""Runtime availability and truthful recorder status without opening a camera."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from vision_service import VisionService, WORKSPACE


class VisionServiceTests(unittest.TestCase):
    def test_opencv_runtime_does_not_require_a_model(self):
        service=VisionService()
        self.assertFalse(service.configuration()['model_required'])
        self.assertIsNone(service.model_path(None))
        self.assertIsNone(service.model_path(''))
        self.assertTrue((WORKSPACE/'vision/web_runtime.py').is_file())
        with self.assertRaises(ValueError):service.model_path('../outside.pt')

    def test_reader_preserves_final_recording_status_after_process_exit(self):
        service=VisionService()
        saved={'active':False,'saved':True,'file':'record.mp4','frames':42,'error':''}
        service._reader(SimpleNamespace(stdout=['ROBOFISH_VISION_RECORDING '+json.dumps(saved)]))
        result=service.state()
        self.assertFalse(result['running'])
        self.assertEqual(result['recording'],saved)

    def test_unexpected_exit_cannot_keep_recording_indicator_active(self):
        service=VisionService()
        service.recording={'active':True,'saved':False,'frames':2,'error':''}
        service.process=SimpleNamespace(poll=lambda:1)
        result=service.state()['recording']
        self.assertFalse(result['active'] or result['saved'])
        self.assertTrue(result['error'])


if __name__=='__main__':unittest.main()
