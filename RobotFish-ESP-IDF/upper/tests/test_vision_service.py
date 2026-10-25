"""Runtime availability and truthful recorder status without opening a camera."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from vision_service import VisionService, WORKSPACE


class VisionServiceTests(unittest.TestCase):
    def test_runtime_requires_and_selects_the_delivered_yolo_model(self):
        service=VisionService()
        self.assertTrue(service.configuration()['model_required'])
        self.assertEqual(service.configuration()['default_model'],'vision/fish_body_yolo/best.pt')
        self.assertEqual(service.model_path(None),WORKSPACE/'vision/fish_body_yolo/best.pt')
        self.assertEqual(service.model_path(''),service.model_path(None))
        self.assertTrue((WORKSPACE/'vision/observer_runtime.py').is_file())
        self.assertTrue(service.configuration()['observation_only'])
        self.assertNotIn('START',service.configuration()['supported_tasks'])
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

    def test_calibration_download_only_reads_the_output_directory(self):
        service=VisionService();service.perspective_test={'data_file':str(WORKSPACE/'upper/server.py')}
        with self.assertRaises(ValueError):service.test_file('json')
        with self.assertRaises(ValueError):service.test_file('other')

    def test_reader_preserves_calibration_test_file_after_exit(self):
        service=VisionService();saved={'image_file':'pool.png','data_file':'pool.json'}
        service._reader(SimpleNamespace(stdout=['ROBOFISH_PERSPECTIVE_TEST '+json.dumps(saved)]))
        self.assertEqual(service.state()['perspective_test'],saved)

    def test_samples_export_requires_a_complete_file_inside_the_output_directory(self):
        service=VisionService()
        with tempfile.TemporaryDirectory() as folder:
            service.output=Path(folder);path=service.output/'heading.csv';path.write_text('frame,x_m\n1,2\n',encoding='utf-8-sig')
            saved={'active':False,'saved':True,'rows':1,'file':str(path),'error':''}
            service._reader(SimpleNamespace(stdout=['ROBOFISH_VISION_SAMPLES '+json.dumps(saved)]))
            self.assertEqual(service.export_samples('csv')[0],path.read_bytes())
            service.samples['active']=True
            with self.assertRaises(ValueError):service.export_samples('csv')
            service.samples.update(active=False,file=str(WORKSPACE/'upper/server.py'))
            with self.assertRaises(ValueError):service.export_samples('csv')
            service.samples.update(active=True,saved=False)
            status=service.state()['samples']
            self.assertFalse(status['active'] or status['saved']);self.assertTrue(status['error'])


if __name__=='__main__':unittest.main()
