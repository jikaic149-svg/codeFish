"""Actual OpenCV file roundtrip, codec failure and recorder lifecycle checks."""
import csv
import importlib.util
import sys
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

if importlib.util.find_spec('cv2') is None:raise unittest.SkipTest('Requires the installed YOLO/OpenCV Python')
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision'))
from video_recorder import AsyncVideoRecorder


class VideoRecordingTests(unittest.TestCase):
    def test_queue_drops_do_not_shorten_the_recording_timeline(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);entered=threading.Event();resume=threading.Event()
            recorder=AsyncVideoRecorder(root/'drops.mp4',cv2.VideoWriter_fourcc(*'mp4v'),
                                       120,(64,48),root/'drops.csv',fields=['source_frame'])
            write=recorder._write
            def paused(*args,**kwargs):
                entered.set()
                if not resume.wait(2):raise RuntimeError('Test writer timed out')
                write(*args,**kwargs)
            try:
                with patch.object(recorder,'_write',side_effect=paused):
                    image=np.zeros((48,64,3),np.uint8)
                    recorder.submit(image,[0],timestamp=100.)
                    self.assertTrue(entered.wait(1))
                    for index in range(1,20):recorder.submit(image,[index],timestamp=100.+index/30)
                    self.assertGreater(recorder.dropped_frames,0)
                    resume.set();recorder.close()
            finally:
                resume.set();recorder.close()
            self.assertTrue(recorder.status()['saved'],recorder.status())
            self.assertEqual(recorder.frames,80)
            with (root/'drops.csv').open(encoding='utf-8-sig',newline='') as file:
                rows=list(csv.DictReader(file))
            self.assertEqual(len(rows),80)
            self.assertEqual([int(row['recording_frame']) for row in rows],list(range(80)))
            self.assertEqual(sum(int(row['recording_repeated']) for row in rows),recorder.repeated_frames)

    def test_invalid_or_unbounded_timestamps_never_produce_a_silent_fast_video(self):
        for timestamps in ((10.,float('nan')),(10.,9.),(10.,16.),(10.,None)):
            with self.subTest(timestamps=timestamps),tempfile.TemporaryDirectory() as directory:
                root=Path(directory)
                recorder=AsyncVideoRecorder(root/'time.mp4',cv2.VideoWriter_fourcc(*'mp4v'),
                                           120,(64,48),root/'time.csv',fields=['source_frame'])
                try:
                    for index,timestamp in enumerate(timestamps):
                        recorder.submit(np.zeros((48,64,3),np.uint8),[index],timestamp=timestamp)
                finally:recorder.close()
                self.assertFalse(recorder.status()['saved'])
                self.assertTrue(recorder.error)
                self.assertLessEqual(recorder.frames,1)

    def test_invalid_codec_is_reported_before_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            recorder=None
            try:
                with self.assertRaises(RuntimeError):
                    recorder=AsyncVideoRecorder(str(root/'bad.mp4'),cv2.VideoWriter_fourcc(*'zzzz'),
                                                20,(320,240),str(root/'bad.csv'))
            finally:
                if recorder:recorder.close()

    def test_mp4_and_csv_are_readable_after_stop(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            recorder=AsyncVideoRecorder(str(root/'output.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),
                                        20,(320,240),str(root/'output.csv'))
            for index in range(6):
                frame=np.full((240,320,3),30+index*20,dtype=np.uint8)
                self.assertTrue(recorder.submit(frame,[index]))
                frame[:]=0  # A queued frame must be independent of later rendering.
            recorder.close()
            result=recorder.status()
            self.assertFalse(result['active'])
            self.assertFalse(result['error'])
            self.assertEqual(result['frames'],6)
            self.assertFalse(recorder.submit(np.zeros((240,320,3),np.uint8),[]))
            recorder.close()  # Stop and final process cleanup may both close it.
            capture=cv2.VideoCapture(str(root/'output.mp4'))
            self.assertTrue(capture.isOpened())
            self.assertEqual(capture.get(cv2.CAP_PROP_FRAME_WIDTH),320)
            self.assertEqual(capture.get(cv2.CAP_PROP_FRAME_HEIGHT),240)
            decoded=[]
            while True:
                ok,frame=capture.read()
                if not ok:break
                decoded.append(float(frame.mean()))
            capture.release()
            self.assertEqual(len(decoded),6)
            self.assertGreater(decoded[-1],80)
            with (root/'output.csv').open(encoding='utf-8-sig',newline='') as file:
                self.assertEqual(len(list(csv.reader(file))),7)

    def test_frame_size_is_fixed_and_empty_recording_is_not_reported_saved(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            recorder=AsyncVideoRecorder(str(root/'size.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),
                                        20,(321,241),str(root/'size.csv'))
            self.assertTrue(recorder.submit(np.full((360,640,3),100,np.uint8),[1]))
            recorder.close()
            capture=cv2.VideoCapture(str(root/'size.mp4'))
            ok,frame=capture.read();capture.release()
            self.assertTrue(ok)
            self.assertEqual(frame.shape[:2],(240,320))
            empty=AsyncVideoRecorder(str(root/'empty.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),
                                     20,(320,240),str(root/'empty.csv'))
            empty.close()
            self.assertFalse(empty.status()['saved'])
            self.assertTrue(empty.status()['error'])


if __name__=='__main__':unittest.main()
