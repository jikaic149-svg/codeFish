"""Observation-gap recovery and control ownership, without hardware or a camera."""
from collections import deque
import importlib.util
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

if importlib.util.find_spec('cv2') is None:
    raise unittest.SkipTest('Requires the installed YOLO/OpenCV Python')
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'vision/src'))
from web_runtime import WebBridge, WorkstationComm


class ContinuityTests(unittest.TestCase):
    def setUp(self):
        self.now=10.
        self.calls=[]
        self.comm=WorkstationComm(SimpleNamespace(parent=self.parent),start_sender=False)
        self.comm.authorized=True
        self.clock=patch('web_runtime.time.monotonic',side_effect=lambda:self.now)
        self.wall=patch('web_runtime.time.time',side_effect=lambda:self.now+1000.)
        self.clock.start();self.wall.start()
        self.addCleanup(self.clock.stop);self.addCleanup(self.wall.stop)

    def parent(self,body):
        self.calls.append(body)
        return {'accepted':True}

    def fresh(self):
        return self.comm.process_tracking_error(0.,0.,1.,speed_mps=.08,speed_target_mps=.1,
            observed_at=self.now+1000.,feedback_valid=True,target_heading_cw_deg=30.,
            yaw_observed_at=self.now+1000.,now=self.now+1000.)

    def test_single_gap_stops_once_and_fresh_observation_resumes_same_session(self):
        self.fresh();self.comm.send_latest()
        session=self.calls[-1]['session']
        self.now=10.05
        self.assertTrue(self.comm.pause_tracking('imu_unavailable'))
        self.comm.send_latest()
        self.now=10.1;self.comm.send_latest()
        self.assertEqual([v['task'] for v in self.calls],['heading','stop'])
        self.assertTrue(self.comm.authorized)
        self.now=10.2;self.fresh();self.comm.send_latest()
        self.assertEqual([v['task'] for v in self.calls],['heading','stop','heading'])
        self.assertTrue(all(v['session']==session for v in self.calls))
        self.assertFalse(self.comm.command_status['paused'])
        self.assertEqual(self.comm.command_status['last_pause_reason'],'imu_unavailable')
        self.assertEqual(self.comm.command_status['pause_count'],1)

    def test_camera_latency_and_receipt_age_are_passed_separately(self):
        fields=dict(speed_mps=.08,speed_target_mps=.1,observed_at=1009.6,feedback_valid=True,
            target_heading_cw_deg=30.,yaw_observed_at=1010.,now=1010.)
        command=self.comm.process_tracking_error(0.,0.,1.,received_at=1009.9,**fields)
        self.assertIsNotNone(command)
        self.assertEqual(self.comm.speed_status['observation_age_ms'],400)
        self.assertEqual(self.comm.speed_status['receipt_age_ms'],100)
        command=self.comm.process_tracking_error(0.,0.,1.,received_at=1009.64,**fields)
        self.assertIsNone(command)
        self.assertEqual(self.comm.pause_reason,'stale_feedback')

    def test_repeated_bad_frames_do_not_extend_the_existing_deadline(self):
        self.comm.pause_tracking('imu_unavailable');self.comm.send_latest()
        self.now=10.2;self.assertTrue(self.comm.pause_tracking('imu_unavailable'))
        self.now=10.251;self.comm.send_latest()
        self.assertFalse(self.comm.authorized)
        self.assertEqual(self.comm.last_stop_reason,'imu_unavailable')
        self.assertEqual(self.comm.pause_count,1)
        self.assertIsNone(self.fresh())
        self.assertTrue(all(v['task']=='stop' for v in self.calls))

    def test_fresh_frame_after_deadline_cannot_revive_before_sender_checks(self):
        self.comm.pause_tracking('stale_imu')
        self.now=10.26
        self.assertIsNone(self.fresh())
        self.assertFalse(self.comm.authorized)
        self.assertEqual(self.comm.last_stop_reason,'stale_imu')
        self.assertEqual([v['task'] for v in self.calls],['stop'])

    def test_manual_stop_and_takeover_never_resume(self):
        self.comm.pause_tracking('imu_unavailable')
        self.comm.stop_now('user_stop')
        self.now=10.1;self.assertIsNone(self.fresh());self.comm.send_latest()
        self.assertEqual(self.comm.last_stop_reason,'user_stop')
        self.assertEqual([v['task'] for v in self.calls],['stop'])
        self.comm.authorized=True;self.calls.clear()
        error=RuntimeError('taken over');error.code='vision_session_changed'
        self.comm.bridge.parent=Mock(side_effect=error)
        self.fresh();self.comm.send_latest()
        self.assertFalse(self.comm.authorized)
        self.assertEqual(self.comm.last_stop_reason,'vision_session_changed')
        self.assertEqual(self.comm.bridge.parent.call_count,1)

    def test_repeated_native_heading_rejections_cannot_restart_pause_deadline(self):
        def unavailable(body):
            self.calls.append(body)
            if body['task']=='heading':
                error=RuntimeError('IMU unavailable');error.code='heading_unavailable';raise error
            return {'accepted':True}
        self.comm.bridge.parent=unavailable
        self.fresh();self.comm.send_latest()
        self.now=10.1;self.fresh();self.comm.send_latest()
        self.now=10.26;self.assertIsNone(self.fresh())
        self.assertFalse(self.comm.authorized)
        self.assertEqual(self.comm.last_stop_reason,'heading_unavailable')
        self.assertEqual([v['task'] for v in self.calls],['heading','heading','stop'])

    def test_ack_delay_is_visible_before_first_heading_and_clears_on_stop(self):
        self.comm.tracking_started_at=1009.
        self.now=10.1
        self.assertTrue(self.comm.command_status['ack_delayed'])
        self.assertAlmostEqual(self.comm.command_status['target_ack_age_ms'],1100.)
        self.comm.stop_now('imu_unavailable')
        self.assertFalse(self.comm.command_status['ack_delayed'])
        self.assertEqual(self.comm.last_stop_at,1010.1)

    def test_poll_transport_failure_retains_timestamp_but_sensor_invalid_replaces(self):
        bridge=WebBridge.__new__(WebBridge)
        bridge.guard=threading.Lock();bridge.yaw_history=deque(maxlen=128)
        bridge.yaw_sample={'valid':False}
        samples=[]
        bridge.closed=SimpleNamespace(is_set=lambda:len(samples)>=3,
            wait=lambda delay:samples.append(dict(bridge.yaw_sample)))
        valid={'valid':True,'yaw_cw_deg':20.,'sample_time':1009.9,'source_id':100}
        responses=[valid,OSError('local poll interrupted'),{'valid':False,'source_id':101}]
        def connection(*args,**kwargs):
            value=responses.pop(0)
            return SimpleNamespace(request=Mock(),close=Mock(),
                getresponse=Mock(side_effect=value) if isinstance(value,Exception) else
                Mock(return_value=SimpleNamespace(status=200,read=lambda n:json.dumps(value).encode())))
        with patch('web_runtime.http.client.HTTPConnection',side_effect=connection):bridge._telemetry_loop()
        self.assertEqual(samples[:2],[valid,valid])
        self.assertEqual(samples[2],{'valid':False,'source_id':101})
        self.assertEqual(len(bridge.yaw_history),1)


if __name__=='__main__':unittest.main()
