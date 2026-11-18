"""Observer-to-sender continuity using synthetic frames; no device or camera."""
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision'))
from observer_runtime import ObserverSession
from pool_perspective import PoolPerspective
from web_runtime import WebBridge,WorkstationComm


class ObserverContinuityTests(unittest.TestCase):
    def test_brief_gap_preserves_path_and_epoch_change_stops(self):
        clock=[1000.]
        with patch.dict(os.environ,{'ROBOFISH_PARENT_TOKEN':'fixture','ROBOFISH_VISION_TOKEN':'fixture'}), \
             patch('observer_runtime.time.time',side_effect=lambda:clock[0]), \
             patch('web_runtime.time.monotonic',side_effect=lambda:clock[0]):
            bridge=WebBridge(observer=True)
            bridge.parent=Mock(return_value={'accepted':True})
            tracker=SimpleNamespace(device='cpu',last_speed={},update=lambda *_:SimpleNamespace(
                position=(500,250),confidence=.99,metrics={'body_bbox':(400,150,600,350),'track_id':1}))
            folder=ROOT/'build/host-continuity-check'
            session=ObserverSession('0','fixture.pt',bridge,detector=tracker,output=folder,calibration=folder/'unused.json')
            frame=np.full((500,1000,3),200,np.uint8)
            cv2.ellipse(frame,(500,250),(80,23),0,0,360,(20,20,20),-1)
            for index in range(6):
                clock[0]=1000.+index*.04
                session.accept_frame(index,clock[0],frame)
                if index==0:session.field=PoolPerspective(3.,1.6).calibrate([[0,0],[999,0],[999,499],[0,499]],frame.shape)
            session.head={'imu_zero_cw_deg':0.,'visual_zero_world_deg':0.,'source_id':1000,'imu_aligned':True}
            session.comm=WorkstationComm(bridge,start_sender=False)
            session.comm.authorized=True
            session.automatic=session.tracking=bridge.control_enabled=True
            session.guide.start([[1.5,.8],[2.7,.8]],session.position(),session.track_time,[1,0])

            def receive(valid,uptime):
                clock[0]+=.04
                bridge.yaw_sample={'valid':valid,'yaw_cw_deg':0.,'sample_time':clock[0]-.02,'source_id':uptime}
                session.accept_frame(session.index+1,clock[0],frame)
                session.comm.send_latest()

            receive(True,1100)
            self.assertEqual(bridge.parent.call_args.args[0]['task'],'heading')
            self.assertGreater(session.comm.speed_status['observation_age_ms'],290)
            self.assertLess(session.comm.speed_status['receipt_age_ms'],1)
            progress=session.guide.progress_m
            stamp=session.guidance['position_sample_time']
            with patch.object(session,'position',return_value=session.position()+[.32,.12]):
                receive(True,1120)
            self.assertFalse(session.guidance['position_accepted'])
            self.assertEqual(session.guidance['position_sample_time'],stamp)
            self.assertEqual(session.velocity._history[-1][0],stamp)
            self.assertEqual(session.comm.speed_status['receipt_age_ms'],40)
            self.assertEqual(session.guide.progress_m,progress)
            receive(False,1140)
            self.assertTrue(session.automatic and session.tracking and session.comm.authorized)
            self.assertTrue(session.comm.command_status['paused'])
            self.assertEqual(bridge.parent.call_args.args[0]['task'],'stop')
            receive(True,1180)
            self.assertTrue(session.automatic and session.comm.authorized)
            self.assertFalse(session.comm.command_status['paused'])
            self.assertGreaterEqual(session.guide.progress_m,progress)
            self.assertEqual(bridge.parent.call_args.args[0]['task'],'heading')
            self.assertEqual(session.comm.pause_count,1)
            receive(True,5)
            self.assertFalse(session.automatic or session.tracking or session.comm.authorized)
            self.assertEqual(session.comm.last_stop_reason,'imu_epoch_changed')


if __name__=='__main__':unittest.main()
