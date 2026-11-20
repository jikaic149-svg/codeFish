"""White-head metrology and control handoff. No real camera or device."""
import os
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision/src'))
from observer_runtime import ObserverSession
from pool_perspective import PoolPerspective
from web_runtime import WebBridge, WorkstationComm
from white_head_point import WhiteHeadPoint,WhiteHeadSpeed
sys.path.insert(0,str(ROOT/'tools'))
from review_head_feedback import review


class WhiteHeadFeedbackTests(unittest.TestCase):
    def test_noisy_cruise_and_speed_step(self):
        time=np.arange(240)/30
        speeds=np.where(time<4.,.08,.04)
        positions=np.cumsum(speeds)/30
        est=WhiteHeadSpeed(.8,.014,.25)
        measured=[]
        for i,t in enumerate(time):
            centre=np.array([positions[i],0.])
            # A one-frame 12 mm marker wobble, below the hard jump limit.
            v=est.update(centre+[.15+(.012 if i%21==5 else 0.),0.],centre,t,1)
            measured.append(np.nan if v is None else v)
        measured=np.asarray(measured)
        self.assertLess(np.sqrt(np.mean((measured[20:105]-.08)**2)),.003)
        self.assertLessEqual(time[np.flatnonzero((time>=4)&(measured<=.06))[0]]-4,.27)

    def test_review_fills_only_between_fresh_anchors_in_same_segment(self):
        times=[0,.03,.06,.09,.12,.15,.18,.21,.24,.50,.53,.56,.59]
        states=['MISSING','TRACKING','REJECTED','TRACKING','MISSING','TRACKING',
                'MISSING','WARMUP','TRACKING','MISSING','TRACKING','MISSING','MISSING']
        rows=[{'t':str(t),'frame':i,'head_state':s,'head_speed_epoch':str(1 if i<5 else 2),
               'speed_mps':str(t) if s=='TRACKING' else '', 'head_x_px':1,'head_y_px':2}
              for i,(t,s) in enumerate(zip(times,states))]
        review(rows)
        self.assertEqual([i for i,r in enumerate(rows) if r['review_state']=='INTERPOLATED'],[2])
        self.assertAlmostEqual(rows[2]['review_speed_mps'],.06)
        self.assertEqual(rows[2]['speed_mps'],'')  # never rewrite the control column
        gap=[dict(rows[0],frame=i,t=t,head_state=state,speed_mps=speed) for i,(t,state,speed) in
             enumerate([('.01','TRACKING','.1'),('.12','HELD','.1'),('.23','TRACKING','.2')])]
        review(gap)
        self.assertEqual(gap[1]['review_state'],'INTERPOLATED')
        self.assertAlmostEqual(gap[1]['review_speed_mps'],.15)
        self.assertEqual(gap[1]['speed_mps'],'.1')

    def test_forward_translation_and_pure_yaw(self):
        for speed,omega in ((.08,0.),(0.,6.),(.08,6.)):
            with self.subTest(speed=speed,omega=omega):
                estimator=WhiteHeadSpeed(.8,.014,.25)
                centre=np.array([1.,.5])
                for i in range(20):
                    t=i*.04
                    axis=np.array([np.cos(t*omega),np.sin(t*omega)])
                    if i:centre+=speed*.04*(axis+old)/np.linalg.norm(axis+old)
                    result=estimator.update(centre+.20*axis,centre,t,1)
                    old=axis
                self.assertAlmostEqual(result,speed,places=6)

    def test_template_tracks_a_moving_cool_cap_but_not_empty_water(self):
        tracker=WhiteHeadPoint()
        def image(offset,visible=True):
            im=np.full((160,260,3),(160,100,20),np.uint8)
            cv2.ellipse(im,(110+offset,80),(50,15),0,0,360,(20,20,20),-1)
            if visible:cv2.ellipse(im,(160+offset,80),(8,8),0,0,360,(198,197,198),-1)
            return im
        box=(55,59,172,101)
        a=tracker.measure(image(0),box,0.)
        self.assertIsNotNone(a)
        with patch.object(tracker,'_detect',return_value=None):
            b=tracker.measure(image(4),box,.04)
            np.testing.assert_allclose(np.subtract(b,a),[4,0],atol=1.)
            self.assertIsNone(tracker.measure(image(4,False),box,.08))
            self.assertIsNone(tracker.measure(image(4),box,.60))

    def test_gaps_outlier_and_identity_never_refresh_measurement_time(self):
        est=WhiteHeadSpeed(.8,.014,.25)
        def sample(t,key=1):
            centre=np.array([t*.08,0.])
            return est.update(centre+[.18,0],centre,t,key)
        for i in range(8):sample(i*.04)
        self.assertAlmostEqual(est.update([1.2,0],[1.,0],.32,1),.08)
        self.assertEqual(est.state,'HELD')
        self.assertAlmostEqual(est.last[2],.28)
        self.assertAlmostEqual(sample(.36),.08)
        self.assertAlmostEqual(est.update(None,[.04,0],.56,1),.08)
        self.assertAlmostEqual(sample(.60),.08)  # bridge distant real anchors
        self.assertIsNone(est.update(None,[.04,0],.86,1))  # held frames never extend deadline
        for i in range(8):sample(.9+i*.04)
        self.assertIsNone(est.update(None,None,1.2,1))  # lost body
        for i in range(8):sample(1.3+i*.04)
        self.assertIsNone(sample(1.60,2))  # no speed across identity change
        self.assertIsNone(sample(1.90,2))  # or across a long camera gap
        self.assertIsNone(sample(1.85,2))  # out-of-order timestamp

    def test_runtime_uses_forward_speed_bridges_short_gaps_and_revokes_long_loss(self):
        clock=[100.]
        tracker=SimpleNamespace(device='cpu',last_speed={})
        with TemporaryDirectory() as folder, \
             patch.dict(os.environ,{'ROBOFISH_PARENT_TOKEN':'fixture','ROBOFISH_VISION_TOKEN':'fixture'}), \
             patch('observer_runtime.time.time',side_effect=lambda:clock[0]), \
             patch('web_runtime.time.monotonic',side_effect=lambda:clock[0]):
            bridge=WebBridge(observer=True)
            bridge.parent=Mock(return_value={'accepted':True})
            session=ObserverSession('0','fixture.pt',bridge,detector=tracker,output=folder,calibration=Path(folder)/'unused.json')
            session.speed_source='white_head_forward'  # explicitly exercise the non-default experiment
            session.comm=WorkstationComm(bridge,start_sender=False)
            session.head={'imu_zero_cw_deg':0.,'visual_zero_world_deg':0.,'imu_aligned':True}
            def frame(i,visible):
                clock[0]=100.+i*.04
                # 1 px forward, 2 px sideways / 40 ms, isotropic .003 m/px.
                x,y=500+i,240+2*i
                img=np.full((500,1000,3),(160,100,20),np.uint8)
                if i==0:session.field=PoolPerspective(2.997,1.497).calibrate([[0,0],[999,0],[999,499],[0,499]],img.shape)
                cv2.ellipse(img,(x,y),(80,23),0,0,360,(20,20,20),-1)
                if visible:cv2.ellipse(img,(x+81,y),(11,13),0,0,360,(182,215,215),-1)
                tracker.update=lambda *_: SimpleNamespace(position=(x,y),confidence=.99,metrics={
                    'body_bbox':(x-100,y-100,x+100,y+100),'track_id':1})
                bridge.yaw_sample={'sample_time':clock[0]-.01,'yaw_cw_deg':0.,'valid':True}
                session.accept_frame(i,clock[0],img)
            session.path_px=[(500,240),(950,240)]
            frame(0,False)
            bridge.events.put({'id':1,'task':'AUTO_START'});session.actions()
            self.assertFalse(session.automatic)
            bridge.parent.assert_not_called()
            for i in range(1,9):frame(i,True)
            frame(9,False)
            self.assertEqual(session.head_speed.state,'HELD')
            bridge.parent.return_value={'accepted':True,'ready':True,'mode':3,'yaw_owner':'S3'}
            bridge.events.put({'id':2,'task':'AUTO_START'});session.actions()
            self.assertTrue(session.automatic)
            frame(10,True);session.comm.send_latest()
            self.assertAlmostEqual(session.speed_mps,.075,places=4)
            self.assertAlmostEqual(session.guidance['speed_mps'],.075,places=4)
            self.assertAlmostEqual(session.comm.speed_status['measured_mps'],.075,places=4)
            self.assertAlmostEqual(session.guidance['v_cross_mps'],.15,places=4)
            self.assertEqual(bridge.parent.call_args.args[0]['task'],'heading')
            self.assertEqual(session.guidance['heading_source'],'IMU_YAW')
            stamp=session.speed_time
            frame(11,False);session.comm.send_latest()
            self.assertIsNone(session.nose_px)
            self.assertEqual(session.head_speed.state,'HELD')
            self.assertEqual(session.speed_time,stamp)
            self.assertAlmostEqual(session.comm._udp_latest_t,stamp+session.camera_latency_s)
            self.assertEqual(session.comm.speed_loop.last_time,stamp)
            self.assertAlmostEqual(session.speed_mps,.075,places=4)
            self.assertEqual(bridge.parent.call_args.args[0]['task'],'heading')
            frame(12,True);session.comm.send_latest()
            self.assertEqual(bridge.parent.call_args.args[0]['task'],'heading')
            for i in range(13,20):
                frame(i,False);session.comm.send_latest();session.actions()
            self.assertIsNone(session.speed_mps)
            self.assertEqual(bridge.parent.call_args.args[0]['task'],'stop')
            for i in range(20,28):
                frame(i,False);session.comm.send_latest();session.actions()
            self.assertFalse(session.automatic or session.comm.authorized)
            sends=bridge.parent.call_count
            for i in range(28,35):frame(i,True);session.comm.send_latest()
            self.assertIsNotNone(session.speed_mps)
            self.assertEqual(bridge.parent.call_count,sends)


if __name__=='__main__':unittest.main()
