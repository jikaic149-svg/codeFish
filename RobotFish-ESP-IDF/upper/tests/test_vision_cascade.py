"""Yaw/LOS/PID and workstation integration without cameras or device commands."""
import math
import importlib.util
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

if importlib.util.find_spec('cv2') is None:
    raise unittest.SkipTest('Run cascade tests with the installed YOLO/OpenCV interpreter')
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'upper'),str(ROOT/'vision')]
import device_client
from path_guidance import PathGuidance
from station import Workstation
from vision_speed import VisualSpeedLoop
from yaw_heading import CalibratedYaw,wrap_radians
from yaw_control import YawLoop
from yolo_tracker import YoloBodyTracker


def sample(degrees,timestamp=10.):
    return {'valid':True,'yaw_cw_deg':degrees,'sample_time':timestamp}


class HeadingTests(unittest.TestCase):
    def test_head_calibration_zeros_yaw_and_preserves_world_direction(self):
        heading=CalibratedYaw()
        result=heading.calibrate(math.pi/2,sample(40),10.)
        self.assertAlmostEqual(result['yaw_deg'],0.)
        np.testing.assert_allclose(result['unit_vector'],[0,-1],atol=1e-8)
        result=heading.update(sample(10,10.1),10.1)
        self.assertAlmostEqual(result['yaw_deg'],30.)
        self.assertAlmostEqual(result['world_yaw_rad'],math.radians(120))
        self.assertAlmostEqual(heading.relative_target_deg(math.pi/2),0.)

    def test_wrap_stale_and_recalibration(self):
        heading=CalibratedYaw()
        heading.calibrate(0,sample(179),10.)
        self.assertAlmostEqual(heading.update(sample(-179,10.1),10.1)['yaw_deg'],-2.)
        self.assertFalse(heading.update(sample(-179,10.1),11.)['usable'])
        heading.calibrate(-math.pi/3,sample(-25,11),11.)
        self.assertAlmostEqual(heading.update(sample(-25,11),11.)['yaw_deg'],0.)
        with self.assertRaises(ValueError):heading.calibrate(0,{'valid':False},11.)
        self.assertFalse(heading.update(sample(-25,10.),11.)['usable'])

    def test_cached_imu_frame_does_not_change_pid_observation_time(self):
        heading=CalibratedYaw()
        heading.calibrate(0,dict(sample(20),source_id=500),10.)
        result=heading.update(dict(sample(20,10.00001),source_id=500),10.1)
        self.assertEqual(result['sample_time'],10.)
        result=heading.update(dict(sample(10,10.2),source_id=600),10.2)
        self.assertEqual(result['sample_time'],10.2)
        self.assertAlmostEqual(result['yaw_deg'],10.)


class ControlTests(unittest.TestCase):
    def test_angle_sign_shortest_turn_deduplication_and_saturation(self):
        pid=YawLoop(kp=50,ki=5,kd=8)
        self.assertGreater(pid.update(math.radians(10),0,10.,now=10.),0)
        a=pid.update(math.radians(10),0,10.1,now=10.1);integral=pid.integral
        self.assertEqual(a,pid.update(math.radians(10),0,10.1,now=10.1))
        self.assertEqual(pid.integral,integral)
        pid.reset()
        self.assertGreater(pid.update(math.radians(-179),math.radians(179),10.,now=10.),0)
        self.assertEqual(pid.update(3,0,10.1,now=10.1),100)
        self.assertEqual(pid.update(0,None,10.2,now=10.2),0)
        self.assertFalse(pid.status['feedback_valid'])

    def test_los_uses_imu_while_stationary_and_removes_course_fallback(self):
        guide=PathGuidance(min_speed_target_mps=.05)
        guide.start([[0,0],[2,0]],[0,.15],10.,[1,0])
        result=guide.update([0,.15],10.1,body_heading=[1,0],speed_mps=.1)
        self.assertGreater(result['heading_error_deg'],0) # fish below line: turn left/up
        self.assertEqual(result['heading_source'],'IMU_YAW')
        result=guide.update([0,.15],10.2,body_heading=[0,-1],speed_mps=.1)
        self.assertLess(result['heading_error_deg'],0)
        with self.assertRaises(RuntimeError):guide.update([0,.15],10.3)

    def test_approach_decelerates_without_stopping_outside_arrival(self):
        guide=PathGuidance(min_speed_target_mps=.05)
        guide.start([[0,0],[1,0]],[0,0],10.,[1,0])
        for x in np.arange(.1,.81,.1):
            result=guide.update([x,0],10.+x,body_heading=[1,0],speed_mps=.2)
        result=guide.update([.72,0],11.,body_heading=[1,0],speed_mps=.2)
        self.assertFalse(result['arrived'])
        self.assertFalse(result['brake_request'])
        self.assertGreater(result['speed_target_mps'],0)
        self.assertLess(result['speed_target_mps'],guide.cruise_speed_mps)

    def test_closed_loop_straight_and_curve_with_parallel_inner_loops(self):
        paths=[np.array([[0,0],[2,0]],float),
               np.column_stack((.65*np.sin(np.linspace(0,2.4,100)),.65*(np.cos(np.linspace(0,2.4,100))-1)))]
        for path in paths:
            guide=PathGuidance(min_speed_target_mps=.05,arrival_radius_m=.12)
            position=path[0]+[0,.08];yaw=0.;speed=0.;rate=0.;dt=.05
            speed_loop=VisualSpeedLoop();angle_loop=YawLoop()
            guide.start(path,position,100.,[1,0])
            errors=[]
            for tick in range(1,1600):
                now=100.+tick*dt
                result=guide.update(position,now,body_heading=[math.cos(yaw),-math.sin(yaw)],speed_mps=speed)
                if result['arrived']:break
                power=speed_loop.update(result['speed_target_mps'],speed,now,now=now)
                steering=angle_loop.update(result['psi_ref_rad'],yaw,now,now=now,feedforward=result['curve_feedforward']*100)
                speed+=dt/.35*(power/100*.36-speed)
                rate+=dt/.25*(steering/100*1.8-rate)
                yaw=wrap_radians(yaw+rate*dt)
                position+=speed*dt*np.array([math.cos(yaw),-math.sin(yaw)])
                errors.append(result['cross_track_m'])
            self.assertTrue(result['arrived'],(position,guide.progress_m))
            self.assertLess(max(errors),.30)


class WorkstationTests(unittest.TestCase):
    def setup_client(self):
        station=Workstation();station.host='192.168.4.1'
        client=SimpleNamespace(is_robotfish=lambda host:True,
            control_binding=lambda host,radio_session=None:'connected-session',
            motion_control=lambda *args,**kwargs:dict(accepted=True),
            robot=lambda host:dict(motion={'mode':1},attitude_control={'enabled':False}))
        return station,client

    def test_prepare_uses_existing_session_and_manual_takeover_revokes_vision(self):
        station,client=self.setup_client();calls=[]
        view=dict(motion={'mode':3},attitude_control={'enabled':True})
        client.robot=lambda host:view
        def command(*args,**kwargs):
            calls.append((args,kwargs))
            if args[1]=='attitude_off':view['attitude_control']['enabled']=False
            if args[1]=='mode':view['motion']['mode']=kwargs['mode']
            return dict(accepted=True)
        client.motion_control=command
        with patch('station.device_client.CLIENT',client):
            result=station.prepare_vision('vision-test',1)
            self.assertTrue(result['ready'])
            self.assertEqual([args[1] for args,_ in calls],['attitude_off','mode'])
            self.assertTrue(all(args[2]=='connected-session' for args,_ in calls))
            self.assertEqual(calls[1][1]['mode'],1)
            station.target(dict(surge=10,yaw=-5,heave=0),'vision-test',2,'vision')
            station.target(dict(surge=0,yaw=0,heave=0),'manual-page',1,'auto')
            with self.assertRaises(device_client.DeviceError):
                station.target(dict(surge=10,yaw=0,heave=0),'vision-test',3,'vision')

    def test_queued_yaw_setting_is_not_overwritten_by_mode(self):
        station,client=self.setup_client();calls=[]
        client.robot=lambda host:dict(motion={'mode':3},attitude_control={'enabled':True})
        client.motion_control=lambda *args,**kwargs:calls.append(args[1]) or dict(queued=True)
        with patch('station.device_client.CLIENT',client):
            self.assertFalse(station.prepare_vision('vision-test',1)['ready'])
        self.assertEqual(calls,['attitude_off'])
        self.assertFalse(station.vision_sessions)

    def test_yaw_feedback_exposes_mapped_imu_with_sample_age(self):
        station=Workstation()
        with patch.object(station,'read',return_value=dict(online=True,checked_at=10.,
                attitude_control=dict(body_yaw_deg=25.,age_ms=100.,heading_valid=True,body_valid=True))):
            result=station.vision_telemetry()
        self.assertTrue(result['valid']);self.assertAlmostEqual(result['sample_time'],9.9)
        self.assertEqual(result['yaw_cw_deg'],25.)


class DetectorTests(unittest.TestCase):
    def test_yolo_identity_measured_gap_and_duplicate_frame(self):
        class Array:
            def __init__(self,value):self.value=value
            def cpu(self):return self
            def tolist(self):return self.value
        class Model:
            task='detect';names={0:'fish'}
            def __init__(self,path):self.calls=0
            def predict(self,*args,**kwargs):
                self.calls+=1
                return [SimpleNamespace(boxes=SimpleNamespace(xyxy=Array([[20,20,100,70]] if self.calls==1 else []),conf=Array([.8] if self.calls==1 else [])))]
        tracker=YoloBodyTracker('fixture.pt',model_factory=Model)
        image=np.random.default_rng(0).integers(0,255,(120,160,3),dtype=np.uint8)
        observation=tracker.update(image,10.)
        self.assertTrue(observation.valid and observation.yolo_confirmed)
        self.assertIs(tracker.update(image,10.),observation);self.assertEqual(tracker.model.calls,1)
        with patch.object(tracker,'_track_flow',return_value=True):
            observation=tracker.update(image,10.1)
        self.assertTrue(observation.valid);self.assertFalse(observation.yolo_confirmed)
        self.assertEqual(observation.metrics['tracking_mode'],'MEASURED_FLOW')
        with patch.object(tracker,'_track_flow',return_value=False):
            self.assertFalse(tracker.update(image,11.).valid)
        self.assertFalse(tracker.update(image,9.).valid)


class RuntimeBridgeTests(unittest.TestCase):
    def test_real_parallel_pids_reach_existing_sender_with_one_turn_sign_conversion(self):
        import cv2
        import vision_app
        import web_runtime
        calls=[]
        bridge=SimpleNamespace(start=lambda:None,server=SimpleNamespace(shutdown=lambda:None))
        def parent(body):
            calls.append(body.copy())
            return {'ready':True,'mode':1,'accepted':True}
        bridge.parent=parent
        def exercise():
            comm=vision_app.RoboFishComm()
            try:
                self.assertTrue(comm.ensure_bionic_mode())
                now=time.time()
                command=comm.process_tracking_error(0.,0.,1.,speed_mps=.1,speed_target_mps=.2,
                    observed_at=now,feedback_valid=True,target_yaw_rad=math.pi/8,body_yaw_rad=0.,yaw_observed_at=now)
                self.assertGreater(command['power_percent'],0)
                self.assertGreater(command['steering_percent'],0) # PID left positive
                deadline=time.monotonic()+.3
                while not any(c.get('task')=='target' for c in calls) and time.monotonic()<deadline:time.sleep(.005)
                axes=next(c['axes'] for c in calls if c.get('task')=='target')
                self.assertGreater(axes['surge'],0)
                self.assertLess(axes['yaw'],0) # hardware interface right positive
                self.assertEqual(axes['heave'],0)
            finally:comm.close()
        cv_names=['namedWindow','resizeWindow','moveWindow','setMouseCallback','imshow','destroyAllWindows','getWindowProperty','waitKey']
        with patch.object(web_runtime,'WebBridge',return_value=bridge),patch.object(vision_app,'main',exercise), \
             patch.object(vision_app,'RoboFishComm',vision_app.RoboFishComm), \
             patch.dict(os.environ,{'ROBOFISH_VISION_OUTPUT':str(ROOT/'build/host-vision')}), \
             patch.multiple(cv2,**{name:getattr(cv2,name) for name in cv_names}):
            web_runtime.run('0',str(ROOT/'vision/fish_body_yolo/best.pt'))


if __name__=='__main__':unittest.main()
