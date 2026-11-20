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
sys.path[:0]=[str(ROOT/'upper/src'),str(ROOT/'vision/src')]
import device_client
from path_guidance import PathGuidance
from station import Workstation
from vision_speed import VisualSpeedLoop
from yaw_heading import CalibratedYaw,wrap_radians
from yolo_tracker import YoloBodyTracker


def sample(degrees,timestamp=10.):
    return {'valid':True,'yaw_cw_deg':degrees,'sample_time':timestamp}


class HeadingTests(unittest.TestCase):
    def test_world_targets_convert_once_to_native_clockwise_yaw(self):
        heading=CalibratedYaw();heading.calibrate(math.pi/2,sample(40),10.)
        self.assertAlmostEqual(heading.target_cw_deg(math.radians(120)),10.)
        self.assertAlmostEqual(heading.target_cw_deg(math.pi/2),40.)
        heading.calibrate(math.radians(179),sample(-179),10.)
        self.assertAlmostEqual(heading.target_cw_deg(math.radians(-179)),179.)

    def test_imu_reboot_invalidates_the_old_zero_but_uptime_wrap_does_not(self):
        heading=CalibratedYaw();heading.calibrate(0,dict(sample(40),source_id=10000),10.)
        result=heading.update(dict(sample(0,10.1),source_id=100),10.1)
        self.assertFalse(result['usable'] or result['initialized'])
        self.assertEqual(result['source'],'IMU_RESTARTED')
        heading.calibrate(0,dict(sample(40),source_id=0xfffffff0),10.)
        self.assertTrue(heading.update(dict(sample(30,10.1),source_id=10),10.1)['usable'])
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



class WorkstationTests(unittest.TestCase):
    def setup_client(self):
        station=Workstation();station.host='192.168.4.1'
        client=SimpleNamespace(is_robotfish=lambda host:True,
            control_binding=lambda host,radio_session=None:'connected-session',
            motion_control=lambda *args,**kwargs:dict(accepted=True),
            robot=lambda host:dict(motion={'mode':3},attitude_control={'enabled':False,'heading_target_supported':True,'heading_tail_guidance_supported':True,'heading_allocation_profile':'hybrid_gait_tail_bias','heading_tail_amplitude_gain':1.5,'heading_bias_priority_supported':True}))
        return station,client

    def test_prepare_uses_existing_session_and_manual_takeover_revokes_vision(self):
        station,client=self.setup_client();calls=[]
        view=dict(motion={'mode':2},attitude_control={'enabled':True,'heading_target_supported':True,'heading_tail_guidance_supported':True,'heading_allocation_profile':'hybrid_gait_tail_bias','heading_tail_amplitude_gain':1.5,'heading_bias_priority_supported':True})
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
            self.assertEqual([args[1] for args,_ in calls],['stop','mode'])
            self.assertTrue(all(args[2]=='connected-session' for args,_ in calls))
            self.assertTrue(calls[0][1]['via_wifi']);self.assertEqual(calls[1][1]['mode'],3)
            station.vision_command('heading_target',dict(heading_cw_deg=-5,surge_pct=10),'vision-test',2)
            station.target(dict(surge=0,yaw=0,heave=0),'manual-page',1,'auto')
            with self.assertRaises(device_client.DeviceError):
                station.vision_command('heading_target',dict(heading_cw_deg=-5,surge_pct=10),'vision-test',3)

    def test_prepare_waits_for_mode_readback_and_preserves_onboard_angle_loop(self):
        station,client=self.setup_client();calls=[]
        client.robot=lambda host:dict(motion={'mode':2},attitude_control={'enabled':True,'heading_target_supported':True,'heading_tail_guidance_supported':True,'heading_allocation_profile':'hybrid_gait_tail_bias','heading_tail_amplitude_gain':1.5,'heading_bias_priority_supported':True})
        client.motion_control=lambda *args,**kwargs:calls.append(args[1]) or dict(queued=True)
        with patch('station.device_client.CLIENT',client):
            self.assertFalse(station.prepare_vision('vision-test',1)['ready'])
        self.assertEqual(calls,['stop','mode'])
        self.assertFalse(station.vision_sessions)

    def test_pid_tuning_preserves_mission_and_manual_mode_stops_the_native_goal(self):
        station,client=self.setup_client();calls=[]
        client.motion_control=lambda *args,**kwargs:calls.append(args[1]) or dict(accepted=True)
        with patch('station.device_client.CLIENT',client):
            station.prepare_vision('vision-test',1)
            station.configure_control('yaw_pid',dict(kp=5,ki=.02,kd=.1),'manual',1)
            self.assertIn('vision-test',station.vision_sessions)
            station.vision_command('heading_target',dict(heading_cw_deg=20,surge_pct=30),'vision-test',2)
            station.configure_control('mode',2,'manual',2)
            self.assertEqual(calls,['stop','yaw_pid','heading_target','stop','mode'])
            self.assertFalse(station.vision_sessions)

    def test_old_firmware_cannot_be_prepared_or_switched_to_a_new_mode(self):
        station,client=self.setup_client();calls=[]
        client.robot=lambda host:dict(motion={'mode':2},attitude_control={'enabled':True})
        client.motion_control=lambda *a,**kw:calls.append(a)
        with patch('station.device_client.CLIENT',client),self.assertRaises(device_client.DeviceError):
            station.prepare_vision('vision-test',1)
        self.assertFalse(calls or station.vision_sessions)

    def test_yaw_feedback_exposes_mapped_imu_with_sample_age(self):
        station=Workstation()
        with patch.object(station,'read',return_value=dict(online=True,checked_at=10.,
                attitude_control=dict(body_yaw_deg=25.,age_ms=100.,heading_valid=True,body_valid=True))):
            result=station.vision_telemetry()
        self.assertTrue(result['valid']);self.assertAlmostEqual(result['sample_time'],9.9)
        self.assertEqual(result['yaw_cw_deg'],25.)

    def test_measured_speed_cannot_start_with_old_heading_allocation(self):
        station,client=self.setup_client();calls=[]
        client.robot=lambda host:dict(motion={'mode':3},attitude_control={'heading_target_supported':True,
            'heading_tail_guidance_supported':True,'heading_allocation_profile':'prop_surge_tail_bias'})
        client.motion_control=lambda *args,**kwargs:calls.append(args)
        with patch('station.device_client.CLIENT',client),self.assertRaises(device_client.DeviceError):
            station.prepare_vision('vision-test',1)
        self.assertFalse(calls or station.vision_sessions)

    def test_yaw_alignment_uses_angle_timestamp_rather_than_older_rate_timestamp(self):
        station=Workstation()
        with patch.object(station,'read',return_value=dict(online=True,checked_at=10.,imu={'age_ms':40.},
                attitude_control=dict(body_yaw_deg=25.,age_ms=100.,heading_valid=True,body_valid=True))):
            result=station.vision_telemetry()
        self.assertTrue(result['valid']);self.assertAlmostEqual(result['sample_time'],9.96)


class DetectorTests(unittest.TestCase):
    def test_local_flow_features_still_measure_motion_in_global_image_coordinates(self):
        import cv2
        from white_black_tracker import WhiteBlackTracker
        tracker=WhiteBlackTracker();gray=np.zeros((776,1384),np.uint8)
        gray[300:350,600:700]=np.random.default_rng(42).integers(0,255,(50,100),dtype=np.uint8)
        with patch('white_black_tracker.cv2.goodFeaturesToTrack',wraps=cv2.goodFeaturesToTrack) as features:
            tracker._seed_flow(gray,(600,300,700,350))
            self.assertLess(features.call_args.args[0].size,150*80)
        self.assertGreater(len(tracker._flow_points),4)
        self.assertTrue(np.all(tracker._flow_points[:,:,0]>=600))
        self.assertTrue(np.all(tracker._flow_points[:,:,1]>=300))
        tracker._flow_gray=gray;tracker._flow_time=10.;tracker._lock_position=np.array([650.,325.])
        tracker._lock_bbox=(600,300,700,350)
        following=cv2.warpAffine(gray,np.float32([[1,0,3],[0,1,-2]]),(1384,776))
        self.assertTrue(tracker._track_flow(following,10.1))
        np.testing.assert_allclose(tracker._lock_position,[653.,323.],atol=.3)

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
    def test_sender_keeps_speed_on_pc_and_sends_angle_to_s3_without_pc_yaw_pid(self):
        import cv2
        import vision_app
        import web_runtime
        calls=[]
        bridge=SimpleNamespace(start=lambda:None,server=SimpleNamespace(shutdown=lambda:None))
        def parent(body):
            calls.append(body.copy())
            return {'ready':True,'mode':3,'yaw_owner':'S3','accepted':True}
        bridge.parent=parent
        def exercise():
            comm=vision_app.RoboFishComm()
            try:
                self.assertTrue(comm.ensure_bionic_mode())
                now=time.time()
                command=comm.process_tracking_error(0.,0.,1.,speed_mps=.1,speed_target_mps=.2,
                    observed_at=now,feedback_valid=True,target_heading_cw_deg=-22.5,yaw_observed_at=now)
                self.assertGreater(command['power_percent'],0)
                self.assertNotIn('steering_percent',command)
                deadline=time.monotonic()+.3
                while not any(c.get('task')=='heading' for c in calls) and time.monotonic()<deadline:time.sleep(.005)
                goal=next(c for c in calls if c.get('task')=='heading')
                self.assertGreater(goal['surge_pct'],0)
                self.assertEqual(goal['heading_cw_deg'],-22.5)
                self.assertNotIn('axes',goal)
                self.assertFalse(hasattr(comm,'yaw_loop'))
            finally:comm.close()
        cv_names=['namedWindow','resizeWindow','moveWindow','setMouseCallback','imshow','destroyAllWindows','getWindowProperty','waitKey']
        with patch.object(web_runtime,'WebBridge',return_value=bridge),patch.object(vision_app,'main',exercise), \
             patch.object(vision_app,'RoboFishComm',vision_app.RoboFishComm), \
             patch.dict(os.environ,{'ROBOFISH_VISION_OUTPUT':str(ROOT/'build/checks/host-vision')}), \
             patch.multiple(cv2,**{name:getattr(cv2,name) for name in cv_names}):
            web_runtime.run('0',str(ROOT/'vision/models/fish_body_yolo/best.pt'))


if __name__=='__main__':unittest.main()
