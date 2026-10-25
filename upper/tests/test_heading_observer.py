"""Perspective optical metrology and passive CSV recording; no camera or hardware."""
import csv
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

if importlib.util.find_spec('cv2') is None:raise unittest.SkipTest('Requires the installed YOLO/OpenCV Python')
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'upper'),str(ROOT/'vision')]
from angle_observation import OpticalBodyAxis,AngleSamples
from observer_runtime import ObserverSession
from pool_perspective import PoolPerspective,transform_points
from web_runtime import WebBridge


def fish(angle=0.,centre=(2.,1.)):
    image=np.full((500,1000,3),(160,100,20),np.uint8)
    point=tuple(round(x*250) for x in centre)
    cv2.ellipse(image,point,(80,23),-angle,0,360,(20,20,20),-1)
    return image,(point[0]-100,point[1]-100,point[0]+100,point[1]+100)


def field_for(image):
    h,w=image.shape[:2]
    return PoolPerspective(4.,2.).calibrate([[0,0],[w-1,0],[w-1,h-1],[0,h-1]],image.shape)


class ObserverTests(unittest.TestCase):
    def test_optical_angle_uses_world_shape_and_manual_head_resolves_ambiguity(self):
        image,bbox=fish(25.);field=field_for(image);optical=OpticalBodyAxis()
        initial=optical.update(image,bbox,1.,field,1)
        self.assertFalse(initial['valid'])
        world=np.asarray(initial['centre_world'])+np.array([math.cos(math.radians(25)),-math.sin(math.radians(25))])*.3
        head=transform_points(np.linalg.inv(field.matrix),[world])[0]
        optical.calibrate(head,initial,field)
        image,bbox=fish(55.)
        measured=optical.update(image,bbox,1.1,field,1)
        self.assertAlmostEqual(measured['relative_deg'],30.,delta=.8)
        # A strong perspective must not turn a pixel slope into the measured angle.
        projection=cv2.getPerspectiveTransform(np.float32([[0,0],[1000,0],[1000,500],[0,500]]),
                                               np.float32([[220,80],[920,30],[980,590],[30,460]]))
        warped=cv2.warpPerspective(image,projection,(1100,650),borderValue=(160,100,20))
        corners=transform_points(projection,[[0,0],[1000,0],[1000,500],[0,500]])
        projected=PoolPerspective(4.,2.).calibrate(corners,warped.shape)
        points=transform_points(projection,[[400,150],[600,150],[600,350],[400,350]])
        bound=(*points.min(axis=0),*points.max(axis=0))
        axis=OpticalBodyAxis();pose=axis.update(warped,bound,1.,projected,1)
        angle=pose['world_deg'];error=min(abs((angle-55+180)%360-180),abs((angle+180-55+180)%360-180))
        self.assertLess(error,1.5)

    def test_loss_invalidates_axis_and_old_frozen_head_cannot_restore_a_new_track(self):
        image,bbox=fish();field=field_for(image);axis=OpticalBodyAxis()
        snapshot=axis.update(image,bbox,1.,field,1);axis.calibrate([580,250],snapshot,field)
        self.assertFalse(axis.update(image,None,1.1,field,1)['valid'])
        self.assertFalse(axis.update(image,bbox,2.,field,1)['valid'])
        with self.assertRaises(ValueError):axis.calibrate([580,250],snapshot,field)

    def make_session(self,folder,source='0'):
        bridge=WebBridge(observer=True)
        detector=SimpleNamespace(device='cpu',last_speed={},
            update=lambda frame,stamp:SimpleNamespace(metrics={'body_bbox':(400,150,600,350),'track_id':1},
                                                     position=(500,250),confidence=.9))
        session=ObserverSession(source,'fixture.pt',bridge,detector=detector,output=folder,calibration=Path(folder)/'pool.json')
        return session,bridge

    def test_csv_has_independent_optical_and_aligned_imu_and_trajectory_without_motion(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{'ROBOFISH_PARENT_TOKEN':'fixture','ROBOFISH_VISION_TOKEN':'fixture'}):
            session,bridge=self.make_session(folder);now=time.time();image,bbox=fish()
            session.field=field_for(image);session.pool_id='field-test'
            session.accept_frame(1,now,image);snapshot=dict(session.pose)
            session.optical.calibrate([580,250],snapshot,session.field);session.pose=dict(session.optical.latest)
            session.head={'calibrated':True,'calibration_id':'head-test','imu_zero_cw_deg':40.,'source_id':100}
            def telemetry(stamp,yaw,uptime):return {'valid':True,'yaw_cw_deg':yaw,'sample_time':stamp,'source_id':uptime,
                'heading_reference':{'valid':True,'zero_cw_deg':40.,'calibration_id':'head-test'},
                'onboard':{'heading_target_cw_deg':10.,'error_cw_deg':-20.,'yaw_active':True,'heading_target_active':True},
                'actuators':{'tail1':120.,'left_esc':80.},'control':{'axes':{'surge':35.}}}
            bridge.yaw_history.extend([telemetry(now-.4,40.,100),telemetry(now-.2,20.,200)])
            bridge.yaw_sample=telemetry(now,0.,300)
            session.samples.start({'observer_only':True});session.publish(write_sample=True);session.publish(write_sample=True)
            self.assertEqual(session.samples.status['rows'],1)
            self.assertTrue(bridge.result['reference_aligned'])
            self.assertEqual(bridge.result['optical']['relative_deg'],0.)
            session.samples.close()
            with Path(session.samples.status['file']).open(encoding='utf-8-sig',newline='') as stream:row=next(csv.DictReader(stream))
            self.assertAlmostEqual(float(row['imu_aligned_cw_deg']),30.,places=4)
            self.assertAlmostEqual(float(row['vision_relative_deg']),0.)
            self.assertAlmostEqual(float(row['vision_error_deg']),30.)
            self.assertEqual(row['reference_aligned'],'True');self.assertEqual(row['position_valid'],'True')
            self.assertEqual(row['tail1_deg'],'120.0')
            # Changing IMU cannot alter the independently fitted visual angle.
            bridge.yaw_sample=telemetry(now,-90.,400);session.publish()
            self.assertEqual(bridge.result['optical']['relative_deg'],0.)
            bridge.yaw_sample=telemetry(now-1.,-90.,400);session.publish()
            self.assertFalse(bridge.result['body_yaw']['usable'])
            with self.assertRaises(RuntimeError):bridge.parent({'task':'heading_target'})
            auth={'Authorization':'Bearer fixture'};client=bridge.app.test_client()
            for task in ('START','STOP','SPEED','RECORD'):
                self.assertEqual(client.post('/action',json={'task':task},headers=auth).status_code,400)
            for task in ('SAMPLE_STOP','POOL_CALIB','EXIT'):
                self.assertEqual(client.post('/action',json={'task':task},headers=auth).status_code,202)
                session.actions()
            self.assertTrue(bridge.closed.is_set())

    def test_replay_never_pairs_live_imu_and_preview_off_keeps_observer_ready(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{'ROBOFISH_PARENT_TOKEN':'fixture','ROBOFISH_VISION_TOKEN':'fixture'}):
            session,bridge=self.make_session(folder,'fixture.mp4');image,bbox=fish();session.field=field_for(image)
            session.accept_frame(1,time.time(),image,0.)
            bridge.yaw_sample={'valid':True,'yaw_cw_deg':50.,'sample_time':time.time(),'source_id':100}
            session.samples.start({'replay':True});session.preview=False;session.render_preview();session.publish(write_sample=True)
            session.samples.close()
            with Path(session.samples.status['file']).open(encoding='utf-8-sig',newline='') as stream:row=next(csv.DictReader(stream))
            for field in ('imu_aligned_cw_deg','imu_latest_cw_deg','native_target_cw_deg','tail1_deg','uptime_ms'):
                self.assertEqual(row[field],'')
            response=bridge.app.test_client().get('/state',headers={'Authorization':'Bearer fixture'}).json
            self.assertTrue(response['ready']);self.assertFalse(response['result_ready'])
            self.assertTrue(response['observation_only'])

    def test_csv_flush_error_is_not_reported_as_saved(self):
        with tempfile.TemporaryDirectory() as folder:
            samples=AngleSamples(folder);samples.start({});samples.file.close()
            samples.file=SimpleNamespace(flush=lambda:(_ for _ in ()).throw(OSError('disk full')),close=lambda:None)
            samples.close();self.assertFalse(samples.status['active'] or samples.status['saved'])
            self.assertIn('disk full',samples.status['error'])

    def test_interpolated_yaw_cannot_borrow_a_future_native_target(self):
        with patch.dict(os.environ,{'ROBOFISH_PARENT_TOKEN':'fixture','ROBOFISH_VISION_TOKEN':'fixture'}):
            bridge=WebBridge(observer=True)
            bridge.yaw_history.extend([
                {'sample_time':9.7,'telemetry_time':9.8,'source_id':100,'yaw_cw_deg':40.,'onboard':{'heading_target_cw_deg':0.}},
                {'sample_time':9.9,'telemetry_time':10.,'source_id':200,'yaw_cw_deg':20.,'onboard':{'heading_target_cw_deg':30.}}])
            sample=bridge.get_yaw_sample_at(9.85)
            self.assertAlmostEqual(sample['yaw_cw_deg'],25.)
            self.assertEqual(sample['onboard']['heading_target_cw_deg'],0.)
            self.assertEqual(sample['native_telemetry_time'],9.8)


if __name__=='__main__':unittest.main()
