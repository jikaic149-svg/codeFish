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
from unittest.mock import Mock,patch

if importlib.util.find_spec('cv2') is None:raise unittest.SkipTest('Requires the installed YOLO/OpenCV Python')
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'upper'),str(ROOT/'vision')]
from angle_observation import OpticalBodyAxis,AngleSamples
from observer_runtime import ObserverSession
from pool_perspective import PoolPerspective,transform_points
from web_runtime import WebBridge,WorkstationComm


def fish(angle=0.,centre=(2.,1.)):
    image=np.full((500,1000,3),(160,100,20),np.uint8)
    point=tuple(round(x*250) for x in centre)
    cv2.ellipse(image,point,(80,23),-angle,0,360,(20,20,20),-1)
    return image,(point[0]-100,point[1]-100,point[0]+100,point[1]+100)


def field_for(image):
    h,w=image.shape[:2]
    return PoolPerspective(4.,2.).calibrate([[0,0],[w-1,0],[w-1,h-1],[0,h-1]],image.shape)


class ObserverTests(unittest.TestCase):
    def test_full_body_mask_has_finite_distance_weights(self):
        image=np.zeros((80,240,3),np.uint8)
        with np.errstate(over='raise',invalid='raise'):
            pose=OpticalBodyAxis().update(image,(0,0,240,80),1.,field_for(image),1)
        self.assertTrue(np.isfinite(pose['centre_world']).all())
        self.assertTrue(math.isfinite(pose['world_deg']))

    def test_control_uses_current_imu_without_a_historical_image_pair(self):
        session=ObserverSession.__new__(ObserverSession)
        session.head={'imu_zero_cw_deg':40.,'visual_zero_world_deg':15.,'source_id':100}
        sample={'valid':True,'yaw_cw_deg':30.,'sample_time':999.95,'source_id':200}
        session.bridge=SimpleNamespace(get_yaw_sample=lambda:sample,get_yaw_sample_at=Mock(return_value={'valid':False}))
        with patch('observer_runtime.time.time',return_value=1000.):
            heading,source=session.control_heading()
            self.assertEqual(source,'IMU_YAW')
            np.testing.assert_allclose(heading,[math.cos(math.radians(25)),-math.sin(math.radians(25))])
            session.bridge.get_yaw_sample_at.assert_not_called()
            sample['sample_time']=998.
            self.assertEqual(session.control_heading(),(None,'UNAVAILABLE'))

    def test_head_selection_stays_in_perspective_and_maps_to_original_pixels(self):
        image=np.zeros((80,240,3),np.uint8);session=ObserverSession.__new__(ObserverSession)
        session.frame=image;session.field=PoolPerspective(4.,2.).calibrate([[20,10],[220,3],[235,75],[8,65]],image.shape)
        session.view_mode='perspective';session.interaction='HEAD_DIRECTION';session.debug=False
        session.frozen={'image':image,'display_space':'field','selection_id':'head'}
        session.raw_overlay=lambda frozen:image.copy()
        shown,_=session.compose_preview()
        self.assertEqual(shown.shape[:2],(500,1000));self.assertEqual(session.display_space,'field')
        point=session.pointer_point({'x':0,'y':0,'space':'field','selection_id':'head'})
        np.testing.assert_allclose(point,[20,10],atol=1e-5)

    def test_sender_period_includes_http_time_and_keeps_stop_reason(self):
        now=[0.];starts=[];waits=[]
        def parent(body):
            if body['task']=='heading':starts.append(now[0])
            now[0]+=.08
            return {'accepted':True}
        comm=WorkstationComm(SimpleNamespace(parent=parent),start_sender=False);comm.authorized=True
        comm._udp_latest={'heading_cw_deg':10.,'power_percent':30.,'brake':False}
        def wait(delay):
            waits.append(delay);now[0]+=delay;comm._udp_latest_t=now[0]
        comm._stop=SimpleNamespace(is_set=lambda:len(starts)>=5,wait=wait)
        with patch('web_runtime.time.monotonic',side_effect=lambda:now[0]):
            comm._send()
            self.assertEqual(comm.sent_targets,5)
            np.testing.assert_allclose(starts,[0,.1,.2,.3,.4],atol=1e-9)
            np.testing.assert_allclose(waits,[.02]*5,atol=1e-9)
            comm.stop_now('imu_unavailable')
        self.assertEqual(comm.last_stop_reason,'imu_unavailable');self.assertFalse(comm.authorized)

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
            for task in ('heading_target','prepare','target'):
                self.assertEqual(client.post('/action',json={'task':task},headers=auth).status_code,400)
            self.assertEqual(client.post('/action',json={'task':'THRESHOLD','value':300},headers=auth).status_code,400)
            self.assertEqual(client.post('/action',json={'task':'VIEW','value':'unknown'},headers=auth).status_code,400)
            for task in ('SAMPLE_STOP','POOL_CALIB','EXIT'):
                self.assertEqual(client.post('/action',json={'task':task},headers=auth).status_code,202)
                session.actions()
            self.assertTrue(bridge.closed.is_set())


    def test_single_point_is_shared_by_guidance_overlay_trajectory_and_csv(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{'ROBOFISH_PARENT_TOKEN':'fixture','ROBOFISH_VISION_TOKEN':'fixture'}):
            session,bridge=self.make_session(folder);now=time.time();image,_=fish()
            session.field=field_for(image);session.accept_frame(1,now,image)
            session.optical.calibrate([580,250],session.pose,session.field)
            session.pose=dict(session.optical.latest)
            # Deliberately offset the optical centre: it must only affect metrology.
            session.pose.update(centre_world=[3.,1.5],x_m=3.,y_m=1.5)
            expected=session.field.transform([[500,250]])[0]
            np.testing.assert_allclose(session.position(),expected)
            np.testing.assert_allclose(session.history[-1],expected)
            session.path_px=[(500,250),(800,250)]
            with patch.object(session.guide,'start',wraps=session.guide.start) as start:
                session.guide.start(session.field.transform(session.path_px),session.position(),session.track_time,[1,0])
                np.testing.assert_allclose(start.call_args.args[1],expected)
            frozen=session.snapshot()
            np.testing.assert_array_equal(session.raw_overlay()[250,500],[0,0,255])
            session.samples.start({});session.publish(write_sample=True)
            point=bridge.result['position']
            self.assertTrue(point['valid'])
            self.assertEqual(point['source'],'YOLO_BOX_CENTRE')
            self.assertEqual((point['x_px'],point['y_px'],point['track_id']),(500.,250.,1))
            self.assertEqual(point['sample_time'],session.track_time)
            np.testing.assert_allclose([point['x_m'],point['y_m']],expected)
            session.samples.close()
            with Path(session.samples.status['file']).open(encoding='utf-8-sig',newline='') as stream:
                row=next(csv.DictReader(stream))
            np.testing.assert_allclose([float(row['x_m']),float(row['y_m'])],expected,atol=1e-5)
            self.assertEqual(row['position_source'],'YOLO_BOX_CENTRE')
            session.detection.position=(525,250)
            np.testing.assert_array_equal(session.raw_overlay(frozen)[250,500],[0,0,255])
            session.publish()
            np.testing.assert_allclose([bridge.result['position'][key] for key in ('x_m','y_m')],
                                      session.field.transform([[525,250]])[0])
            session.detection.position=None;session.publish()
            self.assertFalse(bridge.result['position']['valid'])
            self.assertIsNone(bridge.result['position']['x_m'])
            session.detection.position=(525,250);session.received_at=now-2.;session.publish()
            self.assertFalse(bridge.result['position']['valid'])
            self.assertIsNone(bridge.result['position']['x_px'])
            session.received_at=time.time();session.field=PoolPerspective();session.publish()
            self.assertFalse(bridge.result['position']['valid'])
            self.assertEqual(bridge.result['position']['x_px'],525.)

    def test_automatic_tracking_still_sends_targets_and_stops_without_preview_actions(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{'ROBOFISH_PARENT_TOKEN':'fixture','ROBOFISH_VISION_TOKEN':'fixture'}):
            session,bridge=self.make_session(folder);now=time.time();image,_=fish()
            session.field=field_for(image)
            cv2.ellipse(image,(581,250),(11,13),0,0,360,(182,215,215),-1)
            for i in range(6):session.accept_frame(i+1,now-.20+(i-5)*.04,image)
            session.path_px=[(500,250),(800,250)]
            session.head={'imu_aligned':True,'imu_zero_cw_deg':0.,'visual_zero_world_deg':0.}
            session.comm=SimpleNamespace(authorized=True,last_error='',last_round_trip_ms=None,speed_status={},
                sent_targets=0,last_target_at=None,last_stop_reason='',
                ensure_bionic_mode=Mock(return_value=True),stop_now=Mock(return_value=True),process_tracking_error=Mock())
            client=bridge.app.test_client();headers={'Authorization':'Bearer fixture'}
            for task in ('START','STOP'):
                self.assertEqual(client.post('/action',json={'task':task},headers=headers).status_code,400)
            with patch('observer_runtime.time.time',return_value=now), \
                 patch.object(session,'control_heading',return_value=([1.,0.],'IMU_YAW')), \
                 patch.object(bridge,'get_yaw_sample',return_value={'valid':True,'sample_time':now,'yaw_cw_deg':0.}):
                self.assertEqual(client.post('/action',json={'task':'AUTO_START'},headers=headers).status_code,202)
                session.actions()
                self.assertEqual(bridge.last_task['state'],'processed',bridge.last_task)
                self.assertTrue(session.automatic and session.tracking and bridge.control_enabled)
                session.accept_frame(7,now-.04,image)
                command=session.comm.process_tracking_error.call_args.kwargs
                self.assertAlmostEqual(command['target_heading_cw_deg'],0.,delta=.05)
                self.assertGreater(command['speed_target_mps'],0.)
                session.publish();self.assertTrue(bridge.result['tracking']['automatic'])
                self.assertNotIn('preview_only',bridge.result['tracking'])
                self.assertEqual(client.post('/action',json={'task':'AUTO_STOP'},headers=headers).status_code,202)
                session.actions();session.comm.stop_now.assert_called_once()
                self.assertFalse(session.automatic or session.tracking or bridge.control_enabled)
                self.assertEqual(client.post('/action',json={'task':'AUTO_START'},headers=headers).status_code,202)
                session.actions()
                lost=SimpleNamespace(metrics={},position=None,confidence=0.)
                with patch.object(session.tracker,'update',return_value=lost):
                    session.accept_frame(8,now,image)
                self.assertFalse(session.automatic or session.tracking)
                self.assertEqual(session.comm.stop_now.call_count,2)

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

    def test_rectified_view_selection_coordinates_and_restored_debug_tools(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{'ROBOFISH_PARENT_TOKEN':'fixture','ROBOFISH_VISION_TOKEN':'fixture'}):
            session,bridge=self.make_session(folder,'fixture.mp4')
            image=np.full((600,1000,3),(160,100,20),np.uint8)
            cv2.ellipse(image,(500,250),(80,23),0,0,360,(20,20,20),-1)
            session.field=PoolPerspective(4.,2.).calibrate([[100,70],[900,30],[940,540],[60,480]],image.shape)
            session.accept_frame(1,time.time(),image,0.);session.render_preview();session.publish()
            result=cv2.imdecode(np.frombuffer(bridge.frames['result'],np.uint8),1)
            raw=cv2.imdecode(np.frombuffer(bridge.frames['raw'],np.uint8),1)
            self.assertEqual(result.shape[:2],(500,1000));self.assertEqual(raw.shape[:2],(600,1000))
            self.assertEqual(bridge.metadata['display_space'],'field')
            point=session.pointer_point({'x':.3,'y':.6,'space':'field'})
            np.testing.assert_allclose(session.field.transform([point])[0],[.3*999*4/1000,.6*499*2/500],atol=1e-7)
            client=bridge.app.test_client();headers={'Authorization':'Bearer fixture'}
            def action(body):
                self.assertEqual(client.post('/action',json=body,headers=headers).status_code,202)
                session.actions()
                self.assertEqual(bridge.last_task['state'],'processed',bridge.last_task)
            with patch.object(bridge,'parent',side_effect=AssertionError('visual actuation')) as motion:
                action({'task':'HEAD_DIRECTION'});session.render_preview()
                self.assertEqual(session.display_space,'field')
                with self.assertRaises(ValueError):session.pointer_point({'x':.3,'y':.6,'space':'image'})
                world=session.field.transform([[580,250]])[0]
                action({'kind':'down','x':world[0]/4*1000/999,'y':world[1]/2*500/499,'space':'field'})
                self.assertTrue(session.preview)
                action({'task':'CLAHE'});action({'task':'BINARY'});action({'task':'DEBUG'})
                action({'task':'THRESHOLD','value':110});action({'task':'PREVIEW_FPS','value':10})
                self.assertTrue(session.binary and session.clahe);self.assertEqual(session.threshold,110)
                action({'task':'VIEW','value':'perspective'});session.render_preview()
                for kind,x in [('down',.4),('move',.5),('up',.7)]:action({'kind':kind,'x':x,'y':.5,'space':'field'})
                self.assertGreaterEqual(len(session.path_px),2)
                for task in ('START','STOP'):
                    self.assertEqual(client.post('/action',json={'task':task},headers=headers).status_code,400)
                session.publish();self.assertFalse(bridge.result['tracking']['active'])
                self.assertNotIn('preview_only',bridge.result['tracking'])
                action({'task':'CLEAR_PATH'})
                self.assertFalse(session.path_px or session.tracking)
                action({'task':'TURN_CALIB'});self.assertTrue(session.turn.active)
                action({'task':'CLEAR_PATH'});self.assertFalse(session.turn.active)
                action({'task':'SNAPSHOT'});self.assertTrue(list(Path(folder).glob('vision-snapshot-*.png')))
                action({'task':'RECORD'});session.preview=False
                for index in range(2,7):
                    session.accept_frame(index,time.time(),image,index/30.);session.publish(write_sample=True)
                action({'task':'RECORD'})
                from video_recorder import recording_status
                recorded=recording_status();self.assertTrue(recorded['saved'],recorded)
                capture=cv2.VideoCapture(recorded['path'])
                try:
                    self.assertEqual(int(capture.get(cv2.CAP_PROP_FRAME_COUNT)),5)
                    self.assertEqual(capture.read()[1].shape[:2],(600,1000))
                finally:capture.release()
                with Path(recorded['csv_path']).open(encoding='utf-8-sig',newline='') as stream:
                    self.assertEqual(len(list(csv.DictReader(stream))),5)
                motion.assert_not_called()


if __name__=='__main__':unittest.main()
