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
from observer_runtime import LatestCamera,ObserverSession
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
    def test_camera_requests_120_but_records_the_negotiated_rate(self):
        capture=Mock();capture.get.return_value=30.
        with patch('observer_runtime.cv2.VideoCapture',return_value=capture), \
                patch('observer_runtime.threading.Thread'):
            camera=LatestCamera(0)
            capture.set.assert_any_call(cv2.CAP_PROP_FPS,120)
            capture.set.assert_any_call(cv2.CAP_PROP_AUTO_EXPOSURE,1.)
            self.assertEqual(camera.reported_fps,30.)
            self.assertEqual(camera.measured_fps,0.)
            camera.close()

    def camera_fixture(self):
        values={cv2.CAP_PROP_EXPOSURE:-1.,cv2.CAP_PROP_AUTO_EXPOSURE:0.,cv2.CAP_PROP_FPS:60.}
        capture=Mock();capture.get.side_effect=lambda prop:values.get(prop,0.)
        def set_property(prop,value):
            values[prop]=value
            return True
        capture.set.side_effect=set_property
        with patch('observer_runtime.cv2.VideoCapture',return_value=capture),patch('observer_runtime.threading.Thread'):
            camera=LatestCamera(0)
        camera._exposure['gray_mean']=128.
        capture.read.side_effect=lambda:(True,np.full((16,24,3),
            128 if values[cv2.CAP_PROP_AUTO_EXPOSURE] else min(240,max(1,2**(values[cv2.CAP_PROP_EXPOSURE]+6))),np.uint8))
        self.addCleanup(camera.close)
        return camera,capture,values

    def test_exposure_changes_are_queued_and_applied_before_read_on_camera_thread(self):
        camera,capture,values=self.camera_fixture()
        camera._exposure['mode']='manual'
        calls=capture.set.call_count
        camera.request_exposure(1)
        self.assertEqual(capture.set.call_count,calls)
        self.assertEqual(camera.exposure_status()['pending'],1)
        image=np.full((16,24,3),80,np.uint8)
        def read():
            self.assertEqual(values[cv2.CAP_PROP_AUTO_EXPOSURE],0.)
            self.assertEqual(values[cv2.CAP_PROP_EXPOSURE],0.)
            camera.closed.set()
            return True,image
        capture.read.side_effect=read
        camera._read()
        status=camera.exposure_status()
        self.assertEqual(status['mode'],'manual')
        self.assertEqual(status['requested'],0.)
        self.assertEqual(status['readback'],0.)
        self.assertEqual(status['gray_mean'],80.)
        self.assertEqual(status['pending'],0)
        self.assertIsNone(status['error'])
        self.assertIs(camera.read(0)[2],image,'Exposure must not be a hidden digital image multiplier')

    def test_manual_exposure_has_no_invented_minus_one_upper_limit(self):
        camera,capture,values=self.camera_fixture()
        camera._exposure['mode']='manual'
        for _ in range(3):camera._apply_exposure(1)
        self.assertEqual(values[cv2.CAP_PROP_EXPOSURE],2.)
        self.assertEqual(camera.exposure_status()['readback'],2.)
        camera._apply_exposure(-1)
        self.assertEqual(camera.exposure_status()['readback'],1.)
        camera._apply_exposure(None)
        self.assertEqual(camera.exposure_status()['mode'],'auto')
        self.assertEqual(values[cv2.CAP_PROP_AUTO_EXPOSURE],1.)

    def test_auto_mode_never_silently_switches_to_dark_stale_manual_value(self):
        camera,capture,values=self.camera_fixture()
        capture.set.reset_mock()
        camera._apply_exposure(1)
        self.assertEqual(camera.exposure_status()['mode'],'auto')
        self.assertIn('先点击切换手动曝光',camera.exposure_status()['error'])
        capture.set.assert_not_called()
        camera._apply_exposure(0)
        self.assertEqual(camera.exposure_status()['mode'],'manual')
        self.assertIsNone(camera.exposure_status()['error'])
        camera._apply_exposure(1)
        self.assertEqual(camera.exposure_status()['readback'],0.)

    def test_exposure_driver_clamping_is_an_error_and_restores_auto(self):
        camera,capture,values=self.camera_fixture()
        setter=capture.set.side_effect
        capture.set.side_effect=lambda prop,value:True if prop==cv2.CAP_PROP_EXPOSURE else setter(prop,value)
        camera._exposure['mode']='manual'
        camera._apply_exposure(1)
        status=camera.exposure_status()
        self.assertEqual(status['mode'],'manual')
        self.assertEqual(status['readback'],-1.)
        self.assertIn('不一致',status['error'])
        self.assertEqual(values[cv2.CAP_PROP_EXPOSURE],-1.)
        camera._apply_exposure(None)
        self.assertIsNone(camera.exposure_status()['error'])

    def test_exposure_rejects_unsupported_mode_and_invalid_readback(self):
        camera,capture,values=self.camera_fixture()
        setter=capture.set.side_effect
        capture.set.side_effect=lambda prop,value:False if prop==cv2.CAP_PROP_AUTO_EXPOSURE and value==0. else setter(prop,value)
        capture.set.reset_mock()
        camera._apply_exposure(0)
        self.assertIn('拒绝手动曝光',camera.exposure_status()['error'])
        self.assertFalse(any(call.args[0]==cv2.CAP_PROP_EXPOSURE for call in capture.set.call_args_list))
        capture.set.side_effect=setter;values[cv2.CAP_PROP_EXPOSURE]=float('nan')
        camera._apply_exposure(0)
        self.assertIn('有效曝光读值',camera.exposure_status()['error'])
        capture.set.return_value=False;capture.set.side_effect=None
        camera._apply_exposure(None)
        self.assertIn('拒绝自动曝光',camera.exposure_status()['error'])

    def test_exposure_queue_is_bounded_without_touching_the_driver(self):
        camera,capture,_=self.camera_fixture()
        capture.set.reset_mock()
        for _ in range(16):camera.request_exposure(1)
        with self.assertRaisesRegex(ValueError,'操作过多'):camera.request_exposure(1)
        capture.set.assert_not_called()

    def test_exposure_actions_publish_hardware_state_and_never_request_motion(self):
        camera,capture,_=self.camera_fixture()
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{'ROBOFISH_PARENT_TOKEN':'fixture','ROBOFISH_VISION_TOKEN':'fixture'}):
            session,bridge=self.make_session(folder);session.capture=camera
            client=bridge.app.test_client();headers={'Authorization':'Bearer fixture'}
            with patch.object(bridge,'parent',side_effect=AssertionError('Exposure must not actuate the fish')) as motion:
                for action in ({'task':'EXP_UP','exposure_mode':'auto'},{'task':'EXP_UP','exposure_mode':'manual'},{'task':'EXP_UP'},{'task':'EXP_DOWN'}):
                    self.assertEqual(client.post('/action',json=action,headers=headers).status_code,202)
                    session.actions()
                    self.assertEqual(bridge.last_task['state'],'processed')
                session.publish()
                self.assertEqual(bridge.result['camera_exposure']['pending'],4)
                self.assertEqual(bridge.result['camera_exposure']['mode'],'auto')
                self.assertEqual(bridge.result['debug']['exposure'],-1.)
                self.assertEqual([camera.exposure_commands.get_nowait() for _ in range(4)],[None,0,1,-1])
                self.assertEqual(client.post('/action',json={'task':'EXP_UP','exposure_mode':'bad'},headers=headers).status_code,400)
                session.replay=True
                self.assertEqual(client.post('/action',json={'task':'EXP_AUTO'},headers=headers).status_code,202)
                session.actions()
                self.assertEqual(bridge.last_task['state'],'error')
                self.assertIn('录像',bridge.last_task['status'])
                motion.assert_not_called()

    def test_120fps_preview_is_validated_applied_and_published(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{'ROBOFISH_PARENT_TOKEN':'fixture','ROBOFISH_VISION_TOKEN':'fixture'}):
            session,bridge=self.make_session(folder,'fixture.mp4')
            self.assertEqual(session.preview_fps,120)
            client=bridge.app.test_client();headers={'Authorization':'Bearer fixture'}
            for value in (60,120):
                self.assertEqual(client.post('/action',json={'task':'PREVIEW_FPS','value':value},headers=headers).status_code,202)
                session.actions()
                self.assertEqual(session.preview_fps,value)
            for value in (0,121,True,'120',120.):
                self.assertEqual(client.post('/action',json={'task':'PREVIEW_FPS','value':value},headers=headers).status_code,400)
            image,_=fish()
            session.accept_frame(1,time.time(),image,0.)
            session.render_preview();session.publish()
            self.assertEqual(bridge.result['performance']['preview_max_fps'],120)
            self.assertEqual(bridge.metadata['preview_max_fps'],120)
            self.assertIn('processing_fps',bridge.result['performance'])
            self.assertIsNone(bridge.result['performance']['capture_requested_fps'])

    def test_preview_schedule_does_not_halve_the_rate_with_small_capture_jitter(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{'ROBOFISH_PARENT_TOKEN':'fixture','ROBOFISH_VISION_TOKEN':'fixture'}):
            session,bridge=self.make_session(folder,'fixture.mp4')
            image,_=fish();session.accept_frame(1,time.time(),image,0.)
            for index,stamp in enumerate((1000.004,1000.0124,1000.0208,1000.0291),1):
                session.index=index
                with patch('observer_runtime.time.monotonic',return_value=stamp):
                    session.render_preview()
            self.assertEqual(session.preview_revision,4)

    def test_stream_sends_only_the_latest_frame_without_a_polling_sleep(self):
        with patch.dict(os.environ,{'ROBOFISH_PARENT_TOKEN':'fixture','ROBOFISH_VISION_TOKEN':'fixture'}):
            bridge=WebBridge(observer=True);bridge.frames={'result':b'first'}
            with patch('web_runtime.time.sleep',side_effect=AssertionError('stream must wait for new frames')):
                response=bridge.app.test_client().get('/stream/result',headers={'Authorization':'Bearer fixture'},buffered=False)
                self.assertIn(b'first',next(response.response))
                with bridge.frame_ready:
                    bridge.frames['result']=b'skipped'
                    bridge.frames['result']=b'latest'
                    bridge.frame_ready.notify_all()
                data=next(response.response)
                self.assertIn(b'latest',data);self.assertNotIn(b'skipped',data)
                bridge.close()
                with self.assertRaises(StopIteration):next(response.response)
                response.close()

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

    def test_head_click_sets_exact_yolo_to_click_heading_not_just_axis_sign(self):
        image,bbox=fish();field=field_for(image);axis=OpticalBodyAxis()
        pose=axis.update(image,bbox,1.,field,1)
        body=[480,260];head=[555,210]
        delta=np.diff(field.transform([body,head]),axis=0)[0]
        expected=math.degrees(math.atan2(-delta[1],delta[0]))
        zero=axis.calibrate(head,pose,field,body)
        self.assertAlmostEqual(zero,expected)
        self.assertAlmostEqual(axis.latest['world_deg'],expected)
        self.assertAlmostEqual(axis.latest['relative_deg'],0.)
        image,bbox=fish(30.)
        measured=axis.update(image,bbox,1.1,field,1)
        self.assertAlmostEqual(measured['world_deg'],expected+30.,delta=.8)
        self.assertAlmostEqual(measured['relative_deg'],30.,delta=.8)
        # Recalibration uses the raw axis, without accumulating the previous offset.
        centre=measured['centre_px'];point=[centre[0]-70,centre[1]+40]
        delta=np.diff(field.transform([centre,point]),axis=0)[0]
        zero=axis.calibrate(point,measured,field,centre)
        self.assertAlmostEqual(zero,math.degrees(math.atan2(-delta[1],delta[0])))
        self.assertAlmostEqual(axis.latest['world_deg'],zero)

    def test_head_calibration_imu_arrow_ends_at_exact_click_in_raw_and_perspective(self):
        for view in ('raw','perspective'):
            with self.subTest(view=view),tempfile.TemporaryDirectory() as folder,patch.dict(
                    os.environ,{'ROBOFISH_PARENT_TOKEN':'fixture','ROBOFISH_VISION_TOKEN':'fixture'}):
                session,bridge=self.make_session(folder);image,_=fish()
                session.field=PoolPerspective(4.,2.).calibrate(
                    [[100,50],[900,20],[940,470],[60,440]],image.shape)
                session.accept_frame(1,time.time(),image)
                session.detection.position=(480,260)
                session.view_mode=view;session.compose_preview()
                imu={'valid':True,'yaw_cw_deg':42.,'source_id':100}
                with patch.object(bridge,'get_yaw_sample_at',return_value=imu), \
                        patch.object(bridge,'get_yaw_sample',return_value=imu), \
                        patch.object(bridge,'parent',side_effect=AssertionError('no motion')):
                    bridge.events.put({'id':'freeze','task':'HEAD_DIRECTION'});session.actions()
                    point=[555,210]
                    if view=='raw':x,y=np.asarray(point)/[999,499];space='image'
                    else:
                        width,height=session.field_size()
                        x,y=session.field.transform([point])[0]/[4.,2.]*[width/(width-1),height/(height-1)]
                        space='field'
                    bridge.events.put({'id':'click','kind':'down','x':float(x),'y':float(y),'space':space})
                    session.actions()
                    self.assertEqual(bridge.last_task['state'],'processed',bridge.last_task)
                    np.testing.assert_allclose(session.head['body_pixel'],[480,260])
                    np.testing.assert_allclose(session.head['head_pixel'],point,atol=1e-7)
                    aligned=dict(imu,heading_reference={'valid':True,'zero_cw_deg':42.,
                        'calibration_id':session.head['calibration_id']})
                    with patch.object(bridge,'get_yaw_sample_at',return_value=aligned), \
                            patch('observer_runtime.cv2.arrowedLine') as arrows:
                        session.raw_overlay()
                        self.assertEqual(arrows.call_count,1)
                        for call in arrows.call_args_list:
                            self.assertEqual(call.args[1],(480,260))
                            self.assertEqual(call.args[2],(555,210))
                            self.assertEqual(call.args[3],(0,220,0))

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

    def test_heading_arrows_and_debug_point_share_p_not_optical_centroid(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{'ROBOFISH_PARENT_TOKEN':'fixture','ROBOFISH_VISION_TOKEN':'fixture'}):
            session,bridge=self.make_session(folder);image,_=fish()
            session.field=field_for(image);session.accept_frame(1,time.time(),image)
            session.pose.update(centre_px=[650,280],centre_world=[2.6,1.12],world_deg=0.)
            session.head={'calibration_id':'head','visual_zero_world_deg':0.}
            aligned={'valid':True,'yaw_cw_deg':10.,
                'heading_reference':{'valid':True,'zero_cw_deg':0.,'calibration_id':'head'},
                'onboard':{'heading_target_cw_deg':20.}}
            session.debug=True;session.tracker._flow_points=np.float32([[[600,300]]])
            with patch.object(bridge,'get_yaw_sample_at',return_value=aligned),patch('observer_runtime.cv2.arrowedLine') as arrows:
                rendered=session.raw_overlay()
                self.assertEqual(arrows.call_count,2)
                self.assertTrue(all(call.args[1]==(500,250) for call in arrows.call_args_list))
                self.assertEqual([call.args[3] for call in arrows.call_args_list],[(0,220,0),(0,180,255)])
                np.testing.assert_array_equal(rendered[250,500],[0,0,255])
                frozen=session.snapshot()
            session.detection.position=(520,260)
            with patch.object(bridge,'get_yaw_sample_at',side_effect=AssertionError('frozen IMU must not be re-fetched')),patch('observer_runtime.cv2.arrowedLine') as arrows:
                session.raw_overlay(frozen)
                self.assertEqual(arrows.call_count,2)
                self.assertTrue(all(call.args[1]==(500,250) for call in arrows.call_args_list))
                self.assertEqual([call.args[3] for call in arrows.call_args_list],[(0,220,0),(0,180,255)])

    def test_valid_imu_survives_optical_failure_but_invalid_imu_stays_hidden(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{'ROBOFISH_PARENT_TOKEN':'fixture','ROBOFISH_VISION_TOKEN':'fixture'}):
            session,bridge=self.make_session(folder);image,_=fish()
            session.field=field_for(image);session.accept_frame(1,time.time(),image)
            session.pose={'valid':False,'reason':'BODY_MASK_UNAVAILABLE'}
            session.head={'calibration_id':'head','visual_zero_world_deg':0.}
            aligned={'valid':True,'yaw_cw_deg':10.,
                'heading_reference':{'valid':True,'zero_cw_deg':0.,'calibration_id':'head'}}
            with patch.object(bridge,'get_yaw_sample_at',return_value=aligned),patch('observer_runtime.cv2.arrowedLine') as arrows:
                session.raw_overlay()
                self.assertEqual(arrows.call_count,1)
                self.assertEqual(arrows.call_args.args[1],(500,250))
                self.assertEqual(arrows.call_args.args[3],(0,220,0))
                for invalid in ({'valid':False},dict(aligned,heading_reference={'valid':False}),
                                dict(aligned,heading_reference={'valid':True,'calibration_id':'other'})):
                    arrows.reset_mock()
                    with patch.object(bridge,'get_yaw_sample_at',return_value=invalid):session.raw_overlay()
                    arrows.assert_not_called()
                session.detection.position=None;arrows.reset_mock();session.raw_overlay();arrows.assert_not_called()

    def test_old_selection_events_cannot_change_new_frozen_selection(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{'ROBOFISH_PARENT_TOKEN':'fixture','ROBOFISH_VISION_TOKEN':'fixture'}):
            session,bridge=self.make_session(folder);image,_=fish()
            session.field=field_for(image);session.accept_frame(1,time.time(),image)
            session.interaction='POOL_CALIB';session.frozen=session.snapshot()
            frozen=session.frozen;last_task=bridge.last_task
            for kind in ('down','move','up','right'):
                bridge.events.put({'id':kind,'kind':kind,'x':.5,'y':.5,'space':'image','selection_id':'old'})
            session.actions()
            self.assertIs(session.frozen,frozen);self.assertEqual(session.interaction,'POOL_CALIB')
            self.assertEqual(session.points,[]);self.assertEqual(bridge.last_task,last_task)

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
            session.field=field_for(image);session.accept_frame(1,now,image)
            session.path_px=[(500,250),(800,250)]
            session.head={'imu_aligned':True,'imu_zero_cw_deg':0.,'visual_zero_world_deg':0.}
            session.comm=SimpleNamespace(authorized=True,last_error='',last_round_trip_ms=None,speed_status={},
                sent_targets=0,last_target_at=None,last_stop_reason='',
                ensure_bionic_mode=Mock(return_value=True),stop_now=Mock(return_value=True),process_tracking_error=Mock())
            client=bridge.app.test_client();headers={'Authorization':'Bearer fixture'}
            for task in ('START','STOP'):
                self.assertEqual(client.post('/action',json={'task':task},headers=headers).status_code,400)
            with patch.object(session,'control_heading',return_value=([1.,0.],'IMU_YAW')),                  patch.object(bridge,'get_yaw_sample',return_value={'valid':True,'sample_time':now,'yaw_cw_deg':0.}):
                self.assertEqual(client.post('/action',json={'task':'AUTO_START'},headers=headers).status_code,202)
                session.actions()
                self.assertEqual(bridge.last_task['state'],'processed')
                self.assertTrue(session.automatic and session.tracking and bridge.control_enabled)
                session.accept_frame(2,now+.05,image)
                command=session.comm.process_tracking_error.call_args.kwargs
                self.assertAlmostEqual(command['target_heading_cw_deg'],0.)
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
                    session.accept_frame(3,now+.1,image)
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
