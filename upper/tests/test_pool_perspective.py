"""Perspective geometry, saved input binding, and passive workstation actions."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

if importlib.util.find_spec('cv2') is None:
    raise unittest.SkipTest('Use the installed YOLO/OpenCV Python for perspective tests')
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision/src'))
from pool_perspective import PoolPerspective,image_heading,heading_arrow,transform_points
from web_runtime import WebBridge

POINTS=[[80,70],[570,30],[610,340],[30,320]]
SHAPE=(400,640,3)


class PoolTests(unittest.TestCase):
    def test_heading_and_arrow_are_invariant_to_position_under_perspective(self):
        pool=PoolPerspective().calibrate([[300,80],[1050,170],[1300,700],[50,600]],(776,1384))
        inverse=np.linalg.inv(pool.matrix)
        for centre in ([.3,.3],[2,1],[3.7,1.7]):
            for degrees in (-179,-90,-30,0,35,90,179):
                angle=np.radians(degrees)
                pixels=heading_arrow(inverse,centre,angle)
                self.assertAlmostEqual(image_heading(pool.matrix,*pixels),angle,places=10)
                np.testing.assert_allclose(transform_points(pool.matrix,pixels)[0],centre,atol=1e-9)

    def test_frozen_frame_imu_alignment_wraps_and_rejects_time_gaps_or_reboots(self):
        with patch.dict(os.environ,{'ROBOFISH_PARENT_TOKEN':'fixture','ROBOFISH_VISION_TOKEN':'fixture'}):
            bridge=WebBridge()
        bridge.yaw_history.extend([{'valid':True,'sample_time':10.,'yaw_cw_deg':179.,'source_id':100},
                                   {'valid':True,'sample_time':10.1,'yaw_cw_deg':-179.,'source_id':200}])
        aligned=bridge.get_yaw_sample_at(10.05)
        self.assertTrue(aligned['valid']);self.assertAlmostEqual(abs(aligned['yaw_cw_deg']),180.)
        self.assertFalse(bridge.get_yaw_sample_at(9.)['valid'])
        bridge.yaw_history[-1]['sample_time']=11.
        self.assertFalse(bridge.get_yaw_sample_at(10.1)['valid'])
        bridge.yaw_history[-1].update(sample_time=10.1,source_id=0)
        self.assertFalse(bridge.get_yaw_sample_at(10.05)['valid'])
    def test_projective_interior_point_and_all_four_corners(self):
        # A known projective camera, including a nonzero perspective denominator.
        image_from_world=np.array([[100.,5.,70.],[3.,100.,30.],[.03,.02,1.]])
        world=np.float64([[0,0],[4,0],[4,2],[0,2],[2.3,.7]])
        homogeneous=np.column_stack((world,np.ones(5)))@image_from_world.T
        image=homogeneous[:,:2]/homogeneous[:,2,None]
        pool=PoolPerspective().calibrate(image[:4],SHAPE)
        np.testing.assert_allclose(pool.transform(image),world,atol=2e-6)

    def test_invalid_quadrilaterals_are_rejected(self):
        for points in (POINTS[:3],[[0,0]]*4,[POINTS[i] for i in [0,2,1,3]],
                       [[-1,70],*POINTS[1:]],[[float('nan'),70],*POINTS[1:]],
                       [POINTS[i] for i in [2,3,0,1]]):
            with self.subTest(points=points),self.assertRaises(ValueError):PoolPerspective().calibrate(points,SHAPE)

    def test_saved_matrix_is_bound_to_source_and_resolution(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'build/checks/host-vision') as folder:
            path=Path(folder)/'pool.json';pool=PoolPerspective().calibrate(POINTS,SHAPE);pool.save(path,'sample.mp4')
            loaded=PoolPerspective();self.assertTrue(loaded.load(path,'sample.mp4',SHAPE))
            self.assertEqual(loaded.describe('sample.mp4')['dimension_source'],'configured')
            self.assertFalse(PoolPerspective().load(path,'0',SHAPE))
            self.assertFalse(PoolPerspective().load(path,'sample.mp4',(720,1280,3)))
            value=json.loads(path.read_text(encoding='utf8'));value['physical_width_m']=-1
            path.write_text(json.dumps(value),encoding='utf8')
            invalid=PoolPerspective();self.assertFalse(invalid.load(path,'sample.mp4',SHAPE))
            self.assertEqual(invalid.width_m,4.);self.assertIsNone(invalid.matrix)

    def test_rectified_image_crops_the_quad_with_correct_aspect(self):
        frame=np.zeros(SHAPE,np.uint8)
        colours=[(255,0,0),(0,255,0),(0,0,255),(255,255,255)]
        for point,colour in zip(POINTS,colours):cv2.circle(frame,tuple(point),12,colour,-1)
        image=PoolPerspective().calibrate(POINTS,SHAPE).rectify(frame)
        self.assertEqual(image.shape,(500,1000,3))
        for pixel,colour in zip([image[0,0],image[0,-1],image[-1,-1],image[-1,0]],colours):
            np.testing.assert_allclose(pixel,colour,atol=1)

    def test_existing_calibration_saves_one_test_image_and_serves_point_transforms(self):
        import vision_app
        with tempfile.TemporaryDirectory(dir=ROOT/'build/checks/host-vision') as folder, \
                patch.dict(os.environ,{'ROBOFISH_PARENT_TOKEN':'fixture','ROBOFISH_VISION_TOKEN':'fixture'}):
            bridge=WebBridge()
            client=bridge.app.test_client();headers={'Authorization':'Bearer fixture'}
            with patch.multiple(vision_app,create=True,RECORD_DIR=folder,CAMERA_INDEX='sample.mp4',
                    PHYSICAL_WIDTH=4.,PHYSICAL_HEIGHT=2.,POOL_SIZE_CONFIRMED=True,
                    POOL_PERSPECTIVE_METADATA={},POOL_PERSPECTIVE_TEST={}), \
                    patch.object(bridge,'parent',side_effect=AssertionError('No motion calls')), \
                    patch('cv2.warpPerspective',wraps=cv2.warpPerspective) as warp:
                config=Path(folder)/'pool.local.json'
                vision_app.save_pool_calibration(config,POINTS,640,400,frame=np.zeros(SHAPE,np.uint8))
                self.assertEqual(warp.call_count,1)
                with patch('cv2.imencode',side_effect=AssertionError('No preview work')):
                    for _ in range(5):bridge.send({'pool':{'perspective':vision_app.POOL_PERSPECTIVE_METADATA}})
                self.assertIsNotNone(vision_app.load_pool_calibration(config,640,400)[0])
                self.assertIsNone(vision_app.load_pool_calibration(config,1280,720)[0])
                with patch.object(vision_app,'CAMERA_INDEX','different-source'):
                    self.assertIsNone(vision_app.load_pool_calibration(config,640,400)[0])
                value=json.loads(config.read_text(encoding='utf8'))
                self.assertFalse(value['measured_dimensions'])
                self.assertEqual(value['dimension_source'],'configured')
                response=client.post('/transform',json={'points':POINTS},headers=headers)
                self.assertEqual(response.status_code,200)
                np.testing.assert_allclose(response.json['points_m'],[[0,0],[4,0],[4,2],[0,2]],atol=1e-6)
                self.assertEqual(client.post('/transform',json={'points':[[1,float('nan')]]},headers=headers).status_code,400)
                self.assertEqual(warp.call_count,1)


if __name__=='__main__':unittest.main()
