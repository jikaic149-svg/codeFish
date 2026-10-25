"""Independent optical measurements and CSV sampling, with no control sender."""
import csv
from datetime import datetime,timezone
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np

from fixed_reference_tracker import _clip_roi
from pool_perspective import transform_points


def wrap_degrees(value):
    return (float(value)+180.)%360.-180.


def body_mask(frame,value_max=130):
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    neutral=(hsv[:,:,1]<=90)|(hsv[:,:,0]<75)|(hsv[:,:,0]>135)
    mask=((hsv[:,:,2]<=value_max)&neutral).astype(np.uint8)*255
    return cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))


class OpticalBodyAxis:
    """YOLO selects identity/ROI; the image alone supplies the measured axis.

    Fit body pixels in field metres, weighting the thick rigid body above the
    thin moving tail. A manual head click resolves the 180-degree ambiguity.
    This is a body-axis proxy, not a trained head keypoint or an IMU estimate.
    """
    def __init__(self):self.reset()

    def reset(self):
        self.direction=None;self.zero_world=None;self.initialized=False
        self.last_time=None;self.track_id=None;self.latest=None;self.continuity_id=0

    def update(self,frame,bbox,stamp,field,track_id=None,value_max=130):
        if self.latest is not None and self.latest.get('sample_time')==stamp:return dict(self.latest)
        if field.matrix is None:return {'valid':False,'reason':'NEEDS_POOL_CALIBRATION'}
        roi=_clip_roi(bbox,frame.shape[1],frame.shape[0]) if bbox is not None else None
        if roi is None:return {'valid':False,'reason':'NO_FISH'}
        x1,y1,x2,y2=roi;mask=body_mask(frame[y1:y2,x1:x2],value_max)
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        contours=[c for c in contours if cv2.contourArea(c)>=25]
        if not contours:return {'valid':False,'reason':'BODY_MASK_UNAVAILABLE'}
        solid=np.zeros_like(mask);cv2.drawContours(solid,[max(contours,key=cv2.contourArea)],-1,255,-1)
        # Keep a zero-valued background even when the body fills the entire ROI.
        distance=cv2.distanceTransform(np.pad(solid,1),cv2.DIST_L2,3)[1:-1,1:-1]
        yy,xx=np.nonzero(solid);pixels=np.column_stack((xx+x1,yy+y1)).astype(np.float64)
        weights=np.maximum(distance[yy,xx].astype(np.float64),.1)**2
        world=field.transform(pixels);centre=np.average(world,axis=0,weights=weights);points=world-centre
        values,vectors=np.linalg.eigh((points*weights[:,None]).T@points/weights.sum())
        ratio=float(values[1]/max(values[0],1e-12));axis=vectors[:,1]
        if ratio<1.5:return {'valid':False,'reason':'BODY_AXIS_AMBIGUOUS','axis_ratio':ratio}
        if self.last_time is not None and (stamp-self.last_time>.6 or track_id!=self.track_id):
            self.initialized=False;self.continuity_id+=1
        if self.direction is not None and axis@self.direction<0:axis=-axis
        self.direction=axis;self.last_time=float(stamp);self.track_id=track_id
        angle=wrap_degrees(math.degrees(math.atan2(-axis[1],axis[0])))
        inverse=np.linalg.inv(field.matrix)
        endpoints=transform_points(inverse,[centre,centre+.15*axis])
        self.latest={'valid':self.initialized,'reason':'OK' if self.initialized else 'NEEDS_HEAD_CALIBRATION',
            'world_deg':angle,'relative_deg':wrap_degrees(angle-self.zero_world) if self.initialized else None,
            'x_m':float(centre[0]),'y_m':float(centre[1]),'centre_world':centre.tolist(),'direction_world':axis.tolist(),
            'centre_px':endpoints[0].tolist(),'front_px':endpoints[1].tolist(),'axis_ratio':ratio,
            'quality':min(1.,(ratio-1.)/4.),'sample_time':float(stamp),'track_id':track_id,'continuity_id':self.continuity_id,
            'source':'OPTICAL_BODY_AXIS'}
        return dict(self.latest)

    def calibrate(self,point,snapshot,field):
        if not snapshot.get('direction_world') or self.latest is None:
            raise ValueError('当前鱼身方向不清晰，请调整画面后重新标定')
        if snapshot.get('track_id')!=self.track_id or snapshot.get('continuity_id')!=self.continuity_id:
            raise ValueError('标定期间已丢失目标，请重新选择鱼头')
        head=field.transform([point])[0];centre=np.asarray(snapshot['centre_world'])
        delta=head-centre
        if np.linalg.norm(delta)<.02:raise ValueError('请点击鱼头前端，与身体中心保持距离')
        sign=1 if delta@np.asarray(snapshot['direction_world'])>=0 else -1
        if sign<0:
            self.direction=-self.direction
            self.latest['world_deg']=wrap_degrees(self.latest['world_deg']+180.)
            self.latest['direction_world']=self.direction.tolist()
            self.latest['front_px']=transform_points(np.linalg.inv(field.matrix),[
                np.asarray(self.latest['centre_world'])+.15*self.direction])[0].tolist()
        self.zero_world=wrap_degrees(snapshot['world_deg']+(180. if sign<0 else 0.))
        self.initialized=True
        self.latest.update(valid=True,reason='OK',relative_deg=wrap_degrees(self.latest['world_deg']-self.zero_world))
        return self.zero_world


SAMPLE_FIELDS=('utc','source','replay','frame','video_s','camera_receipt_time','camera_sample_time',
    'pool_calibration_id','head_calibration_id','vision_valid','vision_reason','vision_world_deg','vision_relative_deg',
    'x_m','y_m','position_valid','position_source','axis_quality','detector_confidence','tracking_source','imu_valid','imu_aligned_cw_deg',
    'imu_relative_deg','imu_latest_cw_deg','imu_sample_time','alignment_span_ms','reference_id','reference_aligned',
    'target_relative_deg','vision_error_deg','imu_error_deg','vision_minus_imu_deg',
    'native_target_cw_deg','native_target_active','native_yaw_active','yaw_output_right_pct','yaw_p','yaw_i','yaw_d','surge_pct',
    'tail1_deg','tail2_deg','left_esc_pct','right_esc_pct','uptime_ms','native_telemetry_time','native_source_id','lora_sequence',
    'vision_threshold','speed_mps','detect_ms','optical_ms','preview_ms','processing_ms','yolo_preprocess_ms','yolo_inference_ms','yolo_postprocess_ms',
    'speed_1s_mps','output_sample_time','output_common_pct','output_differential_pct','output_tail_drive_pct',
    'output_tail_bias_pct','output_tail_frequency_hz','output_tail_amplitude_deg',
    'curve_speed_limit_mps','curve_distance_m','curve_radius_m','curve_speed_status','curve_profile_id')


class AngleSamples:
    def __init__(self,folder):
        self.folder=Path(folder);self.folder.mkdir(parents=True,exist_ok=True)
        self.file=None;self.writer=None;self.metadata={};self.last_flush=0.
        self.status={'active':False,'saved':False,'rows':0,'file':'','metadata_file':'','error':''}

    def start(self,metadata):
        if self.file:return
        stem='heading-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f')
        path=self.folder/(stem+'.csv');meta_path=path.with_suffix('.json')
        self.metadata={**metadata,'schema':1,'fields':list(SAMPLE_FIELDS),'calibrations':[]}
        self.status={'active':False,'saved':False,'rows':0,'file':str(path),'metadata_file':str(meta_path),'error':''}
        try:
            self.file=path.open('x',encoding='utf-8-sig',newline='')
            self.writer=csv.DictWriter(self.file,fieldnames=SAMPLE_FIELDS,extrasaction='ignore');self.writer.writeheader()
            self.status['active']=True;self._save_metadata()
        except OSError as error:
            self.status['error']=str(error);self.close();raise
        print('ROBOFISH_VISION_SAMPLES '+json.dumps(self.status,ensure_ascii=False),flush=True)

    def calibration(self,kind,value):
        if not self.file:return
        self.metadata['calibrations'].append({'kind':kind,'value':value,'recorded_at':time.time()})
        try:self._save_metadata()
        except OSError as error:
            self.status['error']=str(error);self.close();raise

    def write(self,row):
        if not self.file:return
        try:
            self.writer.writerow({key:(None if type(value) is float and not math.isfinite(value) else value)
                                  for key,value in row.items()})
            self.status['rows']+=1
            if time.monotonic()-self.last_flush>=1:self.file.flush();self.last_flush=time.monotonic()
        except OSError as error:self.status['error']=str(error);self.close()

    def _save_metadata(self):
        Path(self.status['metadata_file']).write_text(json.dumps({**self.metadata,'status':self.status},ensure_ascii=False,indent=2)+'\n',
                                                    encoding='utf-8',newline='\n')

    def close(self):
        if not self.file:return
        try:
            self.file.flush()
        except OSError as error:self.status['error']=str(error)
        finally:
            try:self.file.close()
            except OSError as error:self.status['error']=str(error)
            self.file=self.writer=None
            self.status.update(active=False,saved=not bool(self.status['error']))
        try:self._save_metadata()
        except OSError as error:self.status.update(error=str(error),saved=False)
        print('ROBOFISH_VISION_SAMPLES '+json.dumps(self.status,ensure_ascii=False),flush=True)
