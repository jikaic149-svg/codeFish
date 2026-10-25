"""Fish identity from YOLO, brief gaps measured with existing optical flow."""
import math
import time
from pathlib import Path

import cv2
import numpy as np

from control_config import CONTROL
from fixed_reference_tracker import ReferenceObservation, ReferenceSource, _clip_roi
from white_black_tracker import WhiteBlackTracker


class YoloBodyTracker(WhiteBlackTracker):
    def __init__(self,model_path,model_factory=None):
        super().__init__()
        self.model_path=str(Path(model_path).resolve())
        started=time.perf_counter()
        if model_factory is None:
            import torch
            from ultralytics import YOLO
            model_factory=YOLO
            self.device=0 if torch.cuda.is_available() else 'cpu'
        else:self.device='cpu'
        self.model=model_factory(self.model_path)
        if self.model.task!='detect' or self.model.names!={0:'fish'}:
            raise ValueError('请选择单类别 fish 的 YOLO 检测模型')
        self.load_seconds=time.perf_counter()-started
        self.confidence=CONTROL['model']['confidence']
        self.imgsz=int(CONTROL['model']['imgsz'])
        self.last_speed={};self.last_predict_ms=0.

    def select(self,frame,roi,timestamp):
        roi=_clip_roi(roi,frame.shape[1],frame.shape[0])
        if roi is None:raise ValueError('请框选完整机器鱼')
        self.reset_motion();self._selection_roi=roi
        observation=self.update(frame,timestamp)
        self._selection_roi=None
        if not observation.valid:raise ValueError('选区内没有可靠的 YOLO 鱼体检测')
        return observation

    def update(self,frame,timestamp):
        now=float(timestamp)
        if now==self._update_time and self._cached_observation is not None:
            return self._cached_observation
        if not math.isfinite(now) or self._update_time is not None and now<self._update_time:
            return ReferenceObservation(now,ReferenceSource.INVALID,None,0.,0.,False,False,False,['STALE_FRAME'])
        if frame is None or frame.ndim!=3 or frame.shape[2]!=3:
            return ReferenceObservation(now,ReferenceSource.INVALID,None,0.,0.,False,False,False,['INVALID_FRAME'])
        gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        if self._flow_gray is not None and self._flow_gray.shape!=gray.shape:self.reset_motion()
        self._update_time=now;self.head_result=self._empty_head()
        flowing=False
        if self._last_detection is not None and now-self._last_detection>self.search_hold_s:
            self._lock_id=None;self._lock_position=None;self._lock_bbox=None;flowing=False
        predict_started=time.perf_counter()
        result=self.model.predict(frame,imgsz=self.imgsz,conf=self.confidence,device=self.device,verbose=False)[0]
        self.last_speed=dict(getattr(result,'speed',{}) or {})
        self.last_predict_ms=(time.perf_counter()-predict_started)*1000.
        self._detection_count+=1
        choices=[]
        for box,confidence in zip(result.boxes.xyxy.cpu().tolist(),result.boxes.conf.cpu().tolist()):
            box=_clip_roi(box,frame.shape[1],frame.shape[0])
            if box is None:continue
            x1,y1,x2,y2=box;centre=np.array([(x1+x2)/2,(y1+y2)/2])
            if self._selection_roi is not None:
                a,b,c,d=self._selection_roi
                if not a<=centre[0]<=c or not b<=centre[1]<=d:continue
            distance=0.
            if self._lock_position is not None and self._lock_bbox is not None:
                a,b,c,d=self._lock_bbox
                distance=float(np.linalg.norm(centre-self._lock_position))/max(25,math.hypot(c-a,d-b))
                if distance>1.2:continue
            choices.append((float(confidence)-.35*distance,box,centre,float(confidence)))
        confirmed=bool(choices)
        if confirmed:
            _,box,centre,confidence=max(choices,key=lambda item:item[0])
            if self._lock_id is None:
                self._identity_counter+=1;self._lock_id=self._identity_counter
            self._lock_position=centre;self._lock_bbox=box
            self._lock_area=(box[2]-box[0])*(box[3]-box[1])
            self._last_detection=now;self._last_visual=now;self._last_colour=now
            self._feature_mask=np.zeros(gray.shape,np.uint8)
            x1,y1,x2,y2=box;dx=round((x2-x1)*.18);dy=round((y2-y1)*.18)
            self._feature_mask[y1+dy:y2-dy,x1+dx:x2-dx]=255
            self._seed_flow(gray,box)
            mode='YOLO_DETECTION'
        else:
            # Detector-confirmed frames need no optical-flow solve. Flow only measures actual gaps.
            flowing=self._lock_id is not None and self._track_flow(gray,now)
            confidence=self.confidence if flowing else 0.
            mode='MEASURED_FLOW' if flowing else 'LOST'
        valid=confirmed or bool(flowing and self._last_detection is not None and now-self._last_detection<=self.search_hold_s)
        if not valid:self._flow_points=None
        point=tuple(float(v) for v in self._lock_position) if valid else None
        metrics={'detector':'YOLO_PT','model':self.model_path,'reference_role':'YOLO_BOX_CENTRE',
                 'track_id':self._lock_id,'tracking_mode':mode,'detection_count':self._detection_count,
                 'flow_count':self._flow_count,'fish_bbox':self._lock_bbox if valid else None,
                 'body_bbox':self._lock_bbox if valid else None,'body_center':point,
                 'bbox':self._lock_bbox if valid else None,'tail_marker_position':point,
                 'tail_marker_bbox':self._lock_bbox if valid else None,'tail_marker_quality':confidence,
                 'detection_age_ms':round((now-self._last_detection)*1000) if self._last_detection is not None else None}
        observation=ReferenceObservation(now,ReferenceSource.YOLO_BODY if valid else ReferenceSource.INVALID,
                    point,confidence,confidence,False,confirmed,valid,
                    [] if valid else ['YOLO_AND_FLOW_LOST'],metrics,self._lock_bbox)
        self._flow_gray=gray;self._flow_time=now;self._cached_observation=observation
        return observation
