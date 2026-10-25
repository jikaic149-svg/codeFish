"""YOLO/optical heading observer. Reads telemetry; has no motion transmitter."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import threading
import time
import uuid

import cv2
import numpy as np

from angle_observation import OpticalBodyAxis,AngleSamples,wrap_degrees
from control_config import CONTROL
from pool_perspective import PoolPerspective,heading_arrow
from web_runtime import WebBridge
from yolo_tracker import YoloBodyTracker

TASKS={'POOL_CALIB','POOL_SIZE','HEAD_DIRECTION','MARKER_ROI','PREVIEW',
       'SAMPLE_START','SAMPLE_STOP','SNAPSHOT','EXIT'}


class LatestCamera:
    def __init__(self,index):
        self.capture=cv2.VideoCapture(index,cv2.CAP_DSHOW)
        if not self.capture.isOpened():raise RuntimeError('无法打开摄像头')
        for key,value in [(cv2.CAP_PROP_FRAME_WIDTH,1280),(cv2.CAP_PROP_FRAME_HEIGHT,720),(cv2.CAP_PROP_FPS,30)]:
            self.capture.set(key,value)
        self.guard=threading.Lock();self.closed=threading.Event();self.latest=None;self.index=0
        self.thread=threading.Thread(target=self._read,daemon=True,name='ObserverCamera');self.thread.start()

    def _read(self):
        while not self.closed.is_set():
            ok,frame=self.capture.read();stamp=time.time()
            if not ok:self.closed.wait(.02);continue
            with self.guard:
                self.index+=1;self.latest=(self.index,stamp,frame)

    def read(self,previous):
        with self.guard:return self.latest if self.latest and self.latest[0]!=previous else None

    def close(self):
        self.closed.set();self.capture.release();self.thread.join(timeout=1)


class ObserverSession:
    def __init__(self,source,model,bridge,*,detector=None,output=None,calibration=None):
        self.source=str(source);self.replay=not self.source.isdigit();self.bridge=bridge
        self.model=str(model);self.tracker=detector or YoloBodyTracker(model)
        self.output=Path(output or os.environ.get('ROBOFISH_VISION_OUTPUT','../build/vision')).resolve()
        self.output.mkdir(parents=True,exist_ok=True)
        configs=Path(os.environ.get('ROBOFISH_VISION_CONFIG',str(Path(__file__).parent/'configs')))
        self.calibration=Path(calibration or configs/'pool_calibration.local.json')
        self.field=PoolPerspective(4.,2.);self.optical=OpticalBodyAxis();self.samples=AngleSamples(self.output)
        self.pool_id='';self.head={};self.frame=None;self.pose={};self.detection=None
        self.index=0;self.received_at=0.;self.sample_time=None;self.track_time=0.;self.video_s=None
        self.preview=True;self.frozen=None;self.interaction=None;self.points=[];self.roi_start=None
        self.preview_clock=0.;self.preview_ms=0.;self.detect_ms=self.optical_ms=self.processing_ms=0.
        self.preview_signature=None;self.roi_end=None
        self.dropped=0;self.eof=False;self.test={};self.status='视觉只负责观测、标定和 CSV 采样；目标角度在操控页发送'
        self.pending_size=(4.,2.)
        self.sampled_frame=None
        self.camera_latency_s=CONTROL.get('camera_latency_s',.30)

    def accept_frame(self,index,received_at,frame,video_s=None):
        started=time.perf_counter()
        if self.frame is None:
            if self.field.load(self.calibration,self.source,frame.shape):self.pool_id=uuid.uuid4().hex
        elif self.frame.shape[:2]!=frame.shape[:2]:
            self.field=PoolPerspective(self.field.width_m,self.field.height_m)
            self.optical.reset();self.head={};self.pool_id='';self.frozen=None;self.interaction=None
            self.preview=True;self.status='分辨率已变化，请重新标定场地和鱼头'
        self.index=index;self.frame=frame;self.received_at=received_at;self.video_s=video_s
        self.sample_time=None if self.replay else received_at-self.camera_latency_s
        self.track_time=float(video_s) if self.replay else self.sample_time
        self.detection=self.tracker.update(frame,self.track_time)
        self.detect_ms=(time.perf_counter()-started)*1000;optical_started=time.perf_counter()
        metrics=self.detection.metrics
        self.pose=self.optical.update(frame,metrics.get('body_bbox'),self.track_time,self.field,metrics.get('track_id'))
        self.optical_ms=(time.perf_counter()-optical_started)*1000
        self.processing_ms=(time.perf_counter()-started)*1000
        with self.bridge.guard:
            self.bridge.height,self.bridge.width=frame.shape[:2];self.bridge.frame_time=received_at

    def snapshot(self):
        if self.frame is None:raise ValueError('等待第一帧图像')
        aligned=self.bridge.get_yaw_sample_at(self.sample_time) if not self.replay else {'valid':False}
        return {'image':self.frame.copy(),'pose':dict(self.pose),'sample_time':self.sample_time,
                'receipt_time':self.received_at,'index':self.index,'imu':aligned}

    def save_test(self):
        image=self.field.rectify(self.frame)
        path=self.output/('pool-perspective-'+str(time.time_ns())+'.png')
        if not cv2.imwrite(str(path),image):raise OSError('透视测试图保存失败')
        data_path=path.with_suffix('.json')
        data_path.write_text(json.dumps(self.field.describe(self.source),ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n')
        self.test={'image_file':str(path),'data_file':str(data_path),'width':image.shape[1],'height':image.shape[0]}
        print('ROBOFISH_PERSPECTIVE_TEST '+json.dumps(self.test,ensure_ascii=False),flush=True)

    def actions(self):
        for _ in range(32):
            try:event=self.bridge.events.get_nowait()
            except queue.Empty:break
            task=event.get('task',event.get('kind'))
            try:
                if task=='EXIT':self.bridge.close()
                elif task=='PREVIEW':
                    self.preview=not self.preview;self.status='原图预览已'+('打开（3 fps）' if self.preview else '关闭，继续后台识别')
                elif task in ('POOL_CALIB','POOL_SIZE','HEAD_DIRECTION','MARKER_ROI'):
                    if self.interaction==task:self.interaction=None;self.frozen=None;self.status='已取消标定'
                    else:
                        if task=='HEAD_DIRECTION' and not self.pose.get('direction_world'):
                            raise ValueError('请先标定场地，等待清晰的鱼身后再选择鱼头')
                        if task=='POOL_SIZE':self.pending_size=(event['width_m'],event['height_m']);task='POOL_CALIB'
                        elif task=='POOL_CALIB':self.pending_size=(self.field.width_m,self.field.height_m)
                        self.frozen=self.snapshot();self.interaction=task;self.points=[];self.roi_start=None;self.preview=True
                        self.status={'POOL_CALIB':'依次点击左上、右上、右下、左下；冻结的仅为预览，采样仍继续',
                                     'HEAD_DIRECTION':'点击冻结画面里的鱼头，分清头尾并建立视觉零位',
                                     'MARKER_ROI':'在冻结原图中拖框选择完整机器鱼'}[task]
                elif task=='right':self.interaction=None;self.frozen=None;self.status='已取消标定'
                elif task=='SAMPLE_START':
                    if self.eof:raise ValueError('录像已结束，请重新开始回放后采样')
                    digest=hashlib.sha256(Path(self.model).read_bytes()).hexdigest() if Path(self.model).is_file() else None
                    self.samples.start({'source':self.source,'replay':self.replay,'model':self.model,'model_sha256':digest,
                        'observer_only':True,'camera_latency_s':self.camera_latency_s,
                        'timing_source':'camera receipt time minus configured latency; not a hardware exposure timestamp',
                        'note':'Optical body-axis and trajectory are independent observations. Actuator columns are commands, not measured positions.'})
                    self.samples.calibration('pool',{**self.field.describe(self.source),'calibration_id':self.pool_id} if self.field.matrix is not None else {})
                    self.samples.calibration('head',dict(self.head));self.status='正在记录 CSV；视觉不发送运动指令'
                elif task=='SAMPLE_STOP':self.samples.close();self.status='CSV 已保存；S3 保持其原有目标'
                elif task=='SNAPSHOT':self.save_test();self.status='已保存透视测试图，未加入实时显示'
                elif self.interaction and task in ('down','move','up'):
                    point=[event['x']*(self.frame.shape[1]-1),event['y']*(self.frame.shape[0]-1)]
                    if self.interaction=='POOL_CALIB' and task=='down':
                        self.points.append(point);self.status=f'已选择 {len(self.points)}/4 个场地角点'
                        if len(self.points)==4:
                            updated=PoolPerspective(*self.pending_size).calibrate(self.points,self.frame.shape)
                            updated.save(self.calibration,self.source);self.field=updated;self.pool_id=uuid.uuid4().hex
                            self.optical.reset();self.head={};self.pose={};self.test={};self.save_test()
                            self.samples.calibration('pool',{**self.field.describe(self.source),'calibration_id':self.pool_id})
                            self.interaction=None;self.frozen=None
                            self.status='场地透视已更新，请标定鱼头；鱼端目标不变'
                    elif self.interaction=='HEAD_DIRECTION' and task=='down':
                        if not self.pose.get('direction_world'):raise ValueError('当前已丢失清晰鱼身，请取消后重试')
                        zero=self.optical.calibrate(point,self.frozen['pose'],self.field);self.pose=dict(self.optical.latest)
                        reference=self.frozen['imu'];live=self.bridge.get_yaw_sample()
                        old_id,new_id=reference.get('source_id'),live.get('source_id')
                        epoch_ok=not(type(old_id) is int and type(new_id) is int and ((new_id-old_id)&0xffffffff)>=0x80000000)
                        imu_ok=not self.replay and reference.get('valid') is True and live.get('valid') is True and epoch_ok
                        self.head={'calibrated':True,'calibration_id':uuid.uuid4().hex,'visual_zero_world_deg':zero,
                            'angle_deg':zero,'imu_zero_cw_deg':reference.get('yaw_cw_deg') if imu_ok else None,
                            'image_sample_time':self.frozen['sample_time'],'source_id':reference.get('source_id'),
                            'imu_aligned':imu_ok,'head_pixel':point,'body_pixel':self.frozen['pose']['centre_px']}
                        self.samples.calibration('head',dict(self.head));self.interaction=None;self.frozen=None
                        self.preview=False;self.status='鱼头和视觉零位已标定，原图预览已关闭以节省开销'
                    elif self.interaction=='MARKER_ROI':
                        if task=='down':self.roi_start=point;self.roi_end=point
                        elif task=='move' and self.roi_start:self.roi_end=point
                        elif self.roi_start:
                            x,y=self.roi_start;a,b=point
                            self.tracker.select(self.frozen['image'],(min(x,a),min(y,b),max(x,a),max(y,b)),self.track_time+.000001)
                            self.optical.reset();self.head={};self.frozen=None;self.interaction=None
                            self.status='已选择鱼体，请重新标定鱼头'
                else:continue
                state='processed'
            except (ValueError,TypeError,OSError,cv2.error) as error:
                state='error';self.status=str(error)
                if self.interaction=='POOL_CALIB' and len(self.points)>=4:self.points=[]
            with self.bridge.guard:self.bridge.last_task={'id':event['id'],'task':task,'state':state,'status':self.status,'tracking':False}
            self.preview_clock=0.;self.preview_signature=None

    def render_preview(self):
        if self.frame is None:return
        if not self.preview:
            with self.bridge.guard:self.bridge.frames.clear()
            self.preview_ms=0.;self.preview_signature=None;return
        if time.monotonic()-self.preview_clock<1/3:return
        started=time.perf_counter();frozen=self.frozen
        signature=(frozen['index'] if frozen else self.index,self.interaction,len(self.points),self.pool_id,self.head.get('calibration_id'))
        if signature==self.preview_signature:return
        image=frozen['image'].copy() if frozen else self.frame.copy();pose=frozen['pose'] if frozen else self.pose
        if self.field.points is not None:cv2.polylines(image,[self.field.points.astype(np.int32)],True,(0,220,255),1)
        for i,point in enumerate(self.points):
            p=tuple(map(round,point));cv2.circle(image,p,5,(0,220,255),-1);cv2.putText(image,str(i+1),p,0,.8,(0,220,255),2)
        if self.interaction=='MARKER_ROI' and self.roi_start and self.roi_end:
            cv2.rectangle(image,tuple(map(round,self.roi_start)),tuple(map(round,self.roi_end)),(0,180,255),2)
        bbox=self.detection.metrics.get('body_bbox') if self.detection else None
        if bbox and not frozen:
            a,b,c,d=map(round,bbox);cv2.rectangle(image,(a,b),(c,d),(0,220,0),1)
        if pose.get('centre_px'):
            cv2.arrowedLine(image,tuple(map(round,pose['centre_px'])),tuple(map(round,pose['front_px'])),(220,60,220),2)
            cv2.putText(image,'VISION',tuple(map(round,pose['front_px'])),0,.45,(220,60,220),1)
            stamp=frozen['sample_time'] if frozen else self.sample_time
            aligned=self.bridge.get_yaw_sample_at(stamp) if stamp is not None else {}
            reference=aligned.get('heading_reference') or {};native=aligned.get('onboard') or {}
            if aligned.get('valid') and reference.get('valid') and reference.get('calibration_id')==self.head.get('calibration_id'):
                inverse=np.linalg.inv(self.field.matrix);zero=self.head['visual_zero_world_deg']
                angles=[('IMU',wrap_degrees(reference['zero_cw_deg']-aligned['yaw_cw_deg']),(0,220,0))]
                target=native.get('heading_target_cw_deg')
                if type(target) in (int,float):angles.append(('TARGET',wrap_degrees(reference['zero_cw_deg']-target),(0,180,255)))
                for label,relative,color in angles:
                    start,tip=heading_arrow(inverse,pose['centre_world'],math.radians(zero+relative))
                    cv2.arrowedLine(image,tuple(map(round,start)),tuple(map(round,tip)),color,2)
                    cv2.putText(image,label,tuple(map(round,tip)),0,.45,color,1)
        ok,jpeg=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,70])
        if ok:
            raw=jpeg.tobytes()
            with self.bridge.guard:self.bridge.frames={'raw':raw,'result':raw}
            self.preview_signature=signature
        self.preview_clock=time.monotonic();self.preview_ms=(time.perf_counter()-started)*1000

    def publish(self,*,write_sample=False):
        pose=dict(self.pose);fresh=self.replay or self.frame is not None and time.time()-self.received_at<=.6
        if not fresh:pose={'valid':False,'reason':'STALE_FRAME'}
        latest=self.bridge.get_yaw_sample() if not self.replay else {'valid':False,'source':'REPLAY_NOT_PAIRED'}
        latest_time=latest.get('sample_time')
        if latest.get('valid') and (type(latest_time) not in (int,float) or not 0<=time.time()-latest_time<=CONTROL['feedback_max_age_s']):
            latest.update(valid=False,source='IMU_STALE')
        aligned=self.bridge.get_yaw_sample_at(self.sample_time) if self.sample_time is not None else {'valid':False}
        ref=aligned.get('heading_reference') or {};native=aligned.get('onboard') or {};act=aligned.get('actuators') or {}
        reference_aligned=bool(aligned.get('valid') and ref.get('valid') and self.head and ref.get('calibration_id')==self.head.get('calibration_id'))
        target_cw=native.get('heading_target_cw_deg')
        target_relative=wrap_degrees(ref['zero_cw_deg']-target_cw) if ref.get('valid') and type(target_cw) in (int,float) else None
        imu_relative=wrap_degrees(ref['zero_cw_deg']-aligned['yaw_cw_deg']) if ref.get('valid') and aligned.get('valid') else None
        visual_relative=pose.get('relative_deg') if pose.get('valid') else None
        vision_error=wrap_degrees(target_relative-visual_relative) if reference_aligned and target_relative is not None and visual_relative is not None else None
        metrics=self.detection.metrics if self.detection else {};position_source='OPTICAL_BODY_CENTRE'
        position=[pose.get('x_m'),pose.get('y_m')]
        if fresh and position[0] is None and self.field.matrix is not None and self.detection and self.detection.position is not None:
            position=self.field.transform([self.detection.position])[0].tolist();position_source='YOLO_BOX_CENTRE'
        speed=dict(getattr(self.tracker,'last_speed',{}) or {})
        performance={'device':self.tracker.device,'detect_ms':self.detect_ms,'optical_ms':self.optical_ms,
            'preview_ms':self.preview_ms,'processing_ms':self.processing_ms,'yolo_stages_ms':speed,
            'preview_max_fps':3,'dropped_capture_frames':self.dropped}
        body_yaw=None
        old_id,new_id=self.head.get('source_id'),latest.get('source_id')
        epoch_ok=not(type(old_id) is int and type(new_id) is int and ((new_id-old_id)&0xffffffff)>=0x80000000)
        if latest.get('valid') and epoch_ok and self.head.get('imu_zero_cw_deg') is not None:
            body_yaw=wrap_degrees(self.head['imu_zero_cw_deg']-latest['yaw_cw_deg'])
        result={'observer_only':True,'replay':self.replay,'status':self.status,'frame':self.index,'video_s':self.video_s,
            'pool':{'width_m':self.field.width_m,'height_m':self.field.height_m,'calibrating':self.interaction=='POOL_CALIB',
                'selected_points':len(self.points),'perspective':self.field.describe(self.source) if self.field.matrix is not None else None,'test':dict(self.test)},
            'initial_heading':dict(self.head),'optical':pose,'imu':latest,
            'body_yaw':{'usable':body_yaw is not None,'yaw_deg':body_yaw,'source':'IMU_ZERO_ALIGNED'},
            'position':{'x_m':position[0],'y_m':position[1],'source':position_source},
            'vision_error_deg':vision_error,'reference_aligned':reference_aligned,'performance':performance,
            'interaction':self.interaction,'eof':self.eof,'samples':dict(self.samples.status)}
        if write_sample and self.samples.file and self.sampled_frame!=self.index:
            control=aligned.get('control') or {}
            row={'utc':datetime.fromtimestamp(self.received_at,timezone.utc).isoformat(timespec='milliseconds'),
                'source':self.source,'replay':self.replay,'frame':self.index,'video_s':self.video_s,
                'camera_receipt_time':self.received_at,'camera_sample_time':self.sample_time,
                'pool_calibration_id':self.pool_id,'head_calibration_id':self.head.get('calibration_id'),
                'vision_valid':bool(pose.get('valid')),'vision_reason':pose.get('reason'),
                'vision_world_deg':pose.get('world_deg') if pose.get('valid') else None,
                'vision_relative_deg':visual_relative,'x_m':position[0],'y_m':position[1],
                'position_source':position_source,'position_valid':position[0] is not None,
                'axis_quality':pose.get('quality'),'detector_confidence':self.detection.confidence if self.detection else None,
                'tracking_source':metrics.get('tracking_mode'),'imu_valid':bool(aligned.get('valid')),
                'imu_aligned_cw_deg':aligned.get('yaw_cw_deg') if aligned.get('valid') else None,
                'imu_relative_deg':imu_relative,'imu_latest_cw_deg':latest.get('yaw_cw_deg') if latest.get('valid') else None,
                'imu_sample_time':aligned.get('sample_time') if aligned.get('valid') else None,
                'alignment_span_ms':aligned.get('alignment_span_s',0)*1000 if aligned.get('valid') else None,
                'reference_id':ref.get('calibration_id'),'reference_aligned':reference_aligned,
                'target_relative_deg':target_relative,'vision_error_deg':vision_error,
                'imu_error_deg':-native['error_cw_deg'] if type(native.get('error_cw_deg')) in (int,float) else None,
                'vision_minus_imu_deg':wrap_degrees(visual_relative-imu_relative) if reference_aligned and visual_relative is not None and imu_relative is not None else None,
                'native_target_cw_deg':target_cw,'native_target_active':native.get('heading_target_active'),
                'native_yaw_active':native.get('yaw_active'),'yaw_output_right_pct':native.get('output_right_pct'),
                'yaw_p':native.get('p'),'yaw_i':native.get('i'),'yaw_d':native.get('d'),
                'surge_pct':(control.get('axes') or {}).get('surge'),
                'tail1_deg':act.get('tail1'),'tail2_deg':act.get('tail2'),'left_esc_pct':act.get('left_esc'),
                'right_esc_pct':act.get('right_esc'),'uptime_ms':aligned.get('source_id'),
                'native_telemetry_time':aligned.get('native_telemetry_time'),'native_source_id':aligned.get('native_source_id'),
                'lora_sequence':control.get('lora_sequence'),'detect_ms':self.detect_ms,'optical_ms':self.optical_ms,
                'preview_ms':self.preview_ms,'processing_ms':self.processing_ms,
                **{'yolo_'+key+'_ms':speed.get(key) for key in ('preprocess','inference','postprocess')}}
            self.samples.write(row);self.sampled_frame=self.index;result['samples']=dict(self.samples.status)
        self.bridge.send(result)
        with self.bridge.guard:self.bridge.metadata={'ready':self.frame is not None,'observation_only':True,
            'preview_enabled':self.preview,'samples':dict(self.samples.status),'control_mode':None,
            'client_revision':'heading-observer-1','heading_model':'independent_optical_body_axis','recording':{'active':False}}


def run(source,model):
    cv2.setNumThreads(2);bridge=WebBridge(observer=True);bridge.start(poll_telemetry=str(source).isdigit());session=None;capture=None
    try:
        session=ObserverSession(source,model,bridge);replay=session.replay
        capture=cv2.VideoCapture(str(source)) if replay else LatestCamera(int(source))
        if replay and not capture.isOpened():raise RuntimeError('无法读取指定录像')
        fps=(capture.get(cv2.CAP_PROP_FPS) or 30.) if replay else 0.;index=0;next_frame=time.monotonic();last_publish=0.
        while not bridge.closed.is_set():
            session.actions();new=False
            if not session.eof:
                if replay and time.monotonic()>=next_frame:
                    ok,frame=capture.read()
                    if ok:
                        index+=1;session.accept_frame(index,time.time(),frame,(index-1)/fps);new=True
                        next_frame=time.monotonic()+max(0,1/fps-session.processing_ms/1000)
                    else:session.eof=True;session.samples.close();session.status='回放结束；录像仅保存视觉数据，不与当前 IMU 配对'
                elif not replay:
                    latest=capture.read(session.index)
                    if latest:
                        frame_index,stamp,frame=latest;session.dropped+=max(0,frame_index-session.index-1)
                        session.accept_frame(frame_index,stamp,frame);new=True
            session.render_preview()
            if new or time.monotonic()-last_publish>=.1:session.publish(write_sample=new);last_publish=time.monotonic()
            bridge.closed.wait(.002)
    finally:
        if session:session.samples.close()
        if capture is not None:
            if isinstance(capture,LatestCamera):capture.close()
            else:capture.release()
        bridge.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--source',required=True);parser.add_argument('--model',required=True)
    args=parser.parse_args();run(args.source,args.model)
