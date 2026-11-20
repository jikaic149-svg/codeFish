"""Visual metrology and explicit Wi-Fi path following with S3-owned yaw PID."""
import argparse
from collections import deque
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

from angle_observation import OpticalBodyAxis,AngleSamples,SAMPLE_FIELDS,body_mask,wrap_degrees
from control_config import CONTROL
from curve_speed import CurveSpeedPlanner,DisplacementSpeedCheck
from pool_perspective import PoolPerspective,heading_arrow,transform_points
from path_guidance import PathGuidance,VelocityEstimator
from turn_calibration import TurnCalibrationSession
from video_recorder import AsyncVideoRecorder,recording_status
from web_runtime import WebBridge,WorkstationComm
from yolo_tracker import YoloBodyTracker
from white_head_point import HEAD_GAP_S,WhiteHeadPoint,WhiteHeadSpeed

TASKS={'POOL_CALIB','POOL_SIZE','HEAD_DIRECTION','MARKER_ROI','PREVIEW','AUTO_START','AUTO_STOP',
       'SAMPLE_START','SAMPLE_STOP','SNAPSHOT','PERSPECTIVE_TEST','EXIT','VIEW','PREVIEW_FPS',
       'CLEAR_PATH','TURN_CALIB','SPEED','RECORD','CLAHE','EXP_DOWN','EXP_UP','DEBUG','BINARY','THRESHOLD'}


class LatestCamera:
    def __init__(self,index):
        self.capture=cv2.VideoCapture(index,cv2.CAP_DSHOW)
        if not self.capture.isOpened():raise RuntimeError('无法打开摄像头')
        # Keep the camera's default size: forcing 1280x720 can fall back to slow YUY2.
        self.capture.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
        self.capture.set(cv2.CAP_PROP_FPS,30)
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
        self.output=Path(output or os.environ.get('ROBOFISH_VISION_OUTPUT',
            str(Path(__file__).resolve().parents[2]/'build/records/vision'))).resolve()
        self.output.mkdir(parents=True,exist_ok=True)
        configs=Path(os.environ.get('ROBOFISH_VISION_CONFIG',str(Path(__file__).resolve().parents[1]/'configs')))
        self.calibration=Path(calibration or configs/'pool_calibration.local.json')
        self.field=PoolPerspective(3.,1.6);self.optical=OpticalBodyAxis();self.samples=AngleSamples(self.output)
        self.pool_id='';self.head={};self.frame=None;self.pose={};self.detection=None
        self.index=0;self.received_at=0.;self.sample_time=None;self.track_time=0.;self.video_s=None
        self.preview=True;self.frozen=None;self.interaction=None;self.points=[];self.roi_start=None
        self.preview_clock=0.;self.preview_ms=0.;self.detect_ms=self.optical_ms=self.processing_ms=0.
        self.preview_signature=None;self.preview_revision=0;self.roi_end=None
        self.dropped=0;self.eof=False;self.test={};self.status='完成场地和鱼头标定、画好路径后，可明确启动 Wi-Fi 自动循迹'
        self.pending_size=(3.,1.6)
        self.sampled_frame=None
        self.camera_latency_s=CONTROL.get('camera_latency_s',.30)
        self.preview_fps=15;self.view_mode='perspective';self.display_space='image'
        self.debug=False;self.binary=False;self.clahe=False;self.threshold=130
        self.exposure=None;self.capture=None;self.source_fps=30.;self.recorder=None
        self.path_px=[];self.drawing=False;self.history=deque(maxlen=1200)
        self.velocity=VelocityEstimator();self.speed_mps=None
        self.speed_source=CONTROL.get('speed_feedback_source','body_center')
        self.nose=WhiteHeadPoint()
        self.head_speed=WhiteHeadSpeed(CONTROL['speed']['max_speed_mps'],
            CONTROL['tangent_tracking']['fix_slack_m'],HEAD_GAP_S)
        self.motion_id=None;self.nose_px=None;self.speed_time=None
        self.curve_speed_planner=CurveSpeedPlanner(CONTROL['curve_speed'],configs)
        self.trace_validation_enabled=CONTROL['trace_validation']['enabled']
        self.speed_check=DisplacementSpeedCheck() if self.trace_validation_enabled else None
        self.speed_1s_mps=None
        self.guide=PathGuidance.from_config(CONTROL,curve_speed_planner=self.curve_speed_planner)
        self.guidance={};self.tracking=False
        self.comm=None;self.automatic=False
        self.turn=TurnCalibrationSession();self.turn_result={}

    def accept_frame(self,index,received_at,frame,video_s=None):
        started=time.perf_counter()
        if self.frame is None:
            if self.field.load(self.calibration,self.source,frame.shape):
                # Reuse the saved camera corners with the confirmed pool dimensions.
                self.field=PoolPerspective(3.,1.6).calibrate(self.field.points,frame.shape)
                self.pool_id=uuid.uuid4().hex
        elif self.frame.shape[:2]!=frame.shape[:2]:
            self.field=PoolPerspective(self.field.width_m,self.field.height_m)
            self.optical.reset();self.head={};self.pool_id='';self.frozen=None;self.interaction=None;self.clear_path()
            self.preview=True;self.status='分辨率已变化，请重新标定场地和鱼头'
        self.index=index;self.frame=frame;self.received_at=received_at;self.video_s=video_s
        self.sample_time=None if self.replay else received_at-self.camera_latency_s
        self.track_time=float(video_s) if self.replay else self.sample_time
        self.detection=self.tracker.update(frame,self.track_time)
        self.detect_ms=(time.perf_counter()-started)*1000;optical_started=time.perf_counter()
        metrics=self.detection.metrics
        self.pose=self.optical.update(frame,metrics.get('body_bbox'),self.track_time,self.field,metrics.get('track_id'),self.threshold)
        position=self.position()
        identity=(self.pool_id,metrics.get('track_id'))
        if identity!=self.motion_id:
            if self.automatic and self.motion_id is not None and position is not None:
                self.stop_auto('track_changed');self.status='鱼体身份已变，已退出自动循迹'
            self.reset_velocity()
            self.motion_id=identity
        self.nose_px=(self.nose.measure(frame,metrics.get('body_bbox') if self.detection.position is not None else None,self.track_time)
                      if self.speed_source=='white_head_forward' else None)
        if self.speed_check is not None:
            self.speed_1s_mps=self.speed_check.update(position,self.track_time,identity)
        accepted=position is not None and (not self.automatic or self.guide.accepts_position(position,self.track_time))
        if accepted:
            self.velocity.update(position,self.track_time)
        elif position is None:
            self.velocity.reset()
        if self.speed_source=='white_head_forward':
            if position is None:
                self.speed_mps=self.head_speed.update(None,position,self.track_time,identity)
            elif accepted:
                head_world=self.field.transform([self.nose_px])[0] if self.nose_px is not None else None
                self.speed_mps=self.head_speed.update(head_world,position,self.track_time,identity)
        elif position is None:
            self.speed_mps=None
        elif accepted:
            self.speed_mps=float(np.linalg.norm(self.velocity.velocity)) if self.velocity.velocity is not None else None
        if self.speed_mps is None:self.speed_time=None
        elif accepted:
            self.speed_time=self.head_speed.last[2] if self.speed_source=='white_head_forward' else self.track_time
        elif self.speed_time is None or not 0<=self.track_time-self.speed_time<=HEAD_GAP_S:
            self.speed_mps=self.speed_time=None
        if position is not None:
            self.history.append(position.tolist())
            if self.turn.active:self.turn.add(position,self.track_time)
            if self.automatic:
                heading,source=self.control_heading()
                if heading is not None:
                    self.guidance=self.guide.update(position,self.track_time,speed_mps=self.speed_mps,
                        velocity=self.velocity.velocity,body_heading=heading)
                    self.guidance['heading_source']=source
                    if source!='IMU_YAW':
                        confirmed=self.stop_auto('imu_unavailable');self.status='IMU 方向不可用，'+('已全停' if confirmed else '全停未确认')
                    elif self.guidance['arrived']:
                        confirmed=self.stop_auto('arrived');self.status='自动循迹到达终点，已全停' if confirmed else '已到终点，全停未确认'
                    else:
                        target=wrap_degrees(self.head['imu_zero_cw_deg']-(math.degrees(self.guidance['psi_ref_rad'])-self.head['visual_zero_world_deg']))
                        was_paused=bool(getattr(self.comm,'command_status',{}).get('paused'))
                        self.comm.process_tracking_error(0.,self.guidance['along_m'],self.guidance['dist_m'],
                            speed_mps=self.speed_mps,speed_target_mps=self.guidance['speed_target_mps'],
                            observed_at=self.speed_time,
                            received_at=self.speed_time+self.camera_latency_s if self.speed_time is not None else None,
                            feedback_valid=self.speed_mps is not None,
                            speed_held=self.speed_source=='white_head_forward' and self.speed_time!=self.track_time,
                            target_heading_cw_deg=target,yaw_observed_at=getattr(self,'control_yaw_time',None))
                        if getattr(self.comm,'command_status',{}).get('paused'):
                            self.status='反馈短暂缺样，暂停输出并保留当前路径进度'
                        elif was_paused:self.status='新鲜反馈已恢复，继续原路径循迹'
                else:
                    reason=getattr(self,'heading_failure_reason','imu_unavailable')
                    if reason in ('imu_epoch_changed','heading_reference_invalid'):
                        confirmed=self.stop_auto(reason);self.status='鱼头零位已失效，'+('已全停，请重新标定' if confirmed else '全停未确认')
                    elif self.comm.pause_tracking(reason):
                        self.status='IMU 反馈短暂缺样，暂停输出并保留路径；等待新鲜反馈'
                    else:
                        self.automatic=False;self.tracking=False;self.bridge.control_enabled=False
                        self.status='IMU 反馈持续不可用，已退出自动循迹：'+reason
        elif self.automatic:
            confirmed=self.stop_auto('position_lost');self.status='鱼体位置丢失，'+('已全停' if confirmed else '全停未确认')
            self.guidance={}
        self.optical_ms=(time.perf_counter()-optical_started)*1000
        self.processing_ms=(time.perf_counter()-started)*1000
        with self.bridge.guard:
            self.bridge.height,self.bridge.width=frame.shape[:2];self.bridge.frame_time=received_at

    def snapshot(self):
        if self.frame is None:raise ValueError('等待第一帧图像')
        aligned=self.bridge.get_yaw_sample_at(self.sample_time) if not self.replay else {'valid':False}
        return {'image':self.frame.copy(),'pose':dict(self.pose),'sample_time':self.sample_time,
                'point_px':self.detection.position if self.detection else None,'nose_px':self.nose_px,
                'display_space':self.display_space,
                'selection_id':uuid.uuid4().hex,'track_time':self.track_time,
                'receipt_time':self.received_at,'index':self.index,'imu':aligned}

    def position(self):
        """Box centre remains the path/trajectory position, not the nose."""
        if self.field.matrix is not None and self.detection is not None and self.detection.position is not None:
            return self.field.transform([self.detection.position])[0]
        return None

    def control_heading(self):
        # Control uses current IMU feedback. Historical image/IMU interpolation
        # remains in the optical comparison and calibration paths only.
        sample=self.bridge.get_yaw_sample();stamp=sample.get('sample_time');yaw=sample.get('yaw_cw_deg')
        self.control_yaw_time=None;self.heading_failure_reason=''
        old_id,new_id=self.head.get('source_id'),sample.get('source_id')
        if type(old_id) is int and type(new_id) is int and ((new_id-old_id)&0xffffffff)>=0x80000000:
            self.heading_failure_reason='imu_epoch_changed'
        elif self.head.get('imu_zero_cw_deg') is None:self.heading_failure_reason='heading_reference_invalid'
        elif not sample.get('valid'):self.heading_failure_reason='imu_unavailable'
        elif (any(type(v) not in (int,float) or not math.isfinite(v) for v in (stamp,yaw)) or
                not 0<=time.time()-stamp<=CONTROL['feedback_max_age_s']):self.heading_failure_reason='imu_stale'
        if self.heading_failure_reason:
            return None,'UNAVAILABLE'
        self.control_yaw_time=stamp
        angle=math.radians(self.head['visual_zero_world_deg']+wrap_degrees(self.head['imu_zero_cw_deg']-yaw))
        return [math.cos(angle),-math.sin(angle)],'IMU_YAW'

    def reset_velocity(self):
        self.nose=WhiteHeadPoint()
        self.head_speed.reset()
        self.nose_px=self.speed_time=self.speed_mps=None
        self.velocity.reset()

    def clear_path(self):
        self.stop_auto()
        self.path_px=[];self.drawing=False;self.history.clear();self.reset_velocity()
        self.guide.clear();self.guidance={};self.tracking=False;self.turn.cancel()

    def stop_auto(self,reason='user_stop'):
        confirmed=(self.comm.stop_now(reason) if self.comm.authorized else False) if self.automatic and self.comm else True
        self.automatic=False;self.bridge.control_enabled=False
        self.tracking=False
        return confirmed

    def close_recording(self):
        recorder=self.recorder;self.recorder=None
        if recorder is not None:recorder.close()

    def pointer_point(self,event):
        space=event.get('space',self.display_space)
        expected=self.frozen.get('display_space','image') if self.interaction=='HEAD_DIRECTION' else 'image'
        if self.interaction and space!=expected:raise ValueError('请等待当前标定画面加载后再点击')
        if self.interaction and event.get('selection_id') not in (None,self.frozen['selection_id']):
            raise ValueError('标定画面已更新，请在当前视角重新选择')
        x,y=event['x'],event['y']
        if space=='field':
            width,height=self.field_size()
            world=[x*(width-1)*self.field.width_m/width,y*(height-1)*self.field.height_m/height]
            return transform_points(np.linalg.inv(self.field.matrix),[world])[0].tolist()
        return [x*(self.frame.shape[1]-1),y*(self.frame.shape[0]-1)]

    def field_size(self):
        scale=1000/max(self.field.width_m,self.field.height_m)
        return max(1,round(self.field.width_m*scale)),max(1,round(self.field.height_m*scale))

    def save_test(self):
        image=self.field.rectify(self.frame)
        path=self.output/('pool-perspective-'+str(time.time_ns())+'.png')
        if not cv2.imwrite(str(path),image):raise OSError('透视测试图保存失败')
        data_path=path.with_suffix('.json')
        data_path.write_text(json.dumps(self.field.describe(self.source),ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n')
        self.test={'image_file':str(path),'data_file':str(data_path),'width':image.shape[1],'height':image.shape[0]}
        print('ROBOFISH_PERSPECTIVE_TEST '+json.dumps(self.test,ensure_ascii=False),flush=True)

    def actions(self):
        if self.automatic and self.comm and not self.comm.authorized:
            self.automatic=False;self.tracking=False;self.bridge.control_enabled=False
            self.status='自动循迹已停止更新：'+(self.comm.last_error or self.comm.last_stop_reason or '控制权已变化，请重新启动')
        for _ in range(32):
            try:event=self.bridge.events.get_nowait()
            except queue.Empty:break
            task=event.get('task',event.get('kind'))
            if task in ('down','move','up') and event.get('selection_id') and not self.frozen:continue
            try:
                if task=='EXIT':self.stop_auto();self.close_recording();self.bridge.close()
                elif task=='PREVIEW':
                    self.preview=not self.preview;self.status='预览已'+('打开' if self.preview else '关闭，继续后台识别')
                elif task=='VIEW':
                    self.view_mode=event['value'];self.preview=True;self.status='已选择'+('完整原图' if self.view_mode=='raw' else '标定后的透视画面')
                elif task=='PREVIEW_FPS':self.preview_fps=event['value'];self.status=f'预览上限 {self.preview_fps} fps，识别与采样独立运行'
                elif task=='DEBUG':self.debug=not self.debug;self.status='调试叠加已'+('开启' if self.debug else '关闭')
                elif task=='BINARY':self.binary=not self.binary;self.status='鱼体二值预览已'+('开启' if self.binary else '关闭')
                elif task=='CLAHE':self.clahe=not self.clahe;self.status='预览对比度增强已'+('开启' if self.clahe else '关闭')
                elif task=='THRESHOLD':self.threshold=event['value'];self.status=f'鱼体暗部阈值 {self.threshold}'
                elif task in ('EXP_UP','EXP_DOWN'):
                    if self.replay or self.capture is None:raise ValueError('录像不能调节相机曝光，请使用实时摄像头')
                    capture=self.capture.capture if isinstance(self.capture,LatestCamera) else self.capture
                    value=capture.get(cv2.CAP_PROP_EXPOSURE)+(1 if task=='EXP_UP' else -1)
                    value=max(-13.,min(-1.,value));capture.set(cv2.CAP_PROP_AUTO_EXPOSURE,.25)
                    if not capture.set(cv2.CAP_PROP_EXPOSURE,value):raise ValueError('当前摄像头不支持手动曝光')
                    self.exposure=capture.get(cv2.CAP_PROP_EXPOSURE);self.status=f'曝光 {self.exposure:g}'
                elif task=='RECORD':
                    if self.recorder is not None:self.close_recording();self.status='MP4 和同步 CSV 已保存'
                    else:
                        if self.frame is None or self.eof:raise ValueError('请先取得新的视频帧')
                        path=self.output/('vision-'+str(time.time_ns())+'.mp4')
                        self.recorder=AsyncVideoRecorder(path,cv2.VideoWriter_fourcc(*'mp4v'),self.source_fps,
                            (self.frame.shape[1],self.frame.shape[0]),path.with_suffix('.csv'),fields=SAMPLE_FIELDS)
                        self.status='正在录制完整原图及同步 CSV；采样与预览开关独立'
                elif task=='CLEAR_PATH':self.clear_path();self.status='已清除路径和观测轨迹；自动循迹已退出'
                elif task=='SPEED':
                    self.curve_speed_planner.validate_speed(event['value'])
                    self.guide.cruise_speed_mps=event['value']
                    self.status='自动循迹速度目标已更新' if self.automatic else '循迹目标速度已设置，启动自动循迹后执行'
                elif task=='AUTO_START':
                    self.stop_auto()
                    if self.interaction:raise ValueError('请先完成或取消当前标定，再启动循迹')
                    position=self.position();heading,source=self.control_heading()
                    if self.field.matrix is None or len(self.path_px)<2:raise ValueError('请先标定场地，并在画面中拖动绘制路径')
                    if position is None or heading is None:raise ValueError('请先标定鱼头，并等待有效鱼体与方向')
                    if self.speed_source=='white_head_forward' and (self.speed_time is None or
                        not 0<=time.time()-(self.speed_time+self.camera_latency_s)<=HEAD_GAP_S):
                        raise ValueError('请等待白头测速就绪；不要求每帧可见')
                    self.guidance=self.guide.start(self.field.transform(self.path_px),position,self.track_time,heading,
                                                   initial_speed_mps=self.speed_mps)
                    self.guidance['heading_source']=source
                    if self.replay or source!='IMU_YAW' or not self.head.get('imu_aligned'):raise ValueError('自动循迹需要实时相机和已对齐的 IMU 鱼头零位')
                    if self.comm is None:self.comm=WorkstationComm(self.bridge)
                    self.bridge.control_enabled=True
                    if not self.comm.ensure_bionic_mode():
                        self.bridge.control_enabled=False
                        raise RuntimeError(self.comm.last_error or 'Wi-Fi 循迹控制尚未准备，请检查连接')
                    self.automatic=True;self.tracking=True
                    self.status='自动循迹运行中：摆尾＋双桨共同推进，尾偏置导向'
                    if self.guide.path_geometry.get('status')=='unachievable':
                        self.status+='；期望圆角未满足空间约束，保持原绘线'
                elif task=='AUTO_STOP':
                    was_auto=self.automatic;confirmed=self.stop_auto();self.guidance={}
                    self.status=('自动循迹已停止并全停' if confirmed else '自动循迹已停止，全停未确认') if was_auto else '自动循迹未运行'
                elif task=='TURN_CALIB':
                    if self.turn.active:
                        fit=self.turn.finish();self.turn_result={**fit.to_dict(),'mode':'OBSERVED_TURN','turn_strength':None}
                        path=self.output/('turn-observation-'+str(time.time_ns())+'.json')
                        path.write_text(json.dumps(self.turn_result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
                        self.turn_result['file']=str(path)
                        self.status=f'观测转弯半径 {fit.radius_m:.3f} m，已保存；转向由操控页人工执行'
                    else:
                        if self.position() is None:raise ValueError('请先完成场地标定并找到鱼体')
                        self.turn.start(self.position(),self.track_time);self.status='转弯采样中：请人工控制鱼转弯，再点击一次拟合半径'
                elif task in ('POOL_CALIB','POOL_SIZE','HEAD_DIRECTION','MARKER_ROI'):
                    self.stop_auto()
                    if task=='MARKER_ROI':self.view_mode='raw'
                    if self.interaction==task:self.interaction=None;self.frozen=None;self.status='已取消标定'
                    else:
                        if task=='HEAD_DIRECTION' and not self.pose.get('direction_world'):
                            raise ValueError('请先标定场地，等待清晰的鱼身后再选择鱼头')
                        if task=='POOL_SIZE':self.pending_size=(event['width_m'],event['height_m']);task='POOL_CALIB'
                        elif task=='POOL_CALIB':self.pending_size=(self.field.width_m,self.field.height_m)
                        self.frozen=self.snapshot();self.interaction=task;self.points=[];self.roi_start=None;self.preview=True
                        self.status={'POOL_CALIB':'依次点击左上、右上、右下、左下；冻结的仅为预览，采样仍继续',
                                     'HEAD_DIRECTION':'已冻结当前视角，请点击鱼头；再次点击“鱼头方向”取消',
                                     'MARKER_ROI':'在冻结原图中拖框选择完整机器鱼'}[task]
                elif task=='right':self.interaction=None;self.frozen=None;self.drawing=False;self.status='已取消当前选择'
                elif task=='SAMPLE_START':
                    if self.eof:raise ValueError('录像已结束，请重新开始回放后采样')
                    digest=hashlib.sha256(Path(self.model).read_bytes()).hexdigest() if Path(self.model).is_file() else None
                    self.samples.start({'source':self.source,'replay':self.replay,'model':self.model,'model_sha256':digest,
                        'observer_only':True,'camera_latency_s':self.camera_latency_s,'control_config':CONTROL,
                        'timing_source':'camera receipt time minus configured latency; not a hardware exposure timestamp',
                        'note':'Optical body-axis and trajectory are independent observations. Actuator columns are commands, not measured positions.'})
                    self.samples.calibration('pool',{**self.field.describe(self.source),'calibration_id':self.pool_id} if self.field.matrix is not None else {})
                    self.samples.calibration('head',dict(self.head));self.status='正在记录 CSV；视觉不发送运动指令'
                elif task=='SAMPLE_STOP':self.samples.close();self.status='CSV 已保存；当前运动模式继续运行'
                elif task=='PERSPECTIVE_TEST':self.save_test();self.status='透视测试图和矩阵已保存'
                elif task=='SNAPSHOT':
                    image,_=self.compose_preview();path=self.output/('vision-snapshot-'+str(time.time_ns())+'.png')
                    if not cv2.imwrite(str(path),image):raise OSError('截图保存失败')
                    self.status='截图已保存：'+str(path)
                elif self.interaction and task in ('down','move','up'):
                    point=self.pointer_point(event)
                    if self.interaction=='POOL_CALIB' and task=='down':
                        self.points.append(point);self.status=f'已选择 {len(self.points)}/4 个场地角点'
                        if len(self.points)==4:
                            updated=PoolPerspective(*self.pending_size).calibrate(self.points,self.frame.shape)
                            updated.save(self.calibration,self.source);self.field=updated;self.pool_id=uuid.uuid4().hex
                            self.optical.reset();self.head={};self.pose={};self.test={};self.clear_path();self.save_test()
                            self.samples.calibration('pool',{**self.field.describe(self.source),'calibration_id':self.pool_id})
                            self.interaction=None;self.frozen=None
                            self.status='场地透视已更新，请重新标定鱼头'
                    elif self.interaction=='HEAD_DIRECTION' and task=='down':
                        if not self.frozen['pose'].get('direction_world'):raise ValueError('冻结画面中鱼身不清晰，请取消后重试')
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
                        self.status='鱼头和视觉零位已标定，保持当前视角'
                    elif self.interaction=='MARKER_ROI':
                        if task=='down':self.roi_start=point;self.roi_end=point
                        elif task=='move' and self.roi_start:self.roi_end=point
                        elif self.roi_start:
                            x,y=self.roi_start;a,b=point
                            self.tracker.select(self.frozen['image'],(min(x,a),min(y,b),max(x,a),max(y,b)),self.frozen['track_time'])
                            self.reset_velocity()
                            self.optical.reset();self.head={};self.pose={};self.detection=None;self.frozen=None;self.interaction=None
                            self.status='已选择鱼体，保持完整全局画面；等待新帧后标定鱼头'
                elif task in ('down','move','up') and self.frame is not None:
                    point=self.pointer_point(event)
                    if task=='down':self.stop_auto();self.path_px=[point];self.drawing=True;self.tracking=False;self.guidance={}
                    elif self.drawing:
                        if np.linalg.norm(np.asarray(point)-self.path_px[-1])>=4 and len(self.path_px)<1000:self.path_px.append(point)
                        if task=='up':self.drawing=False;self.status=f'已绘制 {len(self.path_px)} 个路径点，可启动自动循迹'
                else:continue
                state='processed'
            except (ValueError,TypeError,OSError,RuntimeError,cv2.error) as error:
                state='error';self.status=str(error)
                if self.interaction=='POOL_CALIB' and len(self.points)>=4:self.points=[]
            with self.bridge.guard:self.bridge.last_task={'id':event['id'],'task':task,'state':state,'status':self.status,'tracking':self.tracking}
            self.preview_clock=0.;self.preview_signature=None

    def raw_overlay(self,frozen=None):
        image=frozen['image'].copy() if frozen else self.frame.copy();pose=frozen['pose'] if frozen else self.pose
        if self.binary and not frozen:image=cv2.cvtColor(body_mask(image,self.threshold),cv2.COLOR_GRAY2BGR)
        elif self.clahe and not frozen:
            lab=cv2.cvtColor(image,cv2.COLOR_BGR2LAB)
            lab[:,:,0]=cv2.createCLAHE(clipLimit=2.,tileGridSize=(8,8)).apply(lab[:,:,0])
            image=cv2.cvtColor(lab,cv2.COLOR_LAB2BGR)
        if self.field.points is not None:cv2.polylines(image,[self.field.points.astype(np.int32)],True,(0,220,255),1)
        displayed_path=self.path_px
        if self.guide.prepared and self.field.matrix is not None:
            displayed_path=transform_points(np.linalg.inv(self.field.matrix),self.guide.path)
        if len(displayed_path)>1:cv2.polylines(image,[np.int32(displayed_path)],False,(255,0,255),2)
        if self.field.matrix is not None and len(self.history)>1:
            trail=transform_points(np.linalg.inv(self.field.matrix),list(self.history))
            cv2.polylines(image,[trail.astype(np.int32)],False,(255,200,0),2)
        if self.field.matrix is not None and self.guidance.get('lookahead_point') is not None:
            target=transform_points(np.linalg.inv(self.field.matrix),[self.guidance['lookahead_point']])[0]
            cv2.circle(image,tuple(map(round,target)),7,(0,160,255),2)
        for i,point in enumerate(self.points):
            p=tuple(map(round,point));cv2.circle(image,p,5,(0,220,255),-1);cv2.putText(image,str(i+1),p,0,.8,(0,220,255),2)
        if self.interaction=='MARKER_ROI' and self.roi_start and self.roi_end:
            cv2.rectangle(image,tuple(map(round,self.roi_start)),tuple(map(round,self.roi_end)),(0,180,255),2)
        bbox=self.detection.metrics.get('body_bbox') if self.detection else None
        if self.debug and bbox and not frozen:
            a,b,c,d=map(round,bbox);cv2.rectangle(image,(a,b),(c,d),(0,220,0),1)
        if pose.get('centre_px'):
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
        if self.debug:
            features=getattr(self.tracker,'_flow_points',None)
            if features is not None and not frozen:
                for point in features.reshape(-1,2):cv2.circle(image,tuple(map(round,point)),2,(0,255,255),-1)
        point=frozen.get('point_px') if frozen else (self.detection.position if self.detection else None)
        if point is not None:
            pixel=tuple(map(round,point))
            cv2.circle(image,pixel,7,(255,255,255),-1)
            cv2.circle(image,pixel,4,(0,0,255),-1)
            cv2.putText(image,'P',(pixel[0]+10,pixel[1]-10),0,.55,(0,0,255),2,cv2.LINE_AA)
        nose=frozen.get('nose_px') if frozen else self.nose_px
        if nose is not None:cv2.circle(image,tuple(map(round,nose)),6,(100,255,70),2,cv2.LINE_AA)
        return image

    def compose_preview(self):
        if self.frame is None:raise ValueError('等待第一帧图像')
        image=self.raw_overlay(self.frozen)
        if self.frozen:
            self.display_space=self.frozen.get('display_space','image') if self.interaction=='HEAD_DIRECTION' else 'image'
        else:self.display_space='field' if self.view_mode=='perspective' and self.field.matrix is not None else 'image'
        if self.display_space=='field':image=self.field.rectify(image)
        if self.debug:
            axis=f"{self.pose['relative_deg']:.1f}" if self.pose.get('valid') else '--'
            speed=f'{self.speed_mps:.3f}' if self.speed_mps is not None else '--'
            lines=[f'frame {self.index}  {self.display_space.upper()}  YOLO {self.detect_ms:.1f} ms',
                f"axis {axis} deg  quality {self.pose.get('quality',0):.2f}  threshold {self.threshold}",
                f"speed {speed} m/s  path {len(self.path_px)}  {'AUTO WIFI' if self.automatic else 'OBSERVING'}"]
            for index,line in enumerate(lines):
                point=(12,22+index*23);cv2.putText(image,line,point,0,.5,(0,0,0),3,cv2.LINE_AA)
                cv2.putText(image,line,point,0,.5,(255,255,255),1,cv2.LINE_AA)
        return image,(self.frozen['image'] if self.frozen else self.frame)

    def render_preview(self):
        if self.frame is None:return
        if not self.preview:
            with self.bridge.guard:self.bridge.frames.clear()
            self.preview_ms=0.;self.preview_signature=None;return
        if time.monotonic()-self.preview_clock<1/self.preview_fps:return
        started=time.perf_counter();frozen=self.frozen
        signature=(frozen['index'] if frozen else self.index,self.interaction,len(self.points),self.pool_id,self.head.get('calibration_id'),
            self.view_mode,self.debug,self.binary,self.clahe,self.threshold,len(self.path_px))
        if signature==self.preview_signature:return
        image,original=self.compose_preview()
        result_ok,result=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,75])
        raw_ok,raw=cv2.imencode('.jpg',original,[cv2.IMWRITE_JPEG_QUALITY,75])
        if result_ok and raw_ok:
            with self.bridge.guard:self.bridge.frames={'raw':raw.tobytes(),'result':result.tobytes()}
            self.preview_revision+=1
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
        metrics=self.detection.metrics if self.detection else {};position_source='YOLO_BOX_CENTRE'
        point=self.position() if fresh else None
        position=point.tolist() if point is not None else [None,None]
        pixel=self.detection.position if fresh and self.detection else None
        speed=dict(getattr(self.tracker,'last_speed',{}) or {})
        performance={'device':self.tracker.device,'detect_ms':self.detect_ms,'optical_ms':self.optical_ms,
            'preview_ms':self.preview_ms,'processing_ms':self.processing_ms,'yolo_stages_ms':speed,
            'preview_max_fps':self.preview_fps,'dropped_capture_frames':self.dropped}
        body_yaw=None
        old_id,new_id=self.head.get('source_id'),latest.get('source_id')
        epoch_ok=not(type(old_id) is int and type(new_id) is int and ((new_id-old_id)&0xffffffff)>=0x80000000)
        if latest.get('valid') and epoch_ok and self.head.get('imu_zero_cw_deg') is not None:
            body_yaw=wrap_degrees(self.head['imu_zero_cw_deg']-latest['yaw_cw_deg'])
        speed_fresh=self.speed_time is not None and (self.replay or
            0<=time.time()-(self.speed_time+self.camera_latency_s)<=HEAD_GAP_S)
        head_state=head_point_source='DISABLED'
        if self.speed_source=='white_head_forward':
            head_state=self.head_speed.state
            head_point_source=self.nose.source if fresh else 'STALE'
            if head_state in ('TRACKING','HELD') and self.automatic and self.guidance.get('position_accepted') is False:head_state='POSITION_REJECTED'
            if not fresh or (self.speed_time is not None and not speed_fresh):head_state='STALE'
        result={'observer_only':not self.automatic,'replay':self.replay,'status':self.status,'frame':self.index,'video_s':self.video_s,
            'camera_receipt_time':self.received_at,'camera_sample_time':self.sample_time,'aligned_imu':aligned,
            'pool':{'width_m':self.field.width_m,'height_m':self.field.height_m,'calibrating':self.interaction=='POOL_CALIB',
                'selected_points':len(self.points),'perspective':self.field.describe(self.source) if self.field.matrix is not None else None,'test':dict(self.test)},
            'initial_heading':dict(self.head),'optical':pose,'imu':latest,
            'body_yaw':{'usable':body_yaw is not None,'yaw_deg':body_yaw,'source':'IMU_ZERO_ALIGNED'},
            'position':{'x_m':position[0],'y_m':position[1],'source':position_source,'valid':point is not None,
                'x_px':float(pixel[0]) if pixel is not None else None,'y_px':float(pixel[1]) if pixel is not None else None,
                'track_id':metrics.get('track_id') if pixel is not None else None,
                'sample_time':self.track_time if pixel is not None else None},
            'vision_error_deg':vision_error,'reference_aligned':reference_aligned,'performance':performance,
            'interaction':self.interaction,'eof':self.eof,'samples':dict(self.samples.status),
            'velocity':{'speed_mps':self.speed_mps if speed_fresh else None,'source':self.speed_source,
                'sample_time':self.speed_time if speed_fresh else None,
                'head_px':self.nose_px if fresh else None,
                'head_state':head_state,'head_point_source':head_point_source},
            'path_pixels':self.path_px,'tracking':{'active':self.tracking,'automatic':self.automatic,'transport':'wifi' if self.automatic else None,
                'command_rtt_ms':self.comm.last_round_trip_ms if self.comm else None,'sent_targets':self.comm.sent_targets if self.comm else 0,
                'last_target_at':self.comm.last_target_at if self.comm else None,'stop_reason':self.comm.last_stop_reason if self.comm else '',
                **(getattr(self.comm,'command_status',{}) if self.comm else {})},
            'speed_control':self.comm.speed_status if self.comm else {},
            'guidance':self.guidance,'turn_calibration':{'active':self.turn.active,'samples':self.turn.sample_count,'result':self.turn_result},
            'debug':{'enabled':self.debug,'binary':self.binary,'clahe':self.clahe,'threshold':self.threshold,'exposure':self.exposure},
            'view_mode':self.view_mode,'display_space':self.display_space}
        curve=self.guidance.get('curve_speed') or {}
        validation={'enabled':self.trace_validation_enabled}
        if self.trace_validation_enabled:
            allocation=aligned.get('allocation') or {}
            validation.update(speed_1s_mps=self.speed_1s_mps if fresh else None,
                common_pct=allocation.get('written_common_pct'),differential_pct=allocation.get('written_differential_pct'),
                tail_drive_pct=allocation.get('tail_drive_pct'),tail_bias_pct=allocation.get('tail_bias_pct'),
                tail_frequency_hz=allocation.get('tail_frequency_hz'),tail_amplitude_deg=allocation.get('tail_amplitude_deg'),
                effective_tail_bias_deg=allocation.get('effective_tail_bias_deg'),effective_tail_amplitude_deg=allocation.get('effective_tail_amplitude_deg'),
                output_sample_time=aligned.get('native_telemetry_time'),curve_speed_limit_mps=curve.get('speed_limit_mps'),
                curve_distance_m=curve.get('distance_to_curve_m'),curve_radius_m=curve.get('curve_radius_m'),
                curve_speed_status=curve.get('status','disabled'),curve_profile_id=curve.get('profile_id'))
        result['validation']=validation
        result['curve_speed']={'enabled':self.curve_speed_planner.requested,'status':self.curve_speed_planner.status,
            'active':self.curve_speed_planner.budget is not None,'error':self.curve_speed_planner.error or None}
        result['planned_path_m']=self.guide.path.tolist() if self.guide.prepared else None
        if write_sample and (self.samples.file or self.recorder is not None) and self.sampled_frame!=self.index:
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
                'vision_threshold':self.threshold,'speed_mps':result['velocity']['speed_mps'],
                'speed_source':self.speed_source,'speed_sample_time':result['velocity']['sample_time'],
                'head_speed_state':result['velocity']['head_state'],
                'head_point_source':result['velocity']['head_point_source'],
                'head_speed_epoch':self.head_speed.epoch,
                'head_signed_mps':self.head_speed.signed_mps if result['velocity']['head_state']=='TRACKING' else None,
                'head_x_px':self.nose_px[0] if fresh and self.nose_px else None,
                'head_y_px':self.nose_px[1] if fresh and self.nose_px else None,
                'lora_sequence':control.get('lora_sequence'),'detect_ms':self.detect_ms,'optical_ms':self.optical_ms,
                'preview_ms':self.preview_ms,'processing_ms':self.processing_ms,
                **{'yolo_'+key+'_ms':speed.get(key) for key in ('preprocess','inference','postprocess')}}
            row.update({key:self.guidance.get(key) for key in ('v_along_mps','v_cross_mps','course_error_deg','sideslip_deg',
                'path_curvature_per_m','curve_heading_feedforward_deg','reference_rate_limited','along_m','seg_index',
                'position_accepted','position_sample_time','signed_cross_track_error_m','speed_target_mps','heading_policy')})
            row.update({key:result['tracking'].get(key) for key in ('paused','pause_reason','pause_count','target_ack_age_ms','stop_reason')})
            if self.trace_validation_enabled:
                row.update({key:validation.get(key) for key in
                    ('speed_1s_mps','output_sample_time','curve_speed_limit_mps','curve_distance_m','curve_radius_m','curve_speed_status','curve_profile_id')})
                row.update({'output_'+key:validation.get(key) for key in
                    ('common_pct','differential_pct','tail_drive_pct','tail_bias_pct','tail_frequency_hz','tail_amplitude_deg',
                     'effective_tail_bias_deg','effective_tail_amplitude_deg')})
            self.samples.write(row)
            if self.recorder is not None:self.recorder.submit(self.raw_overlay(),[row.get(key) for key in SAMPLE_FIELDS])
            self.sampled_frame=self.index;result['samples']=dict(self.samples.status)
        self.bridge.send(result)
        with self.bridge.guard:self.bridge.metadata={'ready':self.frame is not None,'observation_only':not self.automatic,
            'preview_enabled':self.preview,'samples':dict(self.samples.status),'control_mode':3 if self.automatic else None,
            'preview_max_fps':self.preview_fps,'display_space':self.display_space,'view_mode':self.view_mode,
            'preview_revision':self.preview_revision,
            'selection_id':self.frozen['selection_id'] if self.frozen else '',
            'client_revision':'heading-observer-2','heading_model':'independent_optical_body_axis','recording':recording_status()}


def run(source,model):
    cv2.setNumThreads(2);bridge=WebBridge(observer=True);bridge.start(poll_telemetry=str(source).isdigit());session=None;capture=None
    try:
        session=ObserverSession(source,model,bridge);replay=session.replay
        capture=cv2.VideoCapture(str(source)) if replay else LatestCamera(int(source))
        session.capture=capture
        if replay and not capture.isOpened():raise RuntimeError('无法读取指定录像')
        fps=(capture.get(cv2.CAP_PROP_FPS) or 30.) if replay else 0.;session.source_fps=fps if replay else 30.
        index=0;next_frame=time.monotonic();last_publish=0.
        while not bridge.closed.is_set():
            session.actions();new=False
            if not session.eof:
                if replay and time.monotonic()>=next_frame:
                    ok,frame=capture.read()
                    if ok:
                        index+=1;session.accept_frame(index,time.time(),frame,(index-1)/fps);new=True
                        next_frame=time.monotonic()+max(0,1/fps-session.processing_ms/1000)
                    else:session.eof=True;session.samples.close();session.close_recording();session.status='回放结束；录像仅保存视觉数据，不与当前 IMU 配对'
                elif not replay:
                    latest=capture.read(session.index)
                    if latest:
                        frame_index,stamp,frame=latest;session.dropped+=max(0,frame_index-session.index-1)
                        session.accept_frame(frame_index,stamp,frame);new=True
            session.render_preview()
            if new or time.monotonic()-last_publish>=.1:session.publish(write_sample=new);last_publish=time.monotonic()
            bridge.closed.wait(.002)
    finally:
        if session:
            session.stop_auto();session.samples.close();session.close_recording()
            if session.comm:session.comm.close()
        if capture is not None:
            if isinstance(capture,LatestCamera):capture.close()
            else:capture.release()
        bridge.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--source',required=True);parser.add_argument('--model',required=True)
    args=parser.parse_args();run(args.source,args.model)
