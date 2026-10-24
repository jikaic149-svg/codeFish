"""Browser transport for the existing VISION UI; no direct MCU HTTP/UDP."""
from collections import deque
import http.client
import json
import math
import os
from pathlib import Path
import queue
import threading
import time
import uuid

import cv2
from vision_speed import VisualSpeedLoop
from yaw_control import YawLoop
from control_config import CONTROL
from video_recorder import recording_status
import numpy as np
from flask import Flask,Response,request
from werkzeug.serving import make_server
from werkzeug.serving import WSGIRequestHandler

TASKS={'START','STOP','CLEAR_PATH','MARKER_ROI','HEAD_DIRECTION','POOL_CALIB','TURN_CALIB',
       'RECORD','SNAPSHOT','CLAHE','EXP_DOWN','EXP_UP','DEBUG','BINARY','EXIT','POOL_SIZE'}


class WebBridge:
    def __init__(self):
        self.guard=threading.Lock();self.events=queue.Queue(maxsize=128)
        self.frames={};self.frame_time=0.;self.width=self.height=0
        self.result={};self.last_task=None;self.active_tasks=[];self.closed=threading.Event()
        self.cruise_speed_mps = 0.30
        self.yaw_sample={'valid':False,'yaw_cw_deg':None,'sample_time':None}
        self.parent_token=os.environ['ROBOFISH_PARENT_TOKEN'];self.token=os.environ['ROBOFISH_VISION_TOKEN']
        self.app=Flask(__name__)
        @self.app.before_request
        def authenticate():
            if request.headers.get('Authorization')!='Bearer '+self.token:
                return {'error':'unauthorized'},403
        @self.app.get('/state')
        def state():
            with self.guard:return {'ready':'raw' in self.frames,'result_ready':'result' in self.frames,'frame_time':self.frame_time,
                'frame_age_ms':int((time.time()-self.frame_time)*1000) if self.frame_time else None,
                'width':self.width,'height':self.height,'result':self.result,'last_task':self.last_task,
                'cruise_speed_mps': self.cruise_speed_mps,'client_revision':'yolo-imu-cascade-1',
                'heading_model':'calibrated_imu_yaw','control_mode':1,'recording':recording_status()}
        @self.app.get('/frame/<kind>')
        def frame(kind):
            with self.guard:raw=self.frames.get(kind)
            return Response(raw,mimetype='image/jpeg',headers={'Cache-Control':'no-store'}) if raw else ({'error':'no frame'},503)
        @self.app.get('/stream/<kind>')
        def stream(kind):
            if kind not in ('raw','result'):return {'error':'invalid stream'},400
            def generate():
                previous=None
                while not self.closed.is_set():
                    with self.guard:raw=self.frames.get(kind)
                    if raw and raw is not previous:
                        previous=raw
                        yield b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(raw)).encode()+b'\r\n\r\n'+raw+b'\r\n'
                    else:time.sleep(.01)
            return Response(generate(),mimetype='multipart/x-mixed-replace; boundary=frame',headers={'Cache-Control':'no-store'})
        @self.app.post('/action')
        def action():
            body=request.get_json(silent=True)
            if not isinstance(body,dict):return {'error':'invalid action'},400
            if body.get('task') not in TASKS | {'SPEED'} and body.get('kind') not in ('down','move','up','right'):
                return {'error':'unsupported action'},400
            if body.get('task') == 'SPEED' and (type(body.get('value')) not in (int, float)
                    or not math.isfinite(body['value']) or not 0 <= body['value'] <= 0.36):
                return {'error':'speed must be 0..0.36 m/s'},400
            if body.get('task') == 'POOL_SIZE' and any(
                    type(body.get(key)) not in (int, float) or not math.isfinite(body[key])
                    or not 0.05 <= body[key] <= 50 for key in ('width_m', 'height_m')):
                return {'error':'pool dimensions must be measured values in 0.05..50 m'},400
            if 'kind' in body and any(type(body.get(key)) not in (int,float) or not 0<=body[key]<=1 for key in ('x','y')):
                return {'error':'invalid coordinate'},400
            body={**body,'id':uuid.uuid4().hex}
            try:self.events.put_nowait(body)
            except queue.Full:return {'error':'action queue full'},409
            with self.guard:self.last_task={'id':body['id'],'task':body.get('task',body.get('kind')),'state':'queued'}
            return {'id':body['id'],'state':'queued'},202

    def start(self):
        if hasattr(self,'server'):return
        class QuietRequest(WSGIRequestHandler):
            def log_request(self,*args,**kwargs):pass
        self.server=make_server('127.0.0.1',0,self.app,threaded=True,request_handler=QuietRequest)
        threading.Thread(target=self.server.serve_forever,daemon=True).start()
        threading.Thread(target=self._telemetry_loop,daemon=True,name='VisionYawTelemetry').start()
        print('ROBOFISH_VISION_READY '+json.dumps({'port':self.server.server_port}),flush=True)

    def raw(self,frame,timestamp):
        if timestamp<=self.frame_time:return
        ok,jpeg=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,75])
        if ok:
            with self.guard:
                self.frames['raw']=jpeg.tobytes();self.frame_time=timestamp
                self.height,self.width=frame.shape[:2]

    def update(self,frame):
        ok,jpeg=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,75])
        if ok:
            with self.guard:self.frames['result']=jpeg.tobytes()

    def send(self,result):
        # Existing VISION publishes this same structured state to its tablet UI.
        def clean(value):
            if isinstance(value,dict):return {str(k):clean(v) for k,v in value.items()}
            if isinstance(value,(list,tuple)):return [clean(v) for v in value]
            if isinstance(value,np.generic):value=value.item()
            if isinstance(value,float) and not math.isfinite(value):return None
            return value
        with self.guard:self.result=clean(result)

    def get_next_command(self):return None
    def get_latest_trajectory(self):return None
    def get_latest_tracking_error(self):return None
    def get_comm_fps(self):return (0.,0.)

    def dispatch(self,callback,pending):
        for _ in range(32):
            try:event=self.events.get_nowait()
            except queue.Empty:break
            self.active_tasks.append(event)
            if event.get('task') == 'SPEED': self.cruise_speed_mps = float(event['value'])
            elif event.get('task') == 'POOL_SIZE':
                pending.append(('POOL_SIZE', (event['width_m'], event['height_m'])))
            elif 'task' in event:pending.append(event['task'])
            elif self.width and self.height:
                kind=event['kind'];code={'down':cv2.EVENT_LBUTTONDOWN,'move':cv2.EVENT_MOUSEMOVE,
                    'up':cv2.EVENT_LBUTTONUP,'right':cv2.EVENT_RBUTTONDOWN}[kind]
                callback(code,int(event['x']*(self.width-1)),int(event['y']*(self.height-1)),
                         cv2.EVENT_FLAG_LBUTTON if kind in ('down','move') else 0,None)

    def complete(self,status,tracking):
        if self.active_tasks:
            event=self.active_tasks[-1]
            with self.guard:self.last_task={'id':event['id'],'task':event.get('task',event.get('kind')),
                'state':'processed','status':status,'tracking':bool(tracking)}
            self.active_tasks.clear()

    def close(self):self.closed.set()

    def _telemetry_loop(self):
        while not self.closed.is_set():
            connection=http.client.HTTPConnection('127.0.0.1',int(os.environ.get('ROBOFISH_PARENT_PORT','8766')),timeout=.7)
            try:
                connection.request('GET','/api/vision/telemetry')
                response=connection.getresponse();value=json.loads(response.read(65536))
                if response.status!=200:value={'valid':False}
            except (OSError,ValueError,http.client.HTTPException):value={'valid':False}
            finally:connection.close()
            with self.guard:self.yaw_sample=value
            self.closed.wait(.05)

    def get_yaw_sample(self):
        with self.guard:return dict(self.yaw_sample)

    def parent(self,body):
        connection=http.client.HTTPConnection('127.0.0.1',int(os.environ.get('ROBOFISH_PARENT_PORT','8765')),timeout=.7)
        try:
            if body.get('task')=='prepare':
                # An old backend would switch to HYBRID before replying. Verify
                # its capability with GET before sending any prepare command.
                connection.request('GET','/api/health')
                response=connection.getresponse();health=json.loads(response.read(65536))
                if response.status!=200 or health.get('vision_control_mode')!=1 or health.get('vision_client_revision')!='yolo-imu-cascade-1':
                    raise RuntimeError('后台仍是旧版本：请运行 restart.cmd 重启上位机。未发送模式切换或循迹启动')
                body={**body,'client_revision':'yolo-imu-cascade-1'}
            connection.request('POST','/api/vision/control',json.dumps(body),
                {'Content-Type':'application/json','Origin':'http://127.0.0.1:'+os.environ.get('ROBOFISH_PARENT_PORT','8765'),
                 'X-AmphiLink-Token':self.parent_token})
            response=connection.getresponse();value=json.loads(response.read(65536))
            if response.status!=200:raise RuntimeError(value.get('error','视觉目标未发送'))
            return value
        finally:connection.close()


def run(source,model=None):
    import vision_app as app
    bridge=WebBridge();bridge.start();app.web_bridge=bridge
    app.MJPEGServer=lambda:bridge;app.TabletTCPServer=lambda **kwargs:bridge
    original_comm=app.RoboFishComm
    class WorkstationComm(original_comm):
        def __init__(self,**kwargs):
            self.vision_seq=0;self._udp_lock=threading.Lock();self._wire_lock=threading.Lock()
            self._udp_latest=None;self._udp_latest_t=0.;self._stop=threading.Event()
            self.session='vision-'+uuid.uuid4().hex;self.sequence=0;self.last_error='';self.last_ack=0.
            self.speed_loop = VisualSpeedLoop(**CONTROL['speed'],max_age_s=CONTROL['feedback_max_age_s'])
            self.yaw_loop=YawLoop()
            self.authorized = False
            self.sender=threading.Thread(target=self._send,daemon=True);self.sender.start()
        def _call(self,task,axes=None):
            with self._wire_lock:
                self.sequence+=1
                body={'task':task,'session':self.session,'sequence':self.sequence}
                if axes is not None:body['axes']=axes
                try:
                    result=bridge.parent(body);self.last_error='';self.last_ack=time.monotonic();return result
                except Exception as error:
                    if str(error)!=self.last_error:print('[视觉控制] '+str(error),flush=True)
                    self.last_error=str(error);return None
        def _send(self):
            while not self._stop.wait(.05):
                with self._udp_lock:target=self._udp_latest;updated=self._udp_latest_t
                if target is not None and time.monotonic()-updated<=app.VISION_UDP_STALE_S:
                    axes = {'surge': target['power_percent'], 'yaw': -target['steering_percent'], 'heave': 0}
                    if target['brake']: axes = {'surge': 0, 'yaw': 0, 'heave': 0}
                    if self._call('target', axes) is None:
                        self.stop_tracking()
                elif target is not None:
                    self.stop_tracking()
                    self._call('target', {'surge': 0, 'yaw': 0, 'heave': 0})
        @property
        def speed_status(self): return dict(self.speed_loop.status)
        @property
        def yaw_status(self):return dict(self.yaw_loop.status)
        def process_tracking_error(self, cross_m, along_m, dist_m, speed_mps=None,
                brake_request=False, steering_demand=0, speed_target_mps=0,
                observed_at=None, feedback_valid=False, target_yaw_rad=None,body_yaw_rad=None,
                yaw_observed_at=None,curve_feedforward=0.,**unused):
            if not self.authorized:
                self.speed_loop.reset('not_authorized')
                self.yaw_loop.reset('not_authorized')
                return None
            power = self.speed_loop.update(float(speed_target_mps),
                float(speed_mps) if feedback_valid and speed_mps is not None else None,
                observed_at, brake=brake_request)
            stopped = brake_request or not self.speed_loop.status['feedback_valid']
            steering=self.yaw_loop.update(target_yaw_rad,body_yaw_rad,yaw_observed_at,
                brake=stopped,feedforward=100.*curve_feedforward)
            stopped=stopped or not self.yaw_loop.status['feedback_valid']
            if stopped:
                self.speed_loop.reset('waiting_feedback');power=0.
            return self.set_tracking_command(
                steering_demand=0 if stopped else steering/100.,
                speed_target_mps=power * app.UDP_SPEED_REFERENCE_MPS / 100.,
                brake_request=stopped)
        def stop_tracking(self):
            original_comm.stop_tracking(self)
            self.speed_loop.reset()
            self.yaw_loop.reset()
            self.authorized = False
        def ensure_bionic_mode(self):
            self.stop_tracking()
            result=self._call('prepare')
            self.authorized = result is not None and result.get('mode') == 1 and result.get('ready') is True
            if result is not None and not self.authorized:self.last_error=result.get('message','等待鱼端控制配置确认')
            return self.authorized
        def stop_now(self):
            was_authorized=self.authorized
            self.stop_tracking()
            return self._call('target',{'surge':0,'yaw':0,'heave':0}) is not None if was_authorized else True
        def send_command(self,command):
            if command in ('stop','emergency_stop'):return self.stop_now()
        def get_mcu_hz(self):return 0.  # PC model receipts are not MCU radio receipts.
        def close(self):
            self._stop.set();self.sender.join(timeout=1);self.stop_now()
    app.RoboFishComm=WorkstationComm
    app.CAMERA_INDEX=int(source) if str(source).isdigit() else str(source)
    app.REPLAY_ONLY=not str(source).isdigit()
    app.MODEL_PATH=str(model) if model else str(Path(__file__).parent/'fish_body_yolo/best.pt')
    app.RECORD_DIR=os.environ['ROBOFISH_VISION_OUTPUT']
    configs=Path(os.environ.get('ROBOFISH_VISION_CONFIG',str(Path(__file__).parent/'configs')))
    app.WORK_DIR=str(configs)
    app.MARKER_PROFILE_PATH=str(configs/'marker_profile.local.json')
    app.POOL_CALIBRATION_PATH=str(configs/'pool_calibration.local.json')
    app.TURN_CALIBRATION_PATH=str(configs/'turn_calibration.local.json')
    for name in ['namedWindow','resizeWindow','moveWindow','setMouseCallback','imshow','destroyAllWindows']:
        setattr(cv2,name,lambda *args,**kwargs:None)
    cv2.getWindowProperty=lambda *args:0 if bridge.closed.is_set() else 1
    cv2.waitKey=lambda *args:255
    # Replay files at their original frame rate; preserve the camera's original loop.
    if not str(source).isdigit():
        original=app.CameraStream
        class Replay(original):
            def update(self):
                interval=1/max(1,self.reported_fps)
                while not self.stopped:
                    time.sleep(interval)
                    with self.lock:
                        ret,frame=self.cap.read()
                        if not ret:
                            self.ret=False;self.stopped=True;bridge.closed.set();break
                        self.ret,self.frame,self.timestamp=ret,frame,time.time()
        app.CameraStream=Replay
    try:app.main()
    finally:bridge.server.shutdown()


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--source',default='0');parser.add_argument('--model',default=None)
    args=parser.parse_args();run(args.source,args.model)
