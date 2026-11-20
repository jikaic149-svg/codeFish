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
from control_config import CONTROL
from video_recorder import recording_status
from pool_perspective import PoolPerspective
import numpy as np
from flask import Flask,Response,request
from werkzeug.serving import make_server
from werkzeug.serving import WSGIRequestHandler

TASKS={'AUTO_START','AUTO_STOP','CLEAR_PATH','MARKER_ROI','HEAD_DIRECTION','POOL_CALIB','TURN_CALIB',
       'RECORD','SNAPSHOT','CLAHE','EXP_DOWN','EXP_UP','DEBUG','BINARY','EXIT','POOL_SIZE'}


class WebBridge:
    def __init__(self,*,observer=False):
        self.observer=observer;self.metadata={};self.control_enabled=False
        self.guard=threading.Lock();self.events=queue.Queue(maxsize=128)
        self.frames={};self.frame_time=0.;self.width=self.height=0
        self.result={};self.last_task=None;self.active_tasks=[];self.closed=threading.Event()
        self.cruise_speed_mps = CONTROL['guidance']['cruise_speed_mps']
        self.yaw_sample={'valid':False,'yaw_cw_deg':None,'sample_time':None}
        self.yaw_history=deque(maxlen=128)
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
                'cruise_speed_mps': self.cruise_speed_mps,'client_revision':'yolo-s3-heading-1',
                'heading_model':'calibrated_imu_yaw','control_mode':3,'yaw_owner':'S3','recording':recording_status(),**self.metadata}
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
            supported=(TASKS|{'SPEED','PREVIEW','VIEW','PREVIEW_FPS','THRESHOLD','PERSPECTIVE_TEST','SAMPLE_START','SAMPLE_STOP'} if self.observer else TASKS|{'SPEED'})
            if body.get('task') not in supported and body.get('kind') not in ('down','move','up','right'):
                return {'error':'unsupported action'},400
            if body.get('task') == 'SPEED' and (type(body.get('value')) not in (int, float)
                    or not math.isfinite(body['value']) or body['value'] < 0):
                return {'error':'speed must be a finite nonnegative value in m/s'},400
            if body.get('task')=='VIEW' and body.get('value') not in ('raw','perspective'):return {'error':'invalid view'},400
            if body.get('task')=='PREVIEW_FPS' and (type(body.get('value')) is not int or body['value'] not in (3,10,15,30)):
                return {'error':'preview FPS must be 3, 10, 15 or 30'},400
            if body.get('task')=='THRESHOLD' and (type(body.get('value')) is not int or not 0<=body['value']<=255):
                return {'error':'threshold must be 0..255'},400
            if 'space' in body and body['space'] not in ('image','field'):return {'error':'invalid coordinate space'},400
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
        @self.app.post('/transform')
        def transform():
            body=request.get_json(silent=True)
            if not isinstance(body,dict) or not isinstance(body.get('points'),list) or not 1<=len(body['points'])<=256:
                return {'error':'请输入 1～256 个原始图像坐标点'},400
            with self.guard:calibration=dict(self.result.get('pool',{}).get('perspective') or {})
            if not calibration:return {'error':'请先完成场地四点标定'},409
            try:
                pool=PoolPerspective(calibration['physical_width_m'],calibration['physical_height_m'])
                pool.matrix=np.asarray(calibration['image_to_field'],dtype=np.float64)
                return {'points_m':pool.transform(body['points']).tolist(),
                        'camera_source':calibration['camera_source'],
                        'frame_width':calibration['frame_width'],'frame_height':calibration['frame_height']}
            except (TypeError,ValueError,cv2.error) as error:return {'error':str(error)},400

    def start(self,*,poll_telemetry=True):
        if hasattr(self,'server'):return
        class QuietRequest(WSGIRequestHandler):
            def log_request(self,*args,**kwargs):pass
        self.server=make_server('127.0.0.1',0,self.app,threaded=True,request_handler=QuietRequest)
        threading.Thread(target=self.server.serve_forever,daemon=True).start()
        if poll_telemetry:threading.Thread(target=self._telemetry_loop,daemon=True,name='VisionYawTelemetry').start()
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
            if isinstance(value,np.ndarray):return clean(value.tolist())
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
            connection=http.client.HTTPConnection('127.0.0.1',int(os.environ.get('ROBOFISH_PARENT_PORT','8766')),timeout=2)
            try:
                connection.request('GET','/api/vision/telemetry')
                response=connection.getresponse();value=json.loads(response.read(65536))
                if response.status!=200 or not isinstance(value,dict):value=None
            except (OSError,ValueError,http.client.HTTPException):value=None
            finally:connection.close()
            with self.guard:
                # A failed local poll is not a new sensor sample. Retain its
                # original timestamp: consumers still enforce the 350 ms age.
                # An explicit invalid sensor result must replace the old value.
                if value is not None:self.yaw_sample=value
                if value is not None and value.get('valid') is True and (not self.yaw_history or
                        value.get('sample_time')!=self.yaw_history[-1].get('sample_time')):
                    self.yaw_history.append(dict(value))
            self.closed.wait(.05)

    def get_yaw_sample(self):
        with self.guard:return dict(self.yaw_sample)

    def get_yaw_sample_at(self,timestamp):
        """Interpolate recorded IMU yaw at a frozen image's exposure time."""
        with self.guard:history=list(self.yaw_history)
        before=[s for s in history if type(s.get('sample_time')) in (int,float) and s['sample_time']<=timestamp]
        after=[s for s in history if type(s.get('sample_time')) in (int,float) and s['sample_time']>=timestamp]
        if not before or not after:return {'valid':False,'source':'NO_ALIGNED_IMU'}
        a=max(before,key=lambda s:s['sample_time']);b=min(after,key=lambda s:s['sample_time'])
        gap=b['sample_time']-a['sample_time']
        if gap>CONTROL['feedback_max_age_s']:return {'valid':False,'source':'IMU_ALIGNMENT_GAP'}
        if (type(a.get('source_id')) is int and type(b.get('source_id')) is int and
                ((b['source_id']-a['source_id'])&0xffffffff)>=0x80000000):
            return {'valid':False,'source':'IMU_EPOCH_CHANGED'}
        y0,y1=a.get('yaw_cw_deg'),b.get('yaw_cw_deg')
        if any(type(v) not in (int,float) or not math.isfinite(v) for v in (y0,y1)):
            return {'valid':False,'source':'INVALID_ALIGNED_IMU'}
        fraction=(timestamp-a['sample_time'])/gap if gap else 0.
        delta=(y1-y0+180)%360-180
        # Yaw is interpolated at its sensor timestamp; command/reference fields
        # are step values at telemetry capture time and must not come from the future.
        timed=[s for s in history if type(s.get('telemetry_time')) in (int,float)]
        past=[s for s in timed if s['telemetry_time']<=timestamp and timestamp-s['telemetry_time']<=CONTROL['feedback_max_age_s']]
        step=max(past,key=lambda s:s['telemetry_time']) if past else ({} if timed else a)
        return {**a,**{key:step.get(key,{}) for key in ('heading_reference','onboard','actuators','control','allocation')},
                'native_telemetry_time':step.get('telemetry_time'),'native_source_id':step.get('source_id'),
                'valid':True,'yaw_cw_deg':(y0+fraction*delta+180)%360-180,
                'sample_time':timestamp,'alignment':'frame_time_interpolated','alignment_span_s':gap}

    def parent(self,body):
        if self.observer and not self.control_enabled:raise RuntimeError('请先明确启动自动循迹')
        connection=http.client.HTTPConnection('127.0.0.1',int(os.environ.get('ROBOFISH_PARENT_PORT','8765')),timeout=2)
        try:
            if body.get('task')=='prepare':
                # An old backend would switch to HYBRID before replying. Verify
                # its capability with GET before sending any prepare command.
                connection.request('GET','/api/health')
                response=connection.getresponse();health=json.loads(response.read(65536))
                if response.status!=200 or health.get('vision_control_mode')!=3 or health.get('vision_client_revision')!='yolo-s3-heading-1':
                    raise RuntimeError('后台仍是旧版本：请用 start.cmd --restart 重启上位机。未发送模式切换或循迹启动')
                body={**body,'client_revision':'yolo-s3-heading-1'}
            connection.request('POST','/api/vision/control',json.dumps(body),
                {'Content-Type':'application/json','Origin':'http://127.0.0.1:'+os.environ.get('ROBOFISH_PARENT_PORT','8765'),
                 'X-AmphiLink-Token':self.parent_token})
            response=connection.getresponse();value=json.loads(response.read(65536))
            if response.status!=200:
                error=RuntimeError(value.get('error','视觉目标未发送'));error.code=value.get('code')
                raise error
            return value
        finally:connection.close()


class WorkstationComm:
    """PC owns visual guidance/speed; each yaw setpoint is executed on S3."""
    def __init__(self,bridge,*,start_sender=True,**unused):
        self.bridge=bridge;self._udp_lock=threading.Lock();self._wire_lock=threading.Lock()
        self._udp_latest=None;self._udp_latest_t=0.;self._stop=threading.Event()
        self.session='vision-'+uuid.uuid4().hex;self.sequence=0;self.last_error='';self.last_ack=0.
        self.last_error_code=None;self.last_round_trip_ms=None
        self.last_stop_reason='not_started';self.sent_targets=0;self.last_target_at=None
        self.last_stop_at=None;self.tracking_started_at=None
        self.pause_reason='';self.paused_at=None;self._pause_stop_sent=False;self._heading_unavailable_since=None
        self.pause_count=0;self.last_pause_reason='';self.last_pause_at=None
        self.speed_loop=VisualSpeedLoop(**CONTROL['speed'],max_age_s=CONTROL['feedback_max_age_s'])
        self.authorized=False;self.vision_seq=0;self.sender=None
        if start_sender:
            self.sender=threading.Thread(target=self._send,daemon=True);self.sender.start()

    def _call(self,task,payload=None,*,require_authorized=False):
        with self._wire_lock:
            if (task=='heading' or require_authorized) and not self.authorized:return None
            self.sequence+=1
            body={'task':task,'session':self.session,'sequence':self.sequence,**(payload or {})}
            started=time.monotonic()
            try:
                result=self.bridge.parent(body);self.last_error='';self.last_error_code=None;self.last_ack=time.monotonic()
                self.last_round_trip_ms=(self.last_ack-started)*1000
                if task=='heading':
                    self.sent_targets+=1;self.last_target_at=time.time();self._heading_unavailable_since=None
                return result
            except Exception as error:
                if str(error)!=self.last_error:print('[视觉控制] '+str(error),flush=True)
                self.last_error=str(error);self.last_error_code=getattr(error,'code',None);return None

    def send_latest(self):
        with self._udp_lock:
            target=self._udp_latest;updated=self._udp_latest_t
            paused_at=self.paused_at;pause_reason=self.pause_reason;pause_stop_sent=self._pause_stop_sent
        if not self.authorized:return
        if paused_at is not None:
            if time.monotonic()-paused_at>.25:self.stop_now(pause_reason);return
            if pause_stop_sent:return
            result=self._call('stop',require_authorized=True)
            if result is not None:
                with self._udp_lock:
                    if self.paused_at==paused_at:self._pause_stop_sent=True
                return
        elif target is None:return
        elif time.monotonic()-updated>.25:self.stop_now('observation_expired');return
        elif target['brake']:
            result=self._call('stop',require_authorized=True)
        else:
            result=self._call('heading',{'heading_cw_deg':target['heading_cw_deg'],'surge_pct':target['power_percent']})
        if result is None and self.authorized:
            reason=self.last_error_code or self.last_error or 'send_failed'
            if self.last_error_code=='heading_unavailable':self.pause_tracking(reason)
            elif self.last_error_code in ('vision_session_changed','control_session_changed','lora_primary_active'):self.stop_tracking(reason)
            else:self.stop_now(reason)

    def _send(self):
        while not self._stop.is_set():
            started=time.monotonic();self.send_latest()
            # Include HTTP time in the 100 ms period; never queue missed ticks.
            self._stop.wait(max(0.,.1-(time.monotonic()-started)))

    @property
    def speed_status(self):return dict(self.speed_loop.status)

    @property
    def command_status(self):
        with self._udp_lock:paused_at=self.paused_at;pause_reason=self.pause_reason
        ack_from=self.last_target_at or self.tracking_started_at
        age=max(0.,(time.time()-ack_from)*1000) if ack_from is not None else None
        return {'paused':paused_at is not None,'pause_reason':pause_reason,
                'pause_age_ms':max(0.,(time.monotonic()-paused_at)*1000) if paused_at is not None else None,
                'target_ack_age_ms':age,'ack_delayed':bool(self.authorized and age is not None and age>1000),
                'last_stop_at':self.last_stop_at,'pause_count':self.pause_count,
                'last_pause_reason':self.last_pause_reason,'last_pause_at':self.last_pause_at}

    @property
    def yaw_status(self):
        sample=self.bridge.get_yaw_sample();native=sample.get('onboard') or {}
        def left(key):
            value=native.get(key)
            return -value if type(value) in (int,float) and math.isfinite(value) else None
        return {'controller':'S3','feedback_valid':sample.get('valid') is True and native.get('yaw_active') is True,
                'steering_percent':left('output_right_pct'),'error_deg':left('error_cw_deg'),
                'p_percent':left('p'),'i_percent':left('i'),'d_percent':left('d'),
                'sample_time':sample.get('sample_time'),'period_ms':native.get('period_ms')}

    def process_tracking_error(self,cross_m,along_m,dist_m,speed_mps=None,brake_request=False,
            speed_target_mps=0.,observed_at=None,feedback_valid=False,target_heading_cw_deg=None,
            yaw_observed_at=None,now=None,received_at=None,speed_held=False,**unused):
        if not self.authorized:self.speed_loop.reset('not_authorized');return None
        now=time.time() if now is None else now
        yaw_fresh=(type(yaw_observed_at) in (int,float) and math.isfinite(yaw_observed_at)
                   and 0<=now-yaw_observed_at<=CONTROL['feedback_max_age_s'])
        target_valid=(type(target_heading_cw_deg) in (int,float) and math.isfinite(target_heading_cw_deg)
                      and -180<=target_heading_cw_deg<=180)
        power=self.speed_loop.update(float(speed_target_mps),
            float(speed_mps) if feedback_valid and speed_mps is not None else None,observed_at,
            now=now,brake=brake_request,received_at=received_at)
        stopped=brake_request or speed_target_mps==0
        if not target_valid:
            self.stop_now('invalid_heading_target');return None
        if not stopped and (not self.speed_loop.status['feedback_valid'] or not yaw_fresh):
            reason=self.speed_loop.status['state'] if not self.speed_loop.status['feedback_valid'] else 'stale_imu'
            self.pause_tracking(reason);return None
        with self._udp_lock:paused_at=self.paused_at;pause_reason=self.pause_reason
        if paused_at is not None and time.monotonic()-paused_at>.25:
            self.stop_now(pause_reason);return None
        if stopped:power=0.
        command={'heading_cw_deg':target_heading_cw_deg,'power_percent':power,'brake':bool(stopped)}
        with self._udp_lock:
            if not self.authorized:return None
            self.pause_reason='';self.paused_at=None;self._pause_stop_sent=False
            self._udp_latest=command
            # A repeated speed sample must not buy another sender timeout if the camera freezes.
            anchor=observed_at if received_at is None else received_at
            self._udp_latest_t=time.monotonic()-(now-anchor if speed_held and not stopped else 0.)
        return command

    def pause_tracking(self,reason):
        """Zero outputs for a brief observation gap, preserving path ownership.

        The existing 250 ms observation deadline bounds recovery; repeated bad
        frames never extend it. Manual stop and control takeover revoke ownership.
        """
        with self._udp_lock:
            if not self.authorized:return False
            if self.paused_at is None:
                if reason=='heading_unavailable' and self._heading_unavailable_since is None:
                    self._heading_unavailable_since=time.monotonic()
                self.paused_at=self._heading_unavailable_since if reason=='heading_unavailable' else time.monotonic()
                self.pause_reason=reason;self._pause_stop_sent=False
                self.pause_count+=1;self.last_pause_reason=reason;self.last_pause_at=time.time()
            paused_at=self.paused_at;pause_reason=self.pause_reason
            self._udp_latest=None;self._udp_latest_t=0.
        self.speed_loop.reset(pause_reason)
        if time.monotonic()-paused_at>.25:
            self.stop_now(pause_reason);return False
        return True

    def stop_tracking(self,reason='stopped'):
        self.authorized=False
        self.last_stop_reason=reason
        self.last_stop_at=time.time()
        with self._udp_lock:
            self._udp_latest=None;self._udp_latest_t=0.
            self.pause_reason='';self.paused_at=None;self._pause_stop_sent=False
            self._heading_unavailable_since=None
        self.speed_loop.reset(reason)

    def ensure_bionic_mode(self):
        # Retain the existing UI callback name; the actual mode is now HYBRID.
        self.stop_tracking('preparing');result=self._call('prepare')
        self.authorized=result is not None and result.get('mode')==3 and result.get('ready') is True and result.get('yaw_owner')=='S3'
        if self.authorized:
            self.last_stop_reason='';self.last_target_at=None;self.tracking_started_at=time.time()
        if result is not None and not self.authorized:self.last_error=result.get('message','等待鱼端航向目标接口')
        return self.authorized

    def stop_now(self,reason='user_stop'):
        was_authorized=self.authorized;self.stop_tracking(reason)
        return self._call('stop') is not None if was_authorized else True

    def send_command(self,command):
        if command in ('stop','emergency_stop'):return self.stop_now()

    def get_mcu_hz(self):return 0.

    def close(self):
        self._stop.set()
        if self.sender:self.sender.join(timeout=1)
        self.stop_now()


def run(source,model=None):
    import vision_app as app
    bridge=WebBridge();bridge.start();app.web_bridge=bridge
    app.MJPEGServer=lambda:bridge;app.TabletTCPServer=lambda **kwargs:bridge
    app.RoboFishComm=lambda **kwargs:WorkstationComm(bridge)
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
