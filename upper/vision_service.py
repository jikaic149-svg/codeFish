"""Own the existing VISION process and proxy only its local web interface."""
from collections import deque
import http.client
import json
import os
from pathlib import Path
import secrets
import subprocess
import threading
import time

from tooling import WORKSPACE

TASKS={'MARKER_ROI','HEAD_DIRECTION','POOL_CALIB','POOL_SIZE','PREVIEW','AUTO_START','AUTO_STOP',
       'SAMPLE_START','SAMPLE_STOP','SNAPSHOT','PERSPECTIVE_TEST','EXIT','VIEW','PREVIEW_FPS',
       'CLEAR_PATH','TURN_CALIB','SPEED','RECORD','CLAHE','EXP_AUTO','EXP_DOWN','EXP_UP','DEBUG','BINARY','THRESHOLD'}


def vision_python():
    python=Path(os.environ.get('ROBOFISH_VISION_PYTHON',r'D:\python\venvs\yolo\ultralytics\8.4.118\Scripts\python.exe'))
    if not python.is_file():raise RuntimeError('找不到视觉Python环境')
    return python


class VisionService:
    def __init__(self):
        self.guard=threading.RLock();self.process=None;self.port=None;self.token=None
        self.logs=deque(maxlen=40);self.source='';self.ready=threading.Event()
        self.model=''
        self.perspective_test={}
        self.recording={'active':False,'saved':False,'error':''}
        self.samples={'active':False,'saved':False,'rows':0,'error':''}
        self.reader=None
        self.output=WORKSPACE/'build/vision'

    def configuration(self):
        models=[]
        for folder in ((WORKSPACE/'vision/fish_body_yolo').resolve(),(WORKSPACE/'vision/models').resolve()):
            for path in sorted(folder.glob('*.pt')):
                if path.is_file() and path.resolve().is_relative_to(folder) and path.name!='last.pt':
                    models.append({'path':path.relative_to(WORKSPACE).as_posix(),'name':path.name,'bytes':path.stat().st_size})
        profiles=[]
        for key,label,name in [('head','鱼头颜色','head_color_profile.local.json'),('tail','鱼尾颜色','marker_profile.local.json'),('pool','场地','pool_calibration.local.json'),('turn','转弯','turn_calibration.local.json')]:
            path=WORKSPACE/'vision/configs'/name
            item={'key':key,'label':label,'path':'vision/configs/'+name,'saved':path.is_file()}
            if path.is_file():
                try:
                    if path.stat().st_size>128*1024:raise ValueError('标定文件过大')
                    item['values']=json.loads(path.read_text(encoding='utf8'))
                except (OSError,ValueError) as error:item['error']=str(error)
            profiles.append(item)
        for profile in profiles:
            if profile['key']=='head':profile.update(label='鱼头与 yaw 零位',saved=False,values={},runtime_only=True)
            if profile['key']=='tail':profile.update(label='YOLO 目标选择',saved=False,values={},runtime_only=True)
        default='vision/fish_body_yolo/best.pt' if (WORKSPACE/'vision/fish_body_yolo/best.pt').is_file() else ''
        guidance=json.loads((WORKSPACE/'vision/configs/control.json').read_text(encoding='utf-8'))['guidance']
        return {'models':models,'default_model':default,'model_required':True,'algorithm':'YOLO + 独立视觉角度与轨迹观测',
                'guidance_defaults':guidance,
                'observation_only':False,'automatic_tracking':True,'tracking_transport':'wifi','supported_tasks':sorted(TASKS),'calibration':profiles,'output':str(self.output)}

    def model_path(self,value=None):
        if value in (None,''):value=self.configuration()['default_model']
        if not isinstance(value,str):raise ValueError('请选择工程内的PT模型')
        path=(WORKSPACE/value).resolve()
        allowed=(WORKSPACE/'vision/models').resolve(),(WORKSPACE/'vision/fish_body_yolo').resolve()
        if not any(path.parent==folder for folder in allowed) or path.suffix.lower()!='.pt' or not path.is_file():
            raise ValueError('请选择 vision 内已交付的 .pt 模型')
        return path

    def cameras(self):
        with self.guard:
            if self.process and self.process.poll() is None:
                raise RuntimeError('请先停止视觉，再重新检测摄像头')
            try:
                result=subprocess.run([str(vision_python()),'-B',str(WORKSPACE/'vision/camera_sources.py')],
                    capture_output=True,text=True,encoding='utf-8',timeout=25,
                    creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            except subprocess.TimeoutExpired as exc:
                raise RuntimeError('摄像头检测超时，请检查设备占用后重新检测') from exc
            if result.returncode:raise RuntimeError('无法检测摄像头，请检查视觉环境或设备占用')
            return json.loads(result.stdout)

    def open_output(self):
        folder=self.output;folder.mkdir(parents=True,exist_ok=True)
        if not hasattr(os,'startfile'):raise RuntimeError('当前系统不支持直接打开文件夹')
        os.startfile(str(folder))
        return {'opened':True,'path':str(folder)}

    def start(self,source,parent_token,parent_port,model=None):
        model=self.model_path(model)
        with self.guard:
            if type(source) is int and 0<=source<=9:argument=str(source)
            elif source=='sample':
                configured=os.environ.get('ROBOFISH_VISION_SAMPLES')
                if configured:sample=next(iter(sorted(Path(configured).glob('*.mp4'))),None)
                else:sample=WORKSPACE.parent/'屏幕录制 2026-10-18 115851.mp4'
                if not sample or not sample.is_file():raise RuntimeError('指定的鱼体训练录像不存在')
                argument=str(sample)
            else:raise ValueError('无效视觉源')
            if self.process and self.process.poll() is None:
                if argument!=self.source or (str(model) if model else '')!=self.model:raise RuntimeError('请先停止视觉，再应用新的相机或模型参数')
                return self.state()
            python=vision_python()
            output=self.output;output.mkdir(parents=True,exist_ok=True)
            for name in ('ultralytics','matplotlib'):(output/name).mkdir(exist_ok=True)
            self.token=secrets.token_hex(24);self.port=None;self.ready.clear();self.logs.clear();self.source=argument
            self.model=str(model) if model else ''
            self.perspective_test={}
            self.recording={'active':False,'saved':False,'error':''}
            self.samples={'active':False,'saved':False,'rows':0,'error':''}
            environment={**os.environ,'ROBOFISH_PARENT_TOKEN':parent_token,'ROBOFISH_PARENT_PORT':str(parent_port),
                'ROBOFISH_VISION_TOKEN':self.token,'ROBOFISH_VISION_OUTPUT':str(output),
                'YOLO_CONFIG_DIR':str(output/'ultralytics'),'MPLCONFIGDIR':str(output/'matplotlib'),
                'PYTHONIOENCODING':'utf-8','PYTHONDONTWRITEBYTECODE':'1'}
            command=[str(python),'-B','-u',str(WORKSPACE/'vision/observer_runtime.py'),'--source',argument]
            if model:command.extend(['--model',str(model)])
            self.process=subprocess.Popen(command,cwd=WORKSPACE/'vision',env=environment,
                stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace',
                creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            self.reader=threading.Thread(target=self._reader,args=(self.process,),daemon=True)
            self.reader.start()
        self.ready.wait(3)
        return self.state()

    def _reader(self,process):
        fields={'ROBOFISH_VISION_RECORDING':'recording','ROBOFISH_PERSPECTIVE_TEST':'perspective_test',
                'ROBOFISH_VISION_SAMPLES':'samples'}
        for line in process.stdout:
            with self.guard:
                if process is not self.process:continue
                prefix,_,payload=line.partition(' ')
                if prefix not in fields and prefix!='ROBOFISH_VISION_READY':
                    self.logs.append(line.rstrip());continue
                try:
                    value=json.loads(payload)
                    if not isinstance(value,dict):raise ValueError('invalid vision status')
                    if prefix=='ROBOFISH_VISION_READY':
                        if type(value.get('port')) is not int or not 1<=value['port']<=65535:
                            raise ValueError('invalid vision port')
                        self.port=value['port'];self.ready.set()
                    else:setattr(self,fields[prefix],value)
                except (ValueError,TypeError,AttributeError):
                    self.logs.append('视觉状态行无效，已忽略')

    def call(self,path,body=None,binary=False,*,process=None):
        with self.guard:
            if process is not None and process is not self.process:
                raise RuntimeError('视觉进程已变化')
            if not self.process or self.process.poll() is not None or not self.port:
                raise RuntimeError('视觉服务未启动或已退出：'+next(iter(reversed(self.logs)),'暂无日志'))
            port,token=self.port,self.token
        connection=http.client.HTTPConnection('127.0.0.1',port,timeout=2)
        try:
            connection.request('POST' if body is not None else 'GET',path,json.dumps(body) if body is not None else None,
                {'Authorization':'Bearer '+token,'Content-Type':'application/json'})
            response=connection.getresponse();data=response.read(4*1024*1024+1)
            if len(data)>4*1024*1024:raise RuntimeError('视觉响应过大')
            if response.status not in (200,202):
                try:message=json.loads(data).get('error')
                except ValueError:message=None
                raise RuntimeError(message or '视觉接口未就绪')
            return data if binary else json.loads(data)
        finally:connection.close()

    def _snapshot(self):
        running=bool(self.process and self.process.poll() is None)
        result={'running':running,'ready':False,'source':self.source,'model':self.model,
                'perspective_test':dict(self.perspective_test),'recording':dict(self.recording),
                'samples':dict(self.samples),'observation_only':True,
                'logs':list(self.logs),'process_id':self.process.pid if running else None}
        if not running and result['recording'].get('active'):
            result['recording'].update(active=False,saved=False,error='视觉进程已退出，录像尚未确认保存')
        if not running and result['samples'].get('active'):
            result['samples'].update(active=False,saved=False,error='视觉进程异常退出，请从输出目录检查采样文件')
        return result

    def state(self):
        with self.guard:
            process=self.process;result=self._snapshot();port=self.port
        if result['running'] and port:
            try:
                remote=self.call('/state',process=process)
                with self.guard:
                    if process is self.process:result.update(remote)
            except (RuntimeError,OSError,http.client.HTTPException):pass
        with self.guard:
            return result if process is self.process else self._snapshot()

    def stream(self,kind):
        with self.guard:
            if kind not in ('raw','result') or not self.port or not self.process or self.process.poll() is not None:
                raise RuntimeError('视觉流未就绪')
            port,token=self.port,self.token
        connection=http.client.HTTPConnection('127.0.0.1',port,timeout=3)
        connection.request('GET','/stream/'+kind,headers={'Authorization':'Bearer '+token})
        response=connection.getresponse()
        if response.status!=200:
            connection.close();raise RuntimeError('视觉流未就绪')
        return connection,response

    def action(self,body):
        if not isinstance(body,dict) or (body.get('task') not in TASKS and body.get('kind') not in ('down','move','up','right')):
            raise ValueError('未映射的视觉操作')
        return self.call('/action',body)

    def transform(self,body):
        return self.call('/transform',body)

    def test_file(self,kind):
        if kind not in ('image','json'):raise ValueError('仅支持测试图和标定 JSON')
        state=self.state();pool=state.get('result',{}).get('pool',{})
        test=pool.get('test') or self.perspective_test
        if kind=='json' and not test.get('data_file') and pool.get('perspective'):
            data=json.dumps(pool['perspective'],ensure_ascii=False,indent=2)+'\n'
            return data.encode('utf-8'),'application/json; charset=utf-8','pool-perspective-calibration.json'
        path=Path(test.get('image_file' if kind=='image' else 'data_file','')).resolve()
        if not path.is_relative_to(self.output.resolve()) or not path.is_file():raise ValueError('请先完成场地四点标定')
        if path.stat().st_size>16*1024*1024:raise ValueError('测试文件过大，请从输出目录查看')
        return path.read_bytes(),('image/png' if kind=='image' else 'application/json; charset=utf-8'),path.name

    def export_samples(self,kind):
        if kind not in ('csv','json'):raise ValueError('仅支持采样 CSV 或元数据 JSON')
        samples=self.state().get('samples') or {}
        if samples.get('active'):raise ValueError('请先停止采样，再导出完整文件')
        if not samples.get('saved'):raise ValueError(samples.get('error') or '尚无已保存的采样文件')
        path=Path(samples.get('file' if kind=='csv' else 'metadata_file','')).resolve()
        if not path.is_relative_to(self.output.resolve()) or not path.is_file():raise ValueError('采样文件不在输出目录中')
        if path.stat().st_size>128*1024*1024:raise ValueError('采样文件较大，请从输出目录查看')
        return path.read_bytes(),('text/csv; charset=utf-8' if kind=='csv' else 'application/json; charset=utf-8'),path.name

    def stop(self):
        with self.guard:process,reader=self.process,self.reader
        if process and process.poll() is None:
            try:self.call('/action',{'task':'EXIT'},process=process)
            except (OSError,RuntimeError,http.client.HTTPException):pass
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                if process.poll() is None:process.terminate()
                process.wait(timeout=3)
        if reader and reader.is_alive():reader.join(timeout=1)
        with self.guard:
            if self.process is process:self.port=None
        return self.state()


VISION=VisionService()
