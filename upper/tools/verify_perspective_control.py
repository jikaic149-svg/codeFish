"""Projected camera -> head zero -> guidance/speed -> real S3 C++ PID/mixer.

The water response is an explicitly synthetic first-order plant. No camera,
UART, Wi-Fi device, firmware flashing or physical heading experiment is used.
"""
import ctypes
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'upper/src'),str(ROOT/'vision/src')]
import station
from lora_link import LoRaLink
from path_guidance import PathGuidance,VelocityEstimator,compensate_camera_latency
from pool_perspective import PoolPerspective,image_heading,heading_arrow,transform_points
from web_runtime import WorkstationComm
from yaw_heading import CalibratedYaw,wrap_radians
from control_config import CONTROL
import vision_app

OUTPUT=ROOT/'build/checks/host-vision/perspective-control'
COMPILER=Path(os.environ.get('ROBOFISH_CXX','C:/msys64/ucrt64/bin/g++.exe'))


def native_model():
    if not (ROOT/'tests/control/heading_model.cpp').is_file():
        raise RuntimeError('历史固件 tests/ 已按要求移除；请使用 tools/simulate_coupled_tracking.py 进行现有联合仿真。')
    OUTPUT.mkdir(parents=True,exist_ok=True)
    library=OUTPUT/'heading-model.dll'
    files=['tests/control/heading_model.cpp','main/Service/LoRa/LoraProtocol.cpp',
           'main/Mode/Motion/MotionMode.cpp','main/Device/Actuator/TailMotion.cpp',
           'main/Device/Actuator/PropulsionMotion.cpp','main/Device/Actuator/PectoralMotion.cpp']
    subprocess.run([str(COMPILER),'-std=c++17','-O2','-shared','-Wall','-Wextra','-Werror',
                    '-I'+str(ROOT/'main'),*[str(ROOT/file) for file in files],'-o',str(library)],check=True,
                   env={**os.environ,'PATH':str(COMPILER.parent)+os.pathsep+os.environ.get('PATH','')})
    handle=os.add_dll_directory(str(COMPILER.parent))
    model=ctypes.CDLL(str(library));model._dll_directory=handle
    model.rf_reset.argtypes=[];model.rf_reset.restype=None
    model.rf_command.argtypes=[ctypes.c_char_p,ctypes.c_uint];model.rf_command.restype=ctypes.c_int
    model.rf_tick.argtypes=[ctypes.c_uint,ctypes.c_double,ctypes.c_double,ctypes.c_int]
    model.rf_tick.restype=ctypes.POINTER(ctypes.c_double)
    return model


def simulate(model,name,quad,path,initial_angle,*,yaw_gain=.6,require_arrival=True):
    pool=PoolPerspective().calibrate(quad,(776,1384));inverse=np.linalg.inv(pool.matrix)
    model.rf_reset();clock=[100.];millis=[0];wire=[];failures=[]
    class UART:
        def write(self,data):
            packet=data[3:].strip();code=int(packet.split(b',')[4]) if packet.startswith(b'101') else 21
            status=model.rf_command(packet,millis[0]);wire.append((clock[0],code,status))
            if status:failures.append((clock[0],code,status))
            return len(data)
    radio=LoRaLink(clock=lambda:clock[0],sequence_file=OUTPUT/(name+'-sequence.json'))
    radio.serial=UART();radio.host='fixture';radio.address=0x0102;radio.channel=0x20
    position=path[0]+[0,.10];world_yaw=float(initial_angle);speed=0.;rate_cw=0.;zero_cw=175.
    data=np.zeros(16);sample={'valid':False};controls=[]
    class Client:
        def is_robotfish(self,host):return True
        def control_binding(self,host,session=None):return 'fixture-session'
        def robot(self,host):return {'motion':{'mode':3},'attitude_control':{'heading_target_supported':True}}
        def motion_control(self,host,action,session,sequence,**payload):
            payload.pop('client_id',None);controls.append(action)
            return radio.submit('fixture',action,dict(payload,mode=3))
    workstation=station.Workstation();workstation.host='fixture'
    def parent(body):
        if body['task']=='prepare':return workstation.prepare_vision(body['session'],body['sequence'])
        payload={key:body[key] for key in ('heading_cw_deg','surge_pct')} if body['task']=='heading' else {}
        return workstation.vision_command('heading_target' if body['task']=='heading' else 'stop',payload,body['session'],body['sequence'])
    bridge=SimpleNamespace(parent=parent,get_yaw_sample=lambda:sample)
    comm=WorkstationComm(bridge,start_sender=False);heading=CalibratedYaw();velocity=VelocityEstimator()
    body_pixel,head_pixel=transform_points(inverse,[position,position+.15*np.array([math.cos(world_yaw),-math.sin(world_yaw)])])
    first=np.array(model.rf_tick(0,zero_cw,0,1)[:16])
    sample={'valid':True,'yaw_cw_deg':float(first[0]),'sample_time':100.,'source_id':0}
    zero=image_heading(pool.matrix,body_pixel,head_pixel)
    heading.calibrate(zero,sample,100.)
    assert abs(heading.update(sample,100.)['yaw_deg'])<1e-6
    guide=PathGuidance(arrival_radius_m=.20,lookahead_m=CONTROL['guidance']['lookahead_m'],
        min_lookahead_m=CONTROL['guidance']['min_lookahead_m'],k_f=CONTROL['guidance']['cross_track_gain'],
        full_attraction_error_m=CONTROL['guidance']['full_attraction_error_m'],curve_preview_m=CONTROL['guidance']['curve_preview_m'],
        cruise_speed_mps=CONTROL['guidance']['cruise_speed_mps'],min_speed_target_mps=CONTROL['guidance']['min_speed_mps'])
    image_path=transform_points(inverse,path)
    # The operator draws pixels; preparation must recover the same metric path.
    recovered=pool.transform(image_path)
    guide.start(recovered,position,100.,heading.update(sample,100.)['unit_vector'])
    history=[];rows=[];smooth=None;arrived=False;result=None;target_world=zero
    with patch.object(station.device_client,'CLIENT',Client()):
        assert comm.ensure_bionic_mode()
        for tick in range(5000):
            t=tick*.02;clock[0]=100.+t;millis[0]=tick*20
            body_cw=math.degrees(wrap_radians(math.radians(zero_cw)-(world_yaw-initial_angle)))
            data=np.array(model.rf_tick(millis[0],body_cw,rate_cw,1)[:16])
            if tick%5==0:
                sample={'valid':bool(data[4]),'yaw_cw_deg':float(data[0]),'sample_time':clock[0],'source_id':millis[0],
                        'onboard':{'yaw_active':bool(data[5]),'error_cw_deg':float(data[1]),
                                   'output_right_pct':float(data[2]),'p':float(data[15]),'i':float(data[13]),'d':float(data[14])}}
            history.append(position.copy())
            if tick%5==0:
                observed=history[max(0,tick-15)] # 300 ms capture delay.
                pixel=transform_points(inverse,[observed])[0]
                measured=pool.transform([pixel])[0]
                alpha=vision_app.POS_SMOOTHING_ALPHA
                smooth=measured if smooth is None else alpha*measured+(1-alpha)*smooth
                vector=velocity.update(smooth,clock[0])
                actual=heading.update(sample,clock[0])
                control_position=smooth if vector is None else compensate_camera_latency(smooth,vector,
                    vision_app.CAMERA_LATENCY_S,vision_app.CAMERA_LATENCY_MAX_PREDICTION_M)
                result=guide.update(control_position,clock[0],body_heading=actual['unit_vector'],
                                    speed_mps=float(np.linalg.norm(vector)) if vector is not None else 0.)
                target_world=result['psi_ref_rad']
                command=comm.process_tracking_error(0,0,0,speed_mps=float(np.linalg.norm(vector)) if vector is not None else None,
                    speed_target_mps=result['speed_target_mps'],observed_at=clock[0],
                    feedback_valid=vector is not None,target_heading_cw_deg=heading.target_cw_deg(target_world),
                    yaw_observed_at=actual['sample_time'],now=clock[0],brake_request=result['arrived'])
                assert 'steering_percent' not in command
                if result['arrived']:
                    comm.stop_now();radio.step();arrived=True;break
            if tick%3==0:comm.send_latest() # 16.7 Hz PC submission, 2 Hz actual RF.
            radio.step()
            # Synthetic hydrodynamics; all actuator commands above are real C++.
            effort=(data[6]-data[7])*.5
            rate_cw+=.02/.25*(effort*yaw_gain-rate_cw)
            world_yaw=wrap_radians(world_yaw-math.radians(rate_cw)*.02)
            common=max(0.,(data[6]+data[7])*.5)
            speed+=.02/.35*(common*.0036-speed)
            position+=speed*.02*np.array([math.cos(world_yaw),-math.sin(world_yaw)])
            # Reprojecting a heading is a display operation, never an extra yaw correction.
            pixels=heading_arrow(inverse,position,actual['world_yaw_rad'])
            assert abs(wrap_radians(image_heading(pool.matrix,*pixels)-actual['world_yaw_rad']))<1e-6
            native_target=wrap_radians(zero+math.radians(zero_cw-data[3])) if data[12] else zero
            native_body=wrap_radians(zero+math.radians(zero_cw-data[0]))
            rows.append([t,*position,math.degrees(world_yaw),math.degrees(target_world),*data[1:4],data[6],data[7],
                         data[0],math.degrees(native_target),math.degrees(native_body),
                         result['cross_track_m'] if result else 0.])
        comm.close();radio.step()
        stopped=np.array(model.rf_tick(millis[0]+20,body_cw,0,1)[:16])
        assert not stopped[5] and stopped[6]==0 and stopped[7]==0
    array=np.asarray(rows)
    np.savetxt(OUTPUT/(name+'.csv'),array,delimiter=',',header='t,x,y,plant_yaw_world,pc_target_world,error_cw,output_right,target_cw,left_pct,right_pct,imu_cw,s3_target_world,s3_imu_world,guidance_cross_m',comments='')
    if require_arrival:assert arrived,(name,position.tolist(),guide.progress_m,'tail',array[-1].tolist())
    assert not failures,(name,failures[:5])
    assert set(controls)<={'heading_target','stop'},set(controls)
    summary={'case':name,'arrived':arrived,'seconds':round(t,2),'endpoint_m':float(np.linalg.norm(position-path[-1])),
             'max_cross_track_m':float(array[:,-1].max()),'rms_cross_track_m':float(np.sqrt(np.mean(array[:,-1]**2))),
             'rf_heading_frames':sum(code==26 for _,code,_ in wire),'rf_rejections':len(failures),
             'zero_world_deg':math.degrees(zero),'zero_cw_deg':zero_cw,'stop_disarms_native_pid':True,
             'synthetic_yaw_gain_deg_per_s_per_pct':yaw_gain}
    return summary,array


def main():
    model=native_model()
    quads={'rectangle':[[150,120],[1250,120],[1250,700],[150,700]],
           'source_video':[[271,150],[1200,121],[1205,632],[259,606]],
           'strong_perspective':[[360,100],[1060,180],[1310,710],[65,610]]}
    x=np.linspace(.45,3.45,120);angle=np.linspace(0,math.pi,100)
    paths={'straight':(np.array([[.45,1.2],[3.45,1.2]]),math.radians(35)),
           's_curve':(np.column_stack((x,1.+.25*np.sin(np.linspace(0,1.8*math.pi,len(x))))),math.radians(-10)),
           'u_turn':(np.column_stack((2.-.65*np.cos(angle),1.25-.65*np.sin(angle))),math.radians(65))}
    reports=[];arrays={}
    projection_checks={}
    for camera,quad in quads.items():
        pool=PoolPerspective().calibrate(quad,(776,1384));inverse=np.linalg.inv(pool.matrix)
        pixel_errors=[];corrected_errors=[]
        for centre in ([.4,.4],[2,1],[3.6,1.6]):
            for angle in np.linspace(-math.pi+.1,math.pi-.1,25):
                ends=heading_arrow(inverse,centre,angle);delta=ends[1]-ends[0]
                pixel_angle=math.atan2(-delta[1],delta[0])
                pixel_errors.append(abs(math.degrees(wrap_radians(pixel_angle-angle))))
                corrected_errors.append(abs(math.degrees(wrap_radians(image_heading(pool.matrix,*ends)-angle))))
        projection_checks[camera]={'max_uncorrected_pixel_angle_error_deg':max(pixel_errors),
                                   'max_corrected_angle_error_deg':max(corrected_errors)}
    for path_name,(path,initial) in paths.items():
        for camera,quad in quads.items():
            result,data=simulate(model,path_name+'-'+camera,quad,path,initial)
            reports.append(result);arrays[(path_name,camera)]=data
            print(json.dumps(result),flush=True)
        baseline=arrays[(path_name,'rectangle')]
        for camera in ('source_video','strong_perspective'):
            other=arrays[(path_name,camera)];length=min(len(baseline),len(other))
            difference=np.linalg.norm(baseline[:length,1:3]-other[:length,1:3],axis=1).max()
            assert difference<.025,(path_name,camera,difference)
    fig,axes=plt.subplots(3,2,figsize=(12,11))
    for row,(name,(path,initial)) in enumerate(paths.items()):
        axes[row,0].plot(path[:,0],path[:,1],'k--',label='reference')
        for camera in quads:
            data=arrays[(name,camera)];axes[row,0].plot(data[:,1],data[:,2],label=camera)
        axes[row,0].set(xlim=(0,4),ylim=(2,0),title=name+' / world metres',xlabel='x / m',ylabel='y / m')
        axes[row,0].set_aspect('equal');axes[row,0].legend(fontsize=8)
        data=arrays[(name,'strong_perspective')]
        axes[row,1].plot(data[:,0],np.degrees(np.unwrap(np.radians(data[:,12]))),label='IMU heading (simulated)')
        axes[row,1].plot(data[:,0],np.degrees(np.unwrap(np.radians(data[:,11]))),label='S3 accepted target')
        axes[row,1].plot(data[:,0],np.degrees(np.unwrap(np.radians(data[:,4]))),'--',alpha=.35,label='PC reference')
        axes[row,1].set(title='World yaw / deg; production S3 controller',xlabel='time / s');axes[row,1].legend(fontsize=8)
    fig.tight_layout();fig.savefig(OUTPUT/'complete-chain.png',dpi=150);plt.close(fig)
    stress=[]
    path,initial=paths['straight']
    for camera,quad in quads.items():
        result,_=simulate(model,'stress-fast-yaw-'+camera,quad,path,initial,yaw_gain=1.6,require_arrival=False)
        stress.append(result)
    assert len({case['arrived'] for case in stress})==1
    report={'cases':reports,'projection_checks':projection_checks,
            'fast_response_stress':stress,
            'controller':'production C++ PID, ControlState, LoRa decoder and actuator allocation',
            'pc':'production head alignment, metric path guidance, speed loop and WorkstationComm',
            'camera':'projected synthetic metric trajectories; original source-video calibration included',
            'pc_settings':{'position_smoothing_alpha':vision_app.POS_SMOOTHING_ALPHA,
                           'guidance':CONTROL['guidance'],'camera_sample_hz':10,
                           'cross_track_source':'latency-compensated visual guidance, not physical measured accuracy'},
            'plant':'synthetic first-order speed/yaw: tau=0.35/0.25s, speed=0.0036 m/s per %, yaw=0.6 deg/s per % differential; 300 ms camera delay, 500 ms RF spacing',
            'boundary':'Software simulation only. Faster unmeasured yaw-response stress outcomes are recorded separately, equally across camera projections. No real-water stability, accuracy or polarity is established; production PID gains were not retuned to fit the assumed plant.'}
    (OUTPUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('PASS: coordinate/zero/S3/path/STOP invariance; 9 nominal arrivals. Fast-response stress outcomes retained in report.',flush=True)


if __name__=='__main__':main()
