"""Profile the exact source/model without a camera, device connection or motion."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import statistics
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision/src'))
import cv2
import numpy as np
import torch
from observer_runtime import ObserverSession
from pool_perspective import PoolPerspective
from web_runtime import WebBridge


def summary(values):
    return {'mean':statistics.mean(values),'median':statistics.median(values),
            'p95':float(np.percentile(values,95))}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--frames',type=int,default=300)
    args=parser.parse_args();cv2.setNumThreads(2)
    output=ROOT/'build/checks/host-vision/heading-observer/profile';output.mkdir(parents=True,exist_ok=True)
    for name,key in [('ultralytics','YOLO_CONFIG_DIR'),('matplotlib','MPLCONFIGDIR')]:
        folder=output/name;folder.mkdir(exist_ok=True);os.environ[key]=str(folder)
    samples=os.environ.get('ROBOFISH_VISION_SAMPLES')
    source=next(iter(sorted(Path(samples).glob('*.mp4')))) if samples else ROOT.parent/'屏幕录制 2026-10-18 115851.mp4'
    model=ROOT/'vision/models/fish_body_yolo/best.pt'
    os.environ.update(ROBOFISH_PARENT_TOKEN='offline-profile',ROBOFISH_VISION_TOKEN='offline-profile')
    bridge=WebBridge(observer=True) # deliberately not started: no telemetry or HTTP threads
    session=ObserverSession(str(source),model,bridge,output=output,calibration=output/'unused-pool.json')
    capture=cv2.VideoCapture(str(source));assert capture.isOpened()
    fps=capture.get(cv2.CAP_PROP_FPS);rows=[];began=time.perf_counter()
    try:
        for index in range(1,args.frames+21):
            started=time.perf_counter();ok,frame=capture.read()
            if not ok:break
            decode=(time.perf_counter()-started)*1000
            if index==1:
                session.field=PoolPerspective(4.,2.).calibrate([[271,150],[1200,121],[1205,632],[259,606]],frame.shape)
                session.pool_id='offline-benchmark-configured-4x2'
                session.samples.start({'source':str(source),'replay':True,'benchmark':True,
                    'note':'No manual head calibration; the benchmark does not validate the physical head direction.'})
            session.accept_frame(index,time.time(),frame,(index-1)/fps)
            session.preview=index<=20+args.frames//2
            # 3 preview frames per 30 source frames, with no artificial replay sleep.
            render=session.preview and index%10==0
            session.preview_clock=0. if render else time.monotonic()
            session.render_preview();session.publish(write_sample=True)
            if index<=20:continue # model initialization, warmup and allocator startup excluded
            speed=session.tracker.last_speed
            rows.append({'frame':index,'preview_enabled':session.preview,'decode_ms':decode,
                'detect_ms':session.detect_ms,'optical_ms':session.optical_ms,
                'model_predict_ms':session.tracker.last_predict_ms,
                'tracking_ms':max(0.,session.detect_ms-session.tracker.last_predict_ms),
                'preview_ms':session.preview_ms if render else 0.,
                **{key+'_ms':speed.get(key,0.) for key in ('preprocess','inference','postprocess')},
                'pipeline_ms':(time.perf_counter()-started)*1000})
        session.samples.close()
    finally:capture.release()
    grouped={}
    for enabled in (True,False):
        selected=[row for row in rows if row['preview_enabled']==enabled]
        if not selected:continue
        grouped['preview_3fps' if enabled else 'preview_off']={
            key:summary([row[key] for row in selected]) for key in selected[0] if key.endswith('_ms')}
        grouped['preview_3fps' if enabled else 'preview_off']['capacity_fps']=1000/statistics.mean(row['pipeline_ms'] for row in selected)
    report={'source':str(source),'model':str(model),'model_sha256':hashlib.sha256(model.read_bytes()).hexdigest(),
        'device':torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'cpu',
        'torch':torch.__version__,'opencv':cv2.__version__,'imgsz':session.tracker.imgsz,
        'fp16':bool(session.tracker.model.predictor.model.fp16),'frames':len(rows),'opencv_threads':cv2.getNumThreads(),
        'source_fps':fps,'total_elapsed_s':time.perf_counter()-began,'stages':grouped,
        'note':'Unpaced offline processing capacity, not physical camera FPS or control response time. Warmup excluded; no fish head accuracy claim.'}
    with (output/'frames.csv').open('w',encoding='utf-8',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    (output/'performance.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':main()
