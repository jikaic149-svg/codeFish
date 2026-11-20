"""Replay the exact source video through the runtime tracker, without devices."""
from collections import Counter
import json
import os
from pathlib import Path
import sys
import time

import cv2

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision/src'))
os.environ.setdefault('YOLO_OFFLINE','1')
from yolo_tracker import YoloBodyTracker


def main():
    samples=os.environ.get('ROBOFISH_VISION_SAMPLES')
    source=next(iter(sorted(Path(samples).glob('*.mp4')))) if samples else ROOT.parent/'屏幕录制 2026-10-18 115851.mp4'
    output=ROOT/'build/checks/host-vision/yolo-imu'
    output.mkdir(parents=True,exist_ok=True)
    capture=cv2.VideoCapture(str(source))
    if not capture.isOpened():raise RuntimeError(source)
    fps=capture.get(cv2.CAP_PROP_FPS);count=int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    width,height=(int(capture.get(key)) for key in (cv2.CAP_PROP_FRAME_WIDTH,cv2.CAP_PROP_FRAME_HEIGHT))
    tracker=YoloBodyTracker(ROOT/'vision/models/fish_body_yolo/best.pt')
    writer=cv2.VideoWriter(str(output/'runtime_tracking.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),fps,(width,height))
    if not writer.isOpened():raise RuntimeError('Video writer failed')
    records=[];started=time.perf_counter()
    try:
        for number in range(count):
            ok,image=capture.read()
            if not ok:raise RuntimeError(f'Decode failed at {number}')
            observation=tracker.update(image,100.+number/fps)
            if observation.valid:
                box=observation.metrics['fish_bbox'];a,b,c,d=map(round,box)
                color=(0,255,0) if observation.yolo_confirmed else (0,180,255)
                cv2.rectangle(image,(a,b),(c,d),color,2)
                cv2.putText(image,observation.metrics['tracking_mode'],(a,max(20,b-7)),cv2.FONT_HERSHEY_SIMPLEX,.5,color,1)
            writer.write(image)
            records.append({'frame':number,'seconds':number/fps,'valid':observation.valid,
                            'detector_confirmed':observation.yolo_confirmed,'track_id':observation.metrics.get('track_id'),
                            'mode':observation.metrics.get('tracking_mode'),'position':observation.position,
                            'bbox':observation.metrics.get('fish_bbox')})
            if number and number%400==0:print(f'Runtime tracker: {number}/{count}',flush=True)
    finally:
        capture.release();writer.release()
    check=cv2.VideoCapture(str(output/'runtime_tracking.mp4'))
    assert int(check.get(cv2.CAP_PROP_FRAME_COUNT))==count
    check.set(cv2.CAP_PROP_POS_FRAMES,count-1);ok,_=check.read();check.release();assert ok
    summary={'source':str(source),'frames':count,'missing_frames':sum(not r['valid'] for r in records),
             'modes':dict(Counter(r['mode'] for r in records)),
             'track_ids':sorted({r['track_id'] for r in records if r['track_id'] is not None}),
             'elapsed_s':time.perf_counter()-started,'video_verified':True,
             'note':'Runtime coverage is not detection accuracy. Yellow boxes are measured optical flow; no yaw is inferred from displacement.'}
    (output/'runtime_tracking.json').write_text(json.dumps({'summary':summary,'frames':records},indent=2),encoding='utf-8')
    print(json.dumps(summary,indent=2),flush=True)


if __name__=='__main__':main()
