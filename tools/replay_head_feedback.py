"""Replay the production nose measurement and speed loop; no camera, sender or IMU."""
import argparse
import csv
import os
from pathlib import Path
import sys
from unittest.mock import Mock

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'vision'))
from observer_runtime import ObserverSession
from pool_perspective import PoolPerspective
from control_config import CONTROL
from vision_speed import VisualSpeedLoop
from web_runtime import WebBridge


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video',type=Path)
    parser.add_argument('--corners',type=float,nargs=8,required=True,help='TL TR BR BL pixel pairs')
    parser.add_argument('--size',type=float,nargs=2,required=True,help='width and height in metres')
    parser.add_argument('--target',type=float,default=.08,help='fixed shadow speed target in m/s')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    capture=cv2.VideoCapture(str(args.video.resolve()))
    fps=capture.get(cv2.CAP_PROP_FPS)
    if not capture.isOpened() or not fps>0:parser.error('video is not readable')
    os.environ.update(ROBOFISH_PARENT_TOKEN='offline',ROBOFISH_VISION_TOKEN='offline')
    bridge=WebBridge(observer=True)  # never started
    bridge.parent=Mock(side_effect=AssertionError('Device access in replay'))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    session=ObserverSession(str(args.video.resolve()),ROOT/'vision/fish_body_yolo/best.pt',bridge,
                            output=args.output.parent,calibration=args.output.parent/'unused-replay-pool.json')
    session.speed_source='white_head_forward'
    loop=VisualSpeedLoop(**CONTROL['speed'],max_age_s=CONTROL['feedback_max_age_s'])
    cv2.setNumThreads(2)
    count=hits=valid=gap=longest=0
    try:
        with args.output.open('w',newline='',encoding='utf-8') as stream:
            out=csv.writer(stream)
            out.writerow(('frame','t','head_x_px','head_y_px','head_state','speed_mps','body_ground_mps',
                          'shadow_target_mps','shadow_power_pct','shadow_state','processing_ms','head_speed_epoch',
                          'head_x_m','head_y_m','body_x_m','body_y_m','track_id','head_signed_mps','head_point_source','speed_sample_time'))
            while True:
                ok,frame=capture.read()
                if not ok:break
                if count==0:
                    session.field=PoolPerspective(*args.size).calibrate(np.array(args.corners).reshape(4,2),frame.shape)
                    session.pool_id='replay'
                t=count/fps
                receipt=t+session.camera_latency_s
                session.accept_frame(count,receipt,frame,t)
                power=loop.update(args.target,session.speed_mps,session.speed_time,now=receipt,
                                  received_at=session.speed_time+session.camera_latency_s if session.speed_time is not None else None)
                if session.speed_mps is None and power!=0:raise AssertionError('Power without measurement')
                nose=session.nose_px
                body=session.velocity.velocity
                centre=session.position()
                out.writerow((count,t,*(nose or ('','')),session.head_speed.state,session.speed_mps,
                              np.linalg.norm(body) if body is not None else None,args.target,power,
                              loop.status['state'],session.processing_ms,session.head_speed.epoch,
                              *(('','') if nose is None else session.field.transform([nose])[0]),
                              *(('','') if centre is None else centre),session.detection.metrics.get('track_id'),
                              session.head_speed.signed_mps if session.head_speed.state=='TRACKING' else None,
                              session.nose.source,session.speed_time))
                hits+=nose is not None
                valid+=session.speed_mps is not None
                gap=gap+1 if session.speed_mps is None else 0
                longest=max(longest,gap)
                count+=1
    finally:capture.release()
    bridge.parent.assert_not_called()
    if not count:raise RuntimeError('No frames decoded')
    print(f'{count} frames; nose {hits/count:.1%}; speed {valid/count:.1%}; longest speed gap {longest/fps:.3f}s; no device I/O.')


if __name__=='__main__':main()
