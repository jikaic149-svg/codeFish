"""PC visual/speed settings. Angle PID gains belong to the S3 configuration."""
import json
import math
from pathlib import Path


def load_control_config():
    data = json.loads((Path(__file__).parent/'configs/control.json').read_text(encoding='utf-8'))
    for group in ('model','speed','guidance'):
        for key,value in data[group].items():
            if type(value) not in (int,float) or not math.isfinite(value) or value<0:
                raise ValueError(f'Invalid control setting: {group}.{key}')
    if (not 0<data['feedback_max_age_s']<=1 or not 0<=data.get('camera_latency_s',.3)<=2 or data['speed']['reference_mps']<=0
            or not 0<data['model']['confidence']<1 or data['model']['imgsz']<32
            or not 0<data['guidance']['min_lookahead_m']<=data['guidance']['lookahead_m']):
        raise ValueError('Invalid model or control ranges')
    return data


CONTROL = load_control_config()
