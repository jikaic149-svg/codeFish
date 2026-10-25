"""PC visual/speed settings. Angle PID gains belong to the S3 configuration."""
import json
import math
from pathlib import Path


def load_control_config():
    data = json.loads((Path(__file__).parent/'configs/control.json').read_text(encoding='utf-8'))
    for group in ('model','speed','guidance'):
        for key,value in data[group].items():
            if group=='speed' and key=='profile_file':
                if value is not None and (not isinstance(value,str) or not value.strip()):
                    raise ValueError('Speed profile_file must be a path or null')
                continue
            if type(value) not in (int,float) or not math.isfinite(value) or value<0:
                raise ValueError(f'Invalid control setting: {group}.{key}')
    if (not 0<data['feedback_max_age_s']<=1 or not 0<=data.get('camera_latency_s',.3)<=2 or data['speed']['reference_mps']<=0
            or not 0<data['model']['confidence']<1 or data['model']['imgsz']<32
            or not 0<data['guidance']['min_lookahead_m']<=data['guidance']['lookahead_m']):
        raise ValueError('Invalid model or control ranges')
    curve=data.get('curve_speed',{})
    validation=data.get('trace_validation',{})
    if type(curve.get('enabled',False)) is not bool or type(validation.get('enabled',False)) is not bool:
        raise ValueError('Feature enabled flags must be boolean')
    if curve:
        for key in ('gentle_speed_mps','tight_speed_mps','simplify_tolerance_m','max_rounding_offset_m'):
            value=curve.get(key)
            if type(value) not in (int,float) or not math.isfinite(value) or value<=0:
                raise ValueError('Invalid curve-speed setting: '+key)
        if curve['tight_speed_mps']>curve['gentle_speed_mps']:
            raise ValueError('Tight-turn speed exceeds gentle-turn speed')
        if curve.get('profile_file') is not None and not isinstance(curve['profile_file'],str):
            raise ValueError('profile_file must be a path or null')
    return data


CONTROL = load_control_config()
