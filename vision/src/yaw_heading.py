"""Align mapped IMU yaw to a manually marked head direction; no position course."""
import math

from control_config import CONTROL


def wrap_radians(angle):
    return (angle+math.pi)%(2*math.pi)-math.pi


class CalibratedYaw:
    def __init__(self,max_age_s=CONTROL['feedback_max_age_s']):
        self.max_age_s=max_age_s
        self.reset()

    def reset(self):
        self.zero_cw=None;self.zero_world=None
        self.clear_samples()

    def clear_samples(self):
        self.last_yaw=None;self.last_time=None;self.last_source_id=None;self.rate=0.

    def valid_sample(self,sample,now):
        if not isinstance(sample,dict) or sample.get('valid') is not True:
            return False
        yaw,timestamp=sample.get('yaw_cw_deg'),sample.get('sample_time')
        return (all(type(v) in (int,float) and math.isfinite(v) for v in (yaw,timestamp,now))
                and -180<=yaw<=180 and 0<=now-timestamp<=self.max_age_s)

    def calibrate(self,world_heading_rad,sample,now):
        if not math.isfinite(world_heading_rad) or not self.valid_sample(sample,now):
            raise ValueError('鱼头标定需要新鲜、有效的鱼体 IMU yaw 遥测')
        self.zero_cw=float(sample['yaw_cw_deg'])
        self.zero_world=wrap_radians(world_heading_rad)
        self.clear_samples()
        return self.update(sample,now)

    def update(self,sample,now):
        initialized=self.zero_cw is not None
        valid=initialized and self.valid_sample(sample,now)
        source_id=sample.get('source_id') if isinstance(sample,dict) else None
        restarted=(valid and type(source_id) is int and type(self.last_source_id) is int
                   and ((source_id-self.last_source_id)&0xffffffff)>=0x80000000)
        if restarted:
            self.reset();initialized=False;valid=False
        result={'initialized':initialized,'usable':bool(valid),'measurement_valid':bool(valid),
                'source':'IMU_RESTARTED' if restarted else 'IMU_YAW' if valid else 'IMU_WAIT' if initialized else 'NEEDS_HEAD_CALIBRATION',
                'yaw_deg':None,'world_yaw_rad':None,'unit_vector':None,'yaw_rate_rps':0.,
                'age_s':None,'raw_yaw_deg':sample.get('yaw_cw_deg') if isinstance(sample,dict) else None,
                'sample_time':sample.get('sample_time') if isinstance(sample,dict) else None}
        if not valid:
            return result
        timestamp=float(sample['sample_time'])
        source_id=sample.get('source_id')
        if source_id is not None and source_id==self.last_source_id:
            timestamp=self.last_time
        if not 0<=now-timestamp<=self.max_age_s:
            result.update(usable=False,measurement_valid=False,source='IMU_STALE')
            return result
        if self.last_time is not None and timestamp<self.last_time:
            result.update(usable=False,measurement_valid=False,source='IMU_OUT_OF_ORDER')
            return result
        relative=wrap_radians(math.radians(self.zero_cw-float(sample['yaw_cw_deg'])))
        world=wrap_radians(self.zero_world+relative)
        if self.last_time is not None and timestamp>self.last_time:
            dt=timestamp-self.last_time
            self.rate=wrap_radians(relative-self.last_yaw)/dt if dt<=self.max_age_s else 0.
        self.last_time=timestamp;self.last_yaw=relative
        self.last_source_id=source_id
        result.update(yaw_deg=math.degrees(relative),world_yaw_rad=world,
                      unit_vector=(math.cos(world),-math.sin(world)),yaw_rate_rps=self.rate,
                      age_s=now-timestamp,sample_time=timestamp)
        return result

    def relative_target_deg(self,world_heading_rad):
        if self.zero_world is None:
            raise ValueError('先标定鱼头与 yaw 零位')
        return math.degrees(wrap_radians(world_heading_rad-self.zero_world))

    def target_cw_deg(self,world_heading_rad):
        """Convert a world reference once into the S3 mapped IMU's CW frame."""
        if self.zero_cw is None or not math.isfinite(world_heading_rad):
            raise ValueError('先标定鱼头与 yaw 零位')
        return math.degrees(wrap_radians(math.radians(self.zero_cw)-
                            wrap_radians(world_heading_rad-self.zero_world)))
