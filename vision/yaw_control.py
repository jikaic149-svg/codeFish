"""Yaw PID inner loop, CCW/left positive; derivative acts on measured yaw."""
import math
import time

from control_config import CONTROL
from yaw_heading import wrap_radians


class YawLoop:
    def __init__(self,**settings):
        values={**CONTROL['yaw'],**settings}
        for name in ('kp','ki','kd','integral_limit'):
            value=values[name]
            if type(value) not in (int,float) or not math.isfinite(value) or value<0:
                raise ValueError('Invalid yaw PID gains')
            setattr(self,name,float(value))
        self.max_age_s=CONTROL['feedback_max_age_s']
        self.reset()

    def reset(self,state='stopped'):
        self.integral=0.;self.last_time=None;self.last_measured=None;self.rate=0.
        self.status={'state':state,'feedback_valid':False,'steering_percent':0.}
        return 0.

    def update(self,target_rad,measured_rad,observed_at,*,now=None,brake=False,feedforward=0.):
        now=time.time() if now is None else now
        if brake:return self.reset('braking')
        if (any(type(v) not in (int,float) or not math.isfinite(v) for v in
                (target_rad,measured_rad,observed_at,now,feedforward))
                or not 0<=now-observed_at<=self.max_age_s):
            return self.reset('waiting_feedback')
        if self.last_time is not None and observed_at<self.last_time:
            return self.reset('out_of_order_feedback')
        dt=0. if self.last_time is None else observed_at-self.last_time
        if dt>self.max_age_s:
            self.integral=0.;self.rate=0.;dt=0.
        if dt>0:
            rate=wrap_radians(measured_rad-self.last_measured)/dt
            self.rate += dt/(.15+dt)*(rate-self.rate)
        error=wrap_radians(target_rad-measured_rad)
        p=self.kp*error;d=-self.kd*self.rate
        candidate=max(-self.integral_limit,min(self.integral_limit,self.integral+self.ki*error*dt))
        demand=feedforward+p+candidate+d
        if (-100<=demand<=100 or demand>100 and error<0 or demand< -100 and error>0):
            self.integral=candidate
        output=max(-100.,min(100.,feedforward+p+self.integral+d))
        self.last_time=observed_at;self.last_measured=measured_rad
        self.status={'state':'tracking','feedback_valid':True,'steering_percent':output,
                     'error_deg':math.degrees(error),'p_percent':p,'i_percent':self.integral,
                     'd_percent':d,'feedforward_percent':feedforward,'sample_time':observed_at}
        return output
