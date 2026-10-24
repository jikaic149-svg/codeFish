"""Continuous heading after explicit initial calibration, x right/y down.

Short-window position slopes gently correct the forward orientation. They never
replace it directly. Stationary, noisy or sideways motion holds the last angle.
"""
from collections import deque
import math


def wrap(angle):
    return (angle+math.pi)%(2*math.pi)-math.pi


class PositionYaw:
    def __init__(self, window_s=.60, min_span_s=.25, pixel_distance=10., world_distance=.03,
                 smoothing_s=.5, maximum_turn_deg_s=90.):
        self.window_s=window_s;self.min_span_s=min_span_s
        self.pixel_distance=pixel_distance;self.world_distance=world_distance
        self.smoothing_s=smoothing_s;self.maximum_turn=math.radians(maximum_turn_deg_s)
        self.reset()

    def reset(self, yaw_rad=None, timestamp=None, space=None):
        self.samples=deque();self.space=space;self.last_frame=None
        self.yaw=wrap(float(yaw_rad)) if yaw_rad is not None else None
        self.last_measurement=timestamp if self.yaw is not None else None
        self.raw_course=None;self.fit_quality=0.;self.fit_span=0.;self.displacement=0.
        self.result=self._result(timestamp,'INITIAL' if self.yaw is not None else 'NEEDS_INITIAL_HEADING')

    def clear_samples(self):
        self.samples.clear();self.last_frame=None
        self.raw_course=None;self.fit_quality=0.;self.fit_span=0.;self.displacement=0.

    def _result(self, now, source, valid=False, rate=0., limited=False):
        age=max(0.,now-self.last_measurement) if now is not None and math.isfinite(now) and self.last_measurement is not None else None
        return {'yaw_deg':math.degrees(self.yaw) if self.yaw is not None else None,
                'yaw_rate_rps':rate,'unit_vector':(math.cos(self.yaw),-math.sin(self.yaw)) if self.yaw is not None else None,
                'source':source,'measurement_valid':valid,'age_s':age,'space':self.space,
                'initialized':self.yaw is not None,'raw_course_deg':self.raw_course,
                'fit_quality':self.fit_quality,'fit_span_s':self.fit_span,
                'displacement':self.displacement,'turn_limited':limited}

    def update(self, position, timestamp, space='pixel'):
        now=float(timestamp)
        if not math.isfinite(now):return self._result(None,'INVALID')
        if self.last_frame is not None and now<=self.last_frame:return self._result(now,'STALE')
        if self.space is not None and self.space!=space:self.reset(space=space)
        previous_frame=self.last_frame;self.space=space;self.last_frame=now
        if position is None or len(position)!=2 or not all(math.isfinite(float(v)) for v in position):
            self.samples.clear();self.result=self._result(now,'LOST');return self.result
        point=tuple(map(float,position))
        if self.yaw is None:
            self.result=self._result(now,'NEEDS_INITIAL_HEADING');return self.result
        if self.samples and now-self.samples[-1][0]>.6:self.samples.clear()
        if self.samples:
            dt=now-self.samples[-1][0];jump=math.dist(point,self.samples[-1][1])
            if jump>max(.03 if space=='world' else 12.,dt*(.8 if space=='world' else 1000.)):
                self.samples.clear();self.samples.append((now,point))
                self.result=self._result(now,'REJECTED_JUMP');return self.result
        self.samples.append((now,point))
        while len(self.samples)>1 and now-self.samples[0][0]>self.window_s:self.samples.popleft()
        span=now-self.samples[0][0];distance=math.dist(point,self.samples[0][1])
        self.fit_span=span;self.displacement=distance;self.fit_quality=0.
        threshold=self.world_distance if space=='world' else self.pixel_distance
        if span<self.min_span_s or distance<threshold:
            self.result=self._result(now,'HOLD');return self.result
        times=[t for t,_ in self.samples];points=[p for _,p in self.samples]
        mean_t=sum(times)/len(times);mean_x=sum(p[0] for p in points)/len(points);mean_y=sum(p[1] for p in points)/len(points)
        denominator=sum((t-mean_t)**2 for t in times)
        vx=sum((t-mean_t)*(p[0]-mean_x) for t,p in self.samples)/denominator
        vy=sum((t-mean_t)*(p[1]-mean_y) for t,p in self.samples)/denominator
        fitted_distance=math.hypot(vx,vy)*span
        residual=sum((p[0]-mean_x-vx*(t-mean_t))**2+(p[1]-mean_y-vy*(t-mean_t))**2 for t,p in self.samples)/len(points)
        self.fit_quality=1/(1+math.sqrt(residual)/max(fitted_distance,1e-6))
        if fitted_distance<threshold or self.fit_quality<.70:
            self.result=self._result(now,'HOLD_NOISY');return self.result
        measured=math.atan2(-vy,vx);self.raw_course=math.degrees(measured)
        delta=wrap(measured-self.yaw)
        # Maintain the calibrated forward half-plane; backward drift is not
        # a 180-degree rotation. Sideways displacement supplies no useful yaw.
        if abs(delta)>math.pi/2:delta=wrap(delta+math.pi)
        if abs(delta)>math.radians(60) and self.fit_quality<.90:
            self.result=self._result(now,'HOLD_SIDEWAYS');return self.result
        dt=min(.10,max(0.,now-previous_frame)) if previous_frame is not None else 0.
        correction=delta*(1-math.exp(-dt/self.smoothing_s))
        limit=self.maximum_turn*dt;step=max(-limit,min(limit,correction))
        self.yaw=wrap(self.yaw+step);self.last_measurement=now
        self.result=self._result(now,'HEADING_FIT',True,step/dt if dt else 0.,abs(correction)>limit)
        return self.result
