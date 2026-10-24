"""Visual translation-speed feedback; output is forward thrust percentage."""
import math
import time


class VisualSpeedLoop:
    def __init__(self, reference_mps=.36, kp=60., ki=30., kd=4., integral_limit=20.,
                 max_age_s=.25, max_speed_mps=.8):
        self.reference_mps = float(reference_mps)
        self.kp, self.ki, self.kd = float(kp), float(ki), float(kd)
        self.integral_limit = float(integral_limit)
        self.max_age_s, self.max_speed_mps = float(max_age_s), float(max_speed_mps)
        if not all(math.isfinite(v) and v >= 0 for v in (
                self.reference_mps, self.kp, self.ki, self.kd, self.integral_limit,
                self.max_age_s, self.max_speed_mps)) or self.reference_mps == 0:
            raise ValueError('Invalid visual speed-loop settings')
        self.reset()

    def reset(self, state='stopped'):
        self.integral = 0.
        self.last_time = None
        self.last_speed = None
        self.acceleration = 0.
        self.status = {'state': state, 'feedback_valid': False, 'power_percent': 0.,
                       'target_mps': None, 'measured_mps': None,
                       'error_mps': None, 'integral_percent': 0.}
        return 0.

    @staticmethod
    def _finite(value):
        return type(value) in (int, float) and math.isfinite(value)

    def update(self, target_mps, measured_mps, observed_at, *, now=None, brake=False):
        now = time.time() if now is None else now
        if brake:
            return self.reset('braking')
        if not self._finite(target_mps) or target_mps < 0:
            return self.reset('invalid_target')
        if target_mps == 0:
            return self.reset()
        if not self._finite(measured_mps) or not 0 <= measured_mps <= self.max_speed_mps:
            return self.reset('waiting_feedback')
        if (not self._finite(observed_at) or not self._finite(now)
                or observed_at <= 0 or not 0 <= now - observed_at <= self.max_age_s):
            return self.reset('stale_feedback')
        if self.last_time is not None and observed_at < self.last_time:
            return self.reset('out_of_order_feedback')

        elapsed = 0. if self.last_time is None else observed_at - self.last_time
        if elapsed > self.max_age_s:
            self.integral = 0.
            self.acceleration = 0.
            elapsed = 0.
        if elapsed > 0 and self.last_speed is not None:
            measured_acceleration = (measured_mps-self.last_speed)/elapsed
            self.acceleration += elapsed/(.15+elapsed)*(measured_acceleration-self.acceleration)
        error = target_mps - measured_mps
        feedforward = 100. * target_mps / self.reference_mps
        # Conditional integration keeps saturation from accumulating hidden thrust.
        candidate = max(-self.integral_limit, min(self.integral_limit,
            self.integral + self.ki * error * elapsed))
        derivative = -self.kd*self.acceleration
        demanded = feedforward + self.kp * error + candidate + derivative
        if 0 <= demanded <= 100 or (demanded > 100 and error < 0) or (demanded < 0 and error > 0):
            self.integral = candidate
        power = max(0., min(100., feedforward + self.kp * error + self.integral + derivative))
        self.last_time = observed_at
        self.last_speed = measured_mps
        self.status = {'state': 'tracking', 'feedback_valid': True,
                       'power_percent': power, 'target_mps': target_mps,
                       'measured_mps': measured_mps, 'error_mps': error,
                       'integral_percent': self.integral, 'derivative_percent': derivative,
                       'observation_age_ms': round((now - observed_at) * 1000)}
        return power
