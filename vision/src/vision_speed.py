"""Visual translation-speed feedback; output is forward thrust percentage."""
import math
import time
import json
from bisect import bisect_left
from pathlib import Path


class VisualSpeedLoop:
    def __init__(self, reference_mps=.36, kp=60., ki=30., kd=4., integral_limit=20.,
                 max_age_s=.25, max_speed_mps=.8, profile_file=None):
        self.reference_mps = float(reference_mps)
        self.kp, self.ki, self.kd = float(kp), float(ki), float(kd)
        self.integral_limit = float(integral_limit)
        self.max_age_s, self.max_speed_mps = float(max_age_s), float(max_speed_mps)
        if not all(math.isfinite(v) and v >= 0 for v in (
                self.reference_mps, self.kp, self.ki, self.kd, self.integral_limit,
                self.max_age_s, self.max_speed_mps)) or self.reference_mps == 0:
            raise ValueError('Invalid visual speed-loop settings')
        self.feedforward_samples = []
        self.profile_id = None
        if profile_file is not None:
            base = (Path(__file__).resolve().parents[1] / 'configs').resolve()
            path = (base / profile_file).resolve()
            if not path.is_relative_to(base) or path.suffix.lower() != '.json':
                raise ValueError('Speed profile must be a JSON file inside configs')
            if path.stat().st_size > 128 * 1024:
                raise ValueError('Speed profile is too large')
            data = json.loads(path.read_text(encoding='utf-8-sig'))
            rows = data.get('propulsion', {}).get('samples')
            if (data.get('schema') != 1 or not isinstance(data.get('profile_id'), str)
                    or not data['profile_id'].strip() or not isinstance(rows, list)
                    or not 2 <= len(rows) <= 100):
                raise ValueError('Speed profile needs an identity and 2..100 measured samples')
            previous_speed = previous_command = 0.
            for row in rows:
                speed, command = row.get('speed_mps'), row.get('command_pct')
                if (not self._finite(speed) or not self._finite(command)
                        or not previous_speed < speed <= self.max_speed_mps
                        or not previous_command < command <= 100):
                    raise ValueError('Measured speed and command must strictly increase')
                self.feedforward_samples.append((float(speed), float(command)))
                previous_speed, previous_command = speed, command
            self.profile_id = data['profile_id']
        self.reset()

    def _feedforward(self, target):
        if not self.feedforward_samples:
            return 100. * target / self.reference_mps, 'reference'
        first, last = self.feedforward_samples[0], self.feedforward_samples[-1]
        if target < first[0]:
            # Zero-to-first interpolation is an estimate, not a measured low-speed sample.
            return first[1] * target / first[0], 'below_measured_range'
        if target > last[0]:
            return last[1], 'above_measured_range'
        index = bisect_left(self.feedforward_samples, (target, -math.inf))
        upper = self.feedforward_samples[index]
        if index == 0:
            return upper[1], 'measured_range'
        lower = self.feedforward_samples[index - 1]
        fraction = (target - lower[0]) / (upper[0] - lower[0])
        return lower[1] + fraction * (upper[1] - lower[1]), 'measured_range'

    def reset(self, state='stopped'):
        self.integral = 0.
        self.last_time = None
        self.last_speed = None
        self.acceleration = 0.
        self.status = {'state': state, 'feedback_valid': False, 'power_percent': 0.,
                       'target_mps': None, 'measured_mps': None,
                       'error_mps': None, 'integral_percent': 0.,
                       'feedforward_percent': 0., 'feedforward_range': None,
                       'observation_age_ms': None, 'receipt_age_ms': None,
                       'feedforward_profile_id': self.profile_id}
        return 0.

    @staticmethod
    def _finite(value):
        return type(value) in (int, float) and math.isfinite(value)

    def update(self, target_mps, measured_mps, observed_at, *, now=None, brake=False, received_at=None):
        now = time.time() if now is None else now
        if brake:
            return self.reset('braking')
        if not self._finite(target_mps) or target_mps < 0:
            return self.reset('invalid_target')
        if target_mps == 0:
            return self.reset()
        if not self._finite(measured_mps) or not 0 <= measured_mps <= self.max_speed_mps:
            return self.reset('waiting_feedback')
        # Capture time drives integration/derivatives; arrival time, when given,
        # gates processing freshness without charging the fixed camera latency
        # against the same budget. Neither timestamp is rewritten to "now".
        fresh_at = observed_at if received_at is None else received_at
        if (not self._finite(observed_at) or not self._finite(now) or not self._finite(fresh_at)
                or observed_at <= 0 or fresh_at <= 0 or observed_at > fresh_at
                or not 0 <= now - fresh_at <= self.max_age_s):
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
        feedforward, feedforward_range = self._feedforward(target_mps)
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
                       'feedforward_percent': feedforward,
                       'feedforward_range': feedforward_range,
                       'feedforward_profile_id': self.profile_id,
                       'integral_percent': self.integral, 'derivative_percent': derivative,
                       'observation_age_ms': round((now - observed_at) * 1000),
                       'receipt_age_ms': round((now - received_at) * 1000) if received_at is not None else None}
        return power
