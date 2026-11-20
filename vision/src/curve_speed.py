"""Optional measured turn/coast envelope. Never invent calibration data."""
import json
import math
from pathlib import Path

import numpy as np


def _positive(value, name):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(name + ' must be finite and positive')
    return float(value)


def _simplify(points, tolerance):
    """Iterative RDP; keep endpoints and preserve the user's path order."""
    keep = {0, len(points) - 1}
    pending = [(0, len(points) - 1)]
    while pending:
        first, last = pending.pop()
        if last <= first + 1:
            continue
        delta = points[last] - points[first]
        square = float(delta @ delta)
        middle = points[first + 1:last]
        ratio = np.clip((middle - points[first]) @ delta / square, 0, 1) if square > 1e-12 else np.zeros(len(middle))
        distance = np.linalg.norm(middle - points[first] - ratio[:, None] * delta, axis=1)
        at = int(np.argmax(distance))
        if distance[at] > tolerance:
            index = first + 1 + at
            keep.add(index)
            pending.extend(((first, index), (index, last)))
    return points[sorted(keep)]


class CurveSpeedPlanner:
    def __init__(self, settings, config_dir):
        self.settings = dict(settings)
        self.requested = settings['enabled']
        self.status = 'disabled' if not self.requested else 'missing_calibration'
        self.profile = None
        self.error = ''
        self.reset()
        if not self.requested or not settings.get('profile_file'):
            return
        try:
            base = Path(config_dir).resolve()
            path = (base / settings['profile_file']).resolve()
            if not path.is_relative_to(base) or path.suffix.lower() != '.json':
                raise ValueError('profile_file must be a JSON file inside configs')
            if path.stat().st_size > 128 * 1024:
                raise ValueError('calibration file is too large')
            data = json.loads(path.read_text(encoding='utf-8-sig'))
            self._validate(data)
            self.profile = data
            self.status = 'ready'
        except FileNotFoundError:
            self.status = 'missing_calibration'
        except (OSError, ValueError, TypeError, KeyError) as error:
            self.status = 'invalid_calibration'
            self.error = str(error)

    def _validate(self, data):
        if (not isinstance(data, dict) or data.get('schema') != 1 or data.get('drive_mode') != 'HYBRID'
                or data.get('coast_method') != 'zero_forward_keep_steering'
                or not isinstance(data.get('profile_id'), str) or not data['profile_id'].strip()
                or not isinstance(data.get('source_segments'), list) or not data['source_segments']
                or any(not isinstance(v, str) or not v.strip() for v in data['source_segments'])):
            raise ValueError('calibration identity, mode, coast method or source segments are missing')
        rows = data.get('samples')
        if not isinstance(rows, list) or not 2 <= len(rows) <= 100:
            raise ValueError('samples needs 2..100 measured rows')
        values = []
        for row in rows:
            values.append([_positive(row[key], key) for key in
                           ('speed_mps', 'coast_distance_m', 'left_radius_m', 'right_radius_m')])
        values = np.asarray(values)
        if np.any(np.diff(values[:, 0]) <= 0) or np.any(np.diff(values[:, 1]) <= 0):
            raise ValueError('speed and coast distance must strictly increase; review noisy measurements first')
        if values[0, 0] > self.settings['tight_speed_mps']:
            raise ValueError('calibration does not cover the tight-turn speed')
        self.rows = values
        self.speeds = np.r_[0., values[:, 0]]
        self.distances = np.r_[0., values[:, 1]]

    def reset(self):
        self.path = self.arc = self.curvature = self.caps = self.budget = None
        self.cruise = None
        self.rounded = False

    def validate_speed(self, speed):
        if self.profile is not None and speed > 0 and not self.rows[0, 0] <= speed <= self.speeds[-1]:
            raise ValueError('目标速度超出实测表范围，请先补充该速度的标定')

    def _row(self, speed):
        indexes = np.flatnonzero(self.rows[:, 0] <= speed + 1e-9)
        if not len(indexes):
            raise ValueError('目标速度低于实测表范围，请补充该速度的标定')
        return self.rows[indexes[-1]]

    def round_path(self, points, spacing, cruise):
        self.reset()
        if self.profile is None:
            return points
        if cruise > self.speeds[-1] + 1e-9:
            self.status = 'insufficient_speed_range'
            return points
        self.status = 'ready'
        if cruise <= 0:
            return points
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        if len(points) < 2 or not np.isfinite(points).all():
            raise ValueError('圆角路线需要至少两个有效点')
        points = points[np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-8]]
        points = _simplify(points, self.settings['simplify_tolerance_m'])
        if len(points) < 3:
            return points
        delta = np.diff(points, axis=0)
        lengths = np.linalg.norm(delta, axis=1)
        if np.any(lengths <= 1e-8):
            raise ValueError('路线有重合折返段，请重画')
        units = delta / lengths[:, None]
        cuts = np.zeros(len(points))
        radii = np.zeros(len(points))
        turns = np.zeros(len(points))
        floors = self._row(min(cruise, self.settings['tight_speed_mps']))
        preferred = self._row(min(cruise, self.settings['gentle_speed_mps']))
        desired = np.zeros(len(points))
        for i in range(1, len(points) - 1):
            u, v = units[i - 1], units[i]
            cross = float(u[0] * v[1] - u[1] * v[0])
            theta = math.atan2(cross, float(np.clip(u @ v, -1, 1)))
            if abs(theta) < 1e-5:
                continue
            if abs(theta) > math.pi - .01:
                raise ValueError(f'路线第 {i} 点接近原路折返，无法自动生成相切圆角')
            column = 2 if theta < 0 else 3  # y-down: negative is a left turn.
            radius = floors[column] * 1.02  # Allow for arc discretization, not a claim of physical margin.
            offset_factor = 1 / math.cos(theta / 2) - 1
            maximum = self.settings['max_rounding_offset_m'] / offset_factor
            if radius > maximum:
                raise ValueError(f'路线第 {i} 点需要更大的圆角偏移，请重画或调整允许偏移')
            radii[i], turns[i] = radius, theta
            desired[i] = min(preferred[column] * 1.02, maximum)
            cuts[i] = radius * math.tan(abs(theta) / 2)
        if np.any(cuts[:-1] + cuts[1:] > lengths - 1e-8):
            raise ValueError('实测转弯半径放不进相邻线段，请放大弯道后再启动')
        # Grow to the gentle-turn radius only where both neighboring fillets leave room.
        for i in range(1, len(points) - 1):
            if radii[i] == 0:
                continue
            available = min(lengths[i - 1] - cuts[i - 1], lengths[i] - cuts[i + 1])
            radii[i] = max(radii[i], min(desired[i], .99 * available / math.tan(abs(turns[i]) / 2)))
            cuts[i] = radii[i] * math.tan(abs(turns[i]) / 2)
        result = [points[0]]
        for i in range(1, len(points) - 1):
            if radii[i] == 0:
                result.append(points[i])
                continue
            u = units[i - 1]
            entry = points[i] - cuts[i] * u
            centre = entry + math.copysign(radii[i], turns[i]) * np.array([-u[1], u[0]])
            start = math.atan2(entry[1] - centre[1], entry[0] - centre[0])
            count = max(3, math.ceil(abs(turns[i]) * radii[i] / spacing) + 1)
            angles = np.linspace(start, start + turns[i], count)
            result.extend(centre + radii[i] * np.column_stack((np.cos(angles), np.sin(angles))))
        result.append(points[-1])
        result = np.asarray(result)
        result = result[np.r_[True, np.linalg.norm(np.diff(result, axis=0), axis=1) > 1e-8]]
        self.rounded = bool(np.any(radii))
        return result

    def prepare(self, path, arc, cruise):
        if self.profile is None or self.status == 'insufficient_speed_range':
            return
        self.path, self.arc = np.asarray(path), np.asarray(arc)
        if cruise > self.speeds[-1] + 1e-9:
            raise ValueError('目标速度超过实测减速表，请先补充标定')
        self.cruise = float(cruise)
        self.curvature = np.zeros(len(path))
        a, b = np.diff(path, axis=0)[:-1], np.diff(path, axis=0)[1:]
        denominator = np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1) * np.linalg.norm(a + b, axis=1)
        if np.any(denominator < 1e-12):
            raise ValueError('路线存在退化折返，无法计算曲率')
        self.curvature[1:-1] = -2 * (a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]) / denominator
        self.caps = np.full(len(path), cruise)
        for i, curvature in enumerate(self.curvature):
            if abs(curvature) < 1e-5 or cruise <= 0:
                continue
            radius = 1 / abs(curvature)
            column = 2 if curvature > 0 else 3
            for speed in (self.settings['gentle_speed_mps'], self.settings['tight_speed_mps']):
                row = self._row(min(cruise, speed))
                if row[column] <= radius * 1.001:
                    self.caps[i] = min(cruise, row[0])
                    break
            else:
                raise ValueError(f'圆角后第 {i} 点仍小于实测转弯半径，请放大路线')
        # D(v_i) <= ds + D(v_(i+1)): propagate measured coast distance backwards.
        self.budget = np.interp(self.caps, self.speeds, self.distances)
        self.budget[-1] = 0.
        for i in range(len(path) - 2, -1, -1):
            self.budget[i] = min(self.budget[i], self.budget[i + 1] + arc[i + 1] - arc[i])

    def limit(self, progress, target, cruise):
        info = {'enabled': self.requested, 'active': self.budget is not None,
                'status': self.status, 'profile_id': self.profile['profile_id'] if self.profile else None,
                'rounded': self.rounded, 'error': self.error or None}
        if self.budget is None:
            return target, info
        if cruise != self.cruise:
            self.prepare(self.path, self.arc, cruise)
        distance = float(np.interp(progress, self.arc, self.budget))
        cap = float(np.interp(distance, self.distances, self.speeds))
        future = np.flatnonzero((self.arc >= progress) & (np.abs(self.curvature) > 1e-5))
        i = int(future[0]) if len(future) else None
        info.update(status='active', speed_limit_mps=cap,
                    distance_to_curve_m=float(self.arc[i] - progress) if i is not None else None,
                    curve_speed_mps=float(self.caps[i]) if i is not None else None,
                    curve_radius_m=float(1 / abs(self.curvature[i])) if i is not None else None)
        return min(target, cap), info


class DisplacementSpeedCheck:
    """One-second chord speed for diagnostics only; never feeds the controller."""
    def __init__(self, max_gap_s=.35):
        from collections import deque
        self.samples = deque(maxlen=90)
        self.identity = None
        self.max_gap_s = max_gap_s

    def update(self, position, stamp, identity):
        if position is None or type(stamp) not in (int, float) or not math.isfinite(stamp):
            self.samples.clear()
            return None
        position = np.asarray(position, dtype=float)
        if position.shape != (2,) or not np.isfinite(position).all():
            self.samples.clear()
            return None
        if identity != self.identity or (self.samples and
                (stamp < self.samples[-1][0] or stamp - self.samples[-1][0] > self.max_gap_s)):
            self.samples.clear()
        self.identity = identity
        if self.samples and stamp == self.samples[-1][0]:
            return None
        while self.samples and stamp - self.samples[0][0] > 1.2:
            self.samples.popleft()
        previous = min(self.samples, key=lambda p: abs(stamp - p[0] - 1.)) if self.samples else None
        speed = float(np.linalg.norm(position - previous[1]) / (stamp - previous[0])) if previous and .8 <= stamp - previous[0] <= 1.2 else None
        self.samples.append((float(stamp), position.copy()))
        return speed
