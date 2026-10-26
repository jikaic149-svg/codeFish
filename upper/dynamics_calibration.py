"""Passive dynamics measurements; control writes belong to the caller.

Each source has its own 10 Hz reader, so a delayed camera cannot stop fish
telemetry collection. Full source snapshots are append-only and flushed at
every write. CSV values are joined observations, never hardware feedback.
"""
from collections import deque
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import statistics
import threading
import time
import uuid


PARAMETERS = ('tail_drive_pct', 'tail_bias_pct', 'prop_surge_pct', 'prop_diff_pct')
FIELDS = ('utc', 'monotonic_s', 'time_s', 'source', 'phase', 'frame',
          'camera_sample_time', 'camera_receipt_time', 'uptime_ms',
          'fish_checked_at', 'fish_age_ms', 'imu_age_ms', 'imu_valid',
          'x_m', 'y_m', 'position_valid', 'yaw_deg', 'yaw_source',
          'imu_yaw_cw_deg', 'raw_yaw_deg', 'filtered_yaw_deg',
          'vision_speed_mps', 'speed_1s_mps', 'yaw_rate_dps',
          'left_pct', 'right_pct', 'tail1_deg', 'tail2_deg',
          *PARAMETERS, 'output_tail_drive_pct', 'output_tail_bias_pct',
          'output_prop_surge_pct', 'output_prop_diff_pct',
          'yaw_p', 'yaw_i', 'yaw_d', 'yaw_output_pct', 'safety', 'owner')


def desktop_directory():
    """Resolve the user's redirected Desktop, including OneDrive."""
    portable = os.environ.get('ROBOFISH_EXPERIMENTS_DIR')
    if portable:
        return Path(portable).resolve()
    if os.name == 'nt':
        import winreg
        with winreg.OpenKey(winreg.HKEY_CURRENT_USER,
                r'Software\Microsoft\Windows\CurrentVersion\Explorer\User Shell Folders') as key:
            value, _ = winreg.QueryValueEx(key, 'Desktop')
        path = Path(os.path.expandvars(value))
        if not path.is_absolute():
            raise OSError('系统桌面目录无效，请检查 Windows 桌面重定向配置')
        return path
    return Path.home() / 'Desktop'


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def number(value):
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (ValueError, TypeError):
        return None


def wrap(value):
    return (value + 180.) % 360. - 180.


def utc(stamp=None):
    return datetime.fromtimestamp(time.time() if stamp is None else stamp,
                                  timezone.utc).isoformat(timespec='milliseconds')


def _clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _clean(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_clean(item) for item in value]
    return value


def _redact(value):
    if isinstance(value, dict):
        return {key: '[redacted]' if key.lower() in (
            'password', 'token', 'authorization', 'cookie', 'session', 'session_id',
            'session_token', 'control_session', 'api_key', 'secret') else _redact(item)
            for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_redact(item) for item in value]
    return value


def _json(value):
    return json.dumps(_clean(value), ensure_ascii=False, separators=(',', ':'), allow_nan=False)


def _atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + '.pending')
    with temporary.open('w', encoding='utf-8') as output:
        output.write(json.dumps(_clean(value), ensure_ascii=False, indent=2, allow_nan=False) + '\n')
        output.flush()
        os.fsync(output.fileno())
    temporary.replace(path)


def _solve3(matrix, target):
    rows = [list(row) + [value] for row, value in zip(matrix, target)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(rows[row][col]))
        if abs(rows[pivot][col]) < 1e-10:
            raise ValueError('轨迹接近直线')
        rows[col], rows[pivot] = rows[pivot], rows[col]
        factor = rows[col][col]
        rows[col] = [value / factor for value in rows[col]]
        for row in range(3):
            if row != col:
                factor = rows[row][col]
                rows[row] = [a - factor * b for a, b in zip(rows[row], rows[col])]
    return [row[3] for row in rows]


def fit_circle(points):
    """Conservative stdlib circle fit; reject short, noisy or reversing arcs."""
    empty = {'turn_radius_m': None, 'fit_rmse_m': None, 'arc_coverage_deg': None,
             'fit_quality': 'insufficient', 'fit_reason': '有效轨迹不足 12 点'}
    if len(points) < 12:
        return empty
    ox = statistics.fmean(point[0] for point in points)
    oy = statistics.fmean(point[1] for point in points)
    scale = math.sqrt(statistics.fmean((x - ox)**2 + (y - oy)**2 for x, y in points))
    if scale < .01:
        return dict(empty, fit_reason='位移不足，无法可靠拟合')
    normalized = [((x - ox) / scale, (y - oy) / scale) for x, y in points]
    def solve(selected):
        design = [(2*x, 2*y, 1.) for x, y in selected]
        target = [x*x + y*y for x, y in selected]
        matrix = [[sum(row[i]*row[j] for row in design) for j in range(3)] for i in range(3)]
        right = [sum(row[i]*value for row, value in zip(design, target)) for i in range(3)]
        cx, cy, c = _solve3(matrix, right)
        radius = math.sqrt(max(0., c + cx*cx + cy*cy))
        return cx, cy, radius
    try:
        cx, cy, radius = solve(normalized)
        residuals = [abs(math.hypot(x-cx, y-cy)-radius) * scale for x, y in normalized]
        median = statistics.median(residuals)
        gate = max(.015, min(.08, median * 3.5))
        selected = [p for p, error in zip(normalized, residuals) if error <= gate]
        if len(selected) < max(12, .7 * len(normalized)):
            return dict(empty, fit_reason='位置跳点过多')
        cx, cy, radius = solve(selected)
        angles = [math.atan2(y-cy, x-cx) for x, y in selected]
        steps = [wrap(math.degrees(b-a)) for a, b in zip(angles, angles[1:])]
        coverage = abs(sum(steps))
        error = math.sqrt(statistics.fmean((math.hypot(x-cx, y-cy)-radius)**2 for x, y in selected)) * scale
        radius *= scale
        detail = dict(empty, fit_rmse_m=error, arc_coverage_deg=coverage,
                      fit_sample_count=len(points), fit_inlier_count=len(selected))
        if coverage < 45:
            return dict(detail, fit_reason='圆弧不足 45°，半径数据不足')
        if not .04 <= radius <= 5:
            return dict(detail, fit_reason='拟合半径超出可信范围')
        if sum(abs(step) for step in steps) > coverage * 1.5 + 10:
            return dict(detail, fit_reason='转弯方向反复，不能作为单向转弯标定')
        if error > max(.02, .10 * radius):
            return dict(detail, fit_quality='poor', fit_reason='圆弧残差过大')
        return dict(detail, turn_radius_m=radius, fit_quality='good', fit_reason='',
                    fit_direction='right' if sum(steps) > 0 else 'left')
    except (ValueError, ZeroDivisionError):
        return dict(empty, fit_reason='轨迹接近直线或不足以拟合圆弧')


class DynamicsRecorder:
    def __init__(self, station, vision, output_dir=None):
        self.station, self.vision = station, vision
        self.output_dir = Path(output_dir) if output_dir is not None else None
        self.lock = threading.RLock()
        self.generation = 0
        self.active = False
        self.saved = False
        self.phase = 'idle'
        self.path = None
        self.error = ''
        self.segment_id = ''
        self.settings = {}
        self.started = self.started_mono = self.ended = None
        self.files = {}
        self.samples = {'vision': 0, 'fish': 0}
        self.events = 0
        self.recent = deque(maxlen=600)
        self.recent_events = deque(maxlen=100)
        self.summary = {}
        self._stop = threading.Event()
        self._threads = []

    def start(self, settings):
        with self.lock:
            if self.active:
                raise ValueError('请先完成当前测段，再开始下一段')
            if self.path and not self.saved:
                raise ValueError('上一段尚未成功保存，请先重试完成本段')
            base = self.output_dir or desktop_directory() / 'RobotFish-Dynamics'
            base.mkdir(parents=True, exist_ok=True)
            self.settings = _clean(dict(settings))
            self.initial_settings = dict(self.settings)
            self.segment_id = uuid.uuid4().hex[:12]
            stamp = datetime.now().strftime('%Y%m%d-%H%M%S-%f')[:-3]
            mode = {1: 'tail', 2: 'prop', 3: 'hybrid'}.get(settings.get('mode'), 'unknown')
            motion = str(settings.get('motion', 'unknown'))
            if motion not in ('straight', 'left', 'right'):
                motion = 'unknown'
            values = '-'.join(str(number(settings.get(key)) or 0).replace('.', 'p') for key in PARAMETERS)
            path = base / f'{stamp}-{mode}-{motion}-{values}-{self.segment_id[:4]}'
            path.mkdir()
            self.path = path
            self.started, self.started_mono = time.time(), time.monotonic()
            self.ended = None
            self.phase, self.active, self.saved, self.error = 'pre', True, False, ''
            self.samples, self.events, self.summary = {'vision': 0, 'fish': 0}, 0, {}
            self.recent.clear(); self.recent_events.clear()
            self._last_key = {}; self._last_fresh = {}; self._source_problem = {}
            self._latest = {}; self._history = deque(maxlen=40)
            self._first_position = self._first_yaw = None
            self._motion_seen = False; self._zero_seen = False
            self._drive_at = self._release_at = None
            self._pending = deque()
            self.generation += 1
            generation = self.generation
            try:
                self.files = {name: (path / name).open('a', encoding='utf-8', newline='')
                              for name in ('fish.jsonl', 'vision.jsonl', 'events.jsonl', 'samples.csv')}
                self.writer = csv.DictWriter(self.files['samples.csv'], fieldnames=FIELDS)
                self.writer.writeheader(); self.files['samples.csv'].flush()
                _atomic_json(path / 'metadata.json', self._metadata())
                self.note_event('segment_start', {'settings': self.settings,
                    'operator_confirmed_stationary': bool(settings.get('stationary_confirmed')),
                    'operator_confirmed_head_calibrated': bool(settings.get('head_confirmed'))})
            except OSError as exc:
                self.active = False
                self.error = str(exc)
                for output in self.files.values(): output.close()
                self.files = {}
                raise
            self._stop = threading.Event()
            self._threads = [threading.Thread(target=self._sample_loop, args=(source, generation, self._stop),
                name='dynamics-' + source, daemon=True) for source in ('fish', 'vision')]
            for thread in self._threads: thread.start()
            return self.state()

    def _metadata(self):
        return {'schema': 1, 'segment_id': self.segment_id, 'started_utc': utc(self.started),
            'started_monotonic_s': self.started_mono, 'finished_utc': utc(self.ended) if self.ended else None,
            'settings': self.settings, 'initial_settings': self.initial_settings,
            'sample_period_s': .1, 'sources': dict(self.samples),
            'event_count': self.events, 'save_complete': self.saved, 'error': self.error,
            'scene': {'width_m': self.settings.get('pool_width_m', 3.),
                'height_m': self.settings.get('pool_height_m', 1.6),
                'camera_height_m': self.settings.get('camera_height_m', 2.67),
                'metric_basis': 'four_point_perspective_calibration; camera height is metadata only'},
            'coordinates': {'x': 'right', 'y': 'down', 'yaw_deg': 'world counterclockwise; zero +x',
                'imu_yaw_cw_deg':'mapped body yaw; raw_yaw_deg and filtered_yaw_deg remain sensor axes'},
            'notes': ['Actuator values are commanded driver outputs, not measured thrust or position.',
                'Source snapshots contain original timestamps. CSV joins latest fish data; use fish_age_ms.',
                'speed_1s_mps uses camera timestamps and about one second of metric displacement.',
                'drive/coast phases follow user requests; output_zero_observed is separate telemetry evidence.',
                'motion_start_estimated uses 1 cm displacement or 3 degree yaw change, not a sensor edge.',
                'Full JSONL and CSV are flushed per sample; UI samples are bounded and may omit old rows.']}

    def state(self):
        with self.lock:
            return {'active': self.active, 'phase': self.phase, 'saved': self.saved,
                'path': str(self.path) if self.path else '', 'error': self.error,
                'segment_id': self.segment_id, 'settings': dict(self.settings),
                'elapsed_s': max(0., (self.ended or time.time()) - self.started) if self.started else 0.,
                'samples': dict(self.samples), 'events': self.events,
                'recent_samples': list(self.recent), 'recent_events': list(self.recent_events),
                'summary': dict(self.summary)}

    def _record(self, filename, item):
        self._pending.append((filename, item))
        self._flush_pending()

    def _flush_pending(self):
        while self._pending:
            filename, item = self._pending[0]
            output = self.files.get(filename)
            if output is None:
                raise OSError('采集文件未打开')
            offset = output.tell()
            try:
                if filename == 'samples.csv':
                    self.writer.writerow({key: item.get(key) for key in FIELDS})
                else:
                    output.write(_json(item) + '\n')
                output.flush()
            except OSError:
                try: output.seek(offset); output.truncate()
                except OSError: pass
                raise
            self._pending.popleft()

    def note_event(self, kind, payload=None):
        with self.lock:
            if not self.path or self.saved:
                return self.state()
            now = time.time()
            item = {'kind': str(kind), 'utc': utc(now), 'monotonic_s': time.monotonic(),
                    'time_s': now - self.started, 'phase': self.phase, 'payload': _clean(_redact(payload or {}))}
            self.events += 1
            self.recent_events.append(item)
            try: self._record('events.jsonl', item)
            except OSError as exc:
                self.error = '数据写入失败，保留待写记录，请重试完成本段：' + str(exc)
            return self.state()

    def drive(self, settings=None):
        with self.lock:
            if not self.active: raise ValueError('请先开始本段')
            if self.error: raise ValueError(self.error)
            if settings:
                changed = {key: {'before': self.settings.get(key), 'after': settings.get(key)}
                    for key in ('mode', 'motion', *PARAMETERS, 'baseline')
                    if key in settings and settings[key] != self.settings.get(key)}
                if changed: self.note_event('parameters_changed', changed)
                self.settings.update(_clean(settings))
            self.phase = 'drive'
            self._drive_at = time.time()
            self._motion_seen = False
            baseline = next((row for row in reversed(self.recent) if row.get('position_valid')), None)
            if baseline:
                self._first_position = baseline['x_m'], baseline['y_m']
                self._first_yaw = baseline['yaw_deg']
            self.note_event('drive_phase', {'settings': dict(self.settings)})
            if self.error:
                raise ValueError(self.error)
            return self.state()

    def release(self):
        with self.lock:
            if self.active and self.phase != 'coast':
                self.phase = 'coast'
                self._release_at = time.time()
                self._zero_seen = False
                self.note_event('release_phase')
            return self.state()

    def _sample_loop(self, source, generation, stop):
        reader = self.station.read if source == 'fish' else self.vision.state
        while not stop.is_set():
            began = time.monotonic()
            try:
                snapshot = reader()
                with self.lock:
                    if not self.active or generation != self.generation: return
                    self._ingest(source, snapshot)
            except Exception as exc:
                with self.lock:
                    if generation == self.generation and self.active:
                        message = str(exc)
                        if self._source_problem.get(source) != message:
                            self._source_problem[source] = message
                            self.note_event('source_error', {'source': source, 'error': message})
            stop.wait(max(0., .1 - (time.monotonic() - began)))

    def _ingest(self, source, snapshot):
        now, monotonic = time.time(), time.monotonic()
        value = snapshot.get('result') or {} if source == 'vision' else snapshot
        if source == 'vision':
            stamp = value.get('camera_sample_time', (value.get('position') or {}).get('sample_time'))
            key = (value.get('frame'), stamp, snapshot.get('running'))
            valid = bool(snapshot.get('running') and finite(stamp) and stamp >= self.started and now-stamp <= .8)
        else:
            raw = snapshot.get('diagnostics') or {}
            stamp = snapshot.get('checked_at')
            key = (raw.get('uptime_ms'), raw.get('telemetry_sequence'), stamp if not raw else None, snapshot.get('online'))
            valid = bool(snapshot.get('online') and finite(stamp) and now-stamp <= .8)
        changed = self._last_key.get(source) != key
        if changed and valid:
            self._last_fresh[source] = now
            if source in self._source_problem:
                self.note_event('source_restored', {'source': source})
                self._source_problem.pop(source, None)
        elif not valid or now-self._last_fresh.get(source, self.started) > .8:
            if self._source_problem.get(source) != 'stale':
                self._source_problem[source] = 'stale'
                self.note_event('source_gap', {'source': source, 'last_source_time': stamp})
            if source == 'vision': self._history.clear()
        if not changed: return
        self._last_key[source] = key
        envelope = {'source': source, 'utc': utc(now), 'host_time': now,
            'monotonic_s': monotonic, 'time_s': now-self.started, 'phase': self.phase,
            'fresh': valid, 'snapshot': _redact(snapshot)}
        self._record(source + '.jsonl', envelope)
        self.samples[source] += 1
        self._latest[source] = (now, snapshot)
        row = self._flatten(source, envelope, valid)
        self._record('samples.csv', row)
        self.recent.append(row)
        if source == 'vision' and valid: self._motion_events(row)
        if source == 'fish' and valid: self._output_events(row, snapshot)

    def _flatten(self, source, envelope, valid):
        fish_at, fish = self._latest.get('fish', (None, {}))
        result = (self._latest.get('vision', (None, {}))[1].get('result') or {}) if source == 'vision' else {}
        position, optical, body = result.get('position') or {}, result.get('optical') or {}, result.get('body_yaw') or {}
        imu, attitude, act = fish.get('imu') or {}, fish.get('attitude_control') or {}, fish.get('actuators') or {}
        native = fish.get('diagnostics') or {}
        allocation = fish.get('allocation') or native.get('allocation') or {}
        yaw, yaw_source = None, None
        if optical.get('valid') and finite(optical.get('world_deg')):
            yaw, yaw_source = optical['world_deg'], 'optical_world'
        elif body.get('usable') and finite(body.get('yaw_deg')):
            zero = (result.get('initial_heading') or {}).get('visual_zero_world_deg')
            if finite(zero): yaw, yaw_source = wrap(zero+body['yaw_deg']), 'imu_zero_world'
        row = {key: envelope[key] for key in ('utc', 'monotonic_s', 'time_s', 'source', 'phase')}
        row.update(frame=result.get('frame'), camera_sample_time=result.get('camera_sample_time', position.get('sample_time')),
            camera_receipt_time=result.get('camera_receipt_time'), uptime_ms=native.get('uptime_ms'),
            fish_checked_at=fish.get('checked_at'), fish_age_ms=(envelope['host_time']-fish_at)*1000 if fish_at else None,
            imu_age_ms=imu.get('age_ms'), imu_valid=imu.get('valid'), x_m=position.get('x_m'), y_m=position.get('y_m'),
            position_valid=bool(valid and position.get('valid')), yaw_deg=yaw, yaw_source=yaw_source,
            imu_yaw_cw_deg=(fish.get('attitude_control') or {}).get('body_yaw_deg'), raw_yaw_deg=imu.get('sensor_yaw'),
            filtered_yaw_deg=imu.get('yaw'), vision_speed_mps=(result.get('velocity') or {}).get('speed_mps'),
            left_pct=act.get('left_esc'), right_pct=act.get('right_esc'), tail1_deg=act.get('tail1'), tail2_deg=act.get('tail2'),
            yaw_p=attitude.get('yaw_p', attitude.get('p')), yaw_i=attitude.get('yaw_i', attitude.get('i')),
            yaw_d=attitude.get('yaw_d', attitude.get('d')), yaw_output_pct=attitude.get('yaw_turn_pct', attitude.get('yaw_correction_pct')),
            safety=fish.get('safety_mode'), owner=fish.get('owner'))
        for key in PARAMETERS:
            row[key] = self.settings.get(key)
            native_key = {'prop_surge_pct': 'written_common_pct',
                          'prop_diff_pct': 'written_differential_pct'}.get(key, key)
            row['output_' + key] = allocation.get(native_key)
        stamp = row['camera_sample_time']
        if row['position_valid'] and all(finite(row.get(key)) for key in ('x_m', 'y_m', 'camera_sample_time')):
            if self._history and stamp-self._history[-1][0] > .35: self._history.clear()
            if not self._history or stamp > self._history[-1][0]:
                previous = min(self._history, key=lambda entry: abs(stamp-entry[0]-1.)) if self._history else None
                if previous and .8 <= stamp-previous[0] <= 1.2:
                    dt = stamp-previous[0]
                    row['speed_1s_mps'] = math.hypot(row['x_m']-previous[1], row['y_m']-previous[2])/dt
                    if finite(yaw) and finite(previous[3]): row['yaw_rate_dps'] = wrap(yaw-previous[3])/dt
                self._history.append((stamp, row['x_m'], row['y_m'], yaw))
        return _clean(row)

    def _motion_events(self, row):
        if not row['position_valid']: return
        if self.phase == 'pre' or self._first_position is None:
            self._first_position = row['x_m'], row['y_m']
            self._first_yaw = row['yaw_deg']
        if self.phase == 'drive' and not self._motion_seen:
            distance = math.hypot(row['x_m']-self._first_position[0], row['y_m']-self._first_position[1])
            angle = abs(wrap(row['yaw_deg']-self._first_yaw)) if finite(row['yaw_deg']) and finite(self._first_yaw) else 0.
            if distance >= .01 or angle >= 3:
                self._motion_seen = True
                self.note_event('motion_start_estimated', {'distance_m': distance, 'angle_deg': angle,
                    'camera_sample_time': row['camera_sample_time'], 'frame': row['frame']})

    def _output_events(self, row, snapshot):
        if self.phase != 'coast' or self._zero_seen: return
        allocation = snapshot.get('dynamics_calibration') or (snapshot.get('diagnostics') or {}).get('dynamics_calibration') or {}
        # STOP is confirmed by fresh telemetry: both ESCs zero and calibration
        # disabled (or explicit all-zero independent outputs), not by HTTP ACK.
        esc_zero = all(finite(row.get(key)) and abs(row[key]) < .01 for key in ('left_pct', 'right_pct'))
        allocation_zero = allocation.get('active') is False or (bool(allocation) and
            all(finite(allocation.get(key)) and abs(allocation[key]) < .01 for key in PARAMETERS))
        tail_neutral = all(finite(row.get(key)) and abs(row[key]-90.) < .5 for key in ('tail1_deg', 'tail2_deg'))
        if esc_zero and allocation_zero and tail_neutral:
            self._zero_seen = True
            self.note_event('output_zero_observed', {'uptime_ms': row['uptime_ms'],
                'tail1_deg': row['tail1_deg'], 'tail2_deg': row['tail2_deg'], 'allocation': allocation})

    def finish(self):
        with self.lock:
            if not self.path: raise ValueError('尚未开始测段')
            if self.saved: return self.state()
            if self.active:
                self.note_event('segment_finish_requested')
                self.active = False
                self.phase = 'finished'
                self.ended = time.time()
                self._stop.set()
            try:
                self._flush_pending()
                for output in self.files.values():
                    output.flush(); os.fsync(output.fileno())
                self.summary = self._summarize()
                _atomic_json(self.path / 'summary.json', self.summary)
                self.error = ''
                self.saved = True
                _atomic_json(self.path / 'metadata.json', self._metadata())
                self._update_table()
                for output in self.files.values(): output.close()
                self.files = {}
            except (OSError, ValueError, csv.Error) as exc:
                self.saved = False
                self.error = '保存未完成，请重试完成本段：' + str(exc)
            return self.state()

    def _summarize(self):
        events = []
        with (self.path / 'events.jsonl').open(encoding='utf-8') as source:
            for line in source:
                event = json.loads(line)
                if event['kind'] in ('drive_phase', 'parameters_changed', 'drive_ack', 'stop_ack', 'motion_start_estimated', 'output_zero_observed', 'source_gap', 'source_error'):
                    events.append(event)
        recipes = {}
        for event in events:
            if event['kind'] == 'drive_phase':
                settings = event['payload'].get('settings', {})
                recipe = {key: settings.get(key) for key in ('mode', 'motion', *PARAMETERS, 'baseline')}
                recipes.setdefault(tuple(recipe.values()), recipe)
        event_time = lambda kind: next((item['time_s'] for item in events if item['kind'] == kind), None)
        zero_time = event_time('output_zero_observed')
        points, drive_points, speeds, turns = [], [], [], []
        first = last = first_coast = last_drive = None
        coast_distance = residual_turn = 0.
        previous_coast = None
        confirmed_first = confirmed_previous = None
        confirmed_distance = confirmed_turn = 0.
        steady_window = deque(maxlen=21)
        n = 0
        # Decimate only derived geometry when huge; all original samples remain on disk.
        stride = max(1, math.ceil(self.samples['vision']/10000))
        with (self.path / 'samples.csv').open(encoding='utf-8', newline='') as source:
            for raw in csv.DictReader(source):
                if raw['source'] != 'vision' or raw['position_valid'] != 'True': continue
                row = {key: number(raw.get(key)) for key in ('x_m', 'y_m', 'yaw_deg', 'time_s', 'camera_sample_time', 'speed_1s_mps', 'yaw_rate_dps')}
                if row['x_m'] is None or row['y_m'] is None: continue
                first = first or row; last = row; n += 1
                if n % stride == 0: points.append((row['x_m'], row['y_m']))
                if raw['phase'] == 'drive':
                    if steady_window and (row['camera_sample_time'] is None or
                            steady_window[-1]['camera_sample_time'] is None or
                            row['camera_sample_time']-steady_window[-1]['camera_sample_time'] > .35):
                        steady_window.clear()
                    steady_window.append(row)
                    last_drive = row
                    if n % stride == 0:
                        drive_points.append((row['x_m'], row['y_m']))
                        if row['speed_1s_mps'] is not None: speeds.append(row['speed_1s_mps'])
                        if row['yaw_rate_dps'] is not None: turns.append(row['yaw_rate_dps'])
                if raw['phase'] == 'coast':
                    first_coast = first_coast or last_drive or row
                    previous_coast = previous_coast or first_coast
                    dt = row['camera_sample_time']-previous_coast['camera_sample_time'] if row['camera_sample_time'] is not None and previous_coast['camera_sample_time'] is not None else 99
                    if 0 < dt <= .35:
                        coast_distance += math.hypot(row['x_m']-previous_coast['x_m'], row['y_m']-previous_coast['y_m'])
                        if row['yaw_deg'] is not None and previous_coast['yaw_deg'] is not None:
                            residual_turn += wrap(row['yaw_deg']-previous_coast['yaw_deg'])
                    previous_coast = row
                    if zero_time is not None and row['camera_sample_time'] is not None and row['camera_sample_time']-self.started >= zero_time:
                        confirmed_first = confirmed_first or row
                        confirmed_previous = confirmed_previous or row
                        dt = row['camera_sample_time']-confirmed_previous['camera_sample_time']
                        if 0 < dt <= .35:
                            confirmed_distance += math.hypot(row['x_m']-confirmed_previous['x_m'], row['y_m']-confirmed_previous['y_m'])
                            if row['yaw_deg'] is not None and confirmed_previous['yaw_deg'] is not None:
                                confirmed_turn += wrap(row['yaw_deg']-confirmed_previous['yaw_deg'])
                        confirmed_previous = row
        summary = {'speed_mps': statistics.median(speeds) if len(speeds) >= 10 else None,
            'yaw_rate_dps': statistics.median(turns) if len(turns) >= 10 else None,
            'speed_estimate_method': 'median one-second displacement during drive; stability checked separately',
            'steady_speed_mps': None, 'steady_yaw_rate_dps': None,
            'valid_position_samples': n, 'duration_s': (self.ended or time.time())-self.started,
            'displacement_m': None, 'lateral_drift_m': None,
            'coast_distance_m': coast_distance if first_coast else None,
            'coast_displacement_m': math.hypot(last['x_m']-first_coast['x_m'], last['y_m']-first_coast['y_m']) if first_coast and last else None,
            'residual_turn_deg': residual_turn if first_coast else None,
            'coast_measurement_basis': 'release_request; output_zero_observed time is reported separately',
            'coast_distance_note': 'sample-to-sample path length may be inflated by tracking jitter; compare coast_displacement_m and raw trajectory',
            'confirmed_coast_distance_m': confirmed_distance if confirmed_first else None,
            'confirmed_residual_turn_deg': confirmed_turn if confirmed_first else None,
            'drive_ack_time_s': event_time('drive_ack'), 'motion_start_time_s': event_time('motion_start_estimated'),
            'stop_ack_time_s': event_time('stop_ack'), 'output_zero_time_s': event_time('output_zero_observed'),
            'source_gap_count': sum(item['kind'] in ('source_gap', 'source_error') for item in events),
            'fit_geometry_decimation': stride,
            'drive_count': sum(item['kind'] == 'drive_phase' for item in events),
            'mixed_settings': len(recipes) > 1, 'parameter_sets': list(recipes.values()),
            'warnings': [],
            'steady_reason': '数据不足或驱动速度尚未稳定', **fit_circle(drive_points)}
        if first and last:
            dx, dy = last['x_m']-first['x_m'], last['y_m']-first['y_m']
            summary['displacement_m'] = math.hypot(dx, dy)
            if finite(first['yaw_deg']):
                heading = math.radians(first['yaw_deg'])
                summary['lateral_drift_m'] = abs(dx*math.sin(heading)+dy*math.cos(heading))
        continuous = (len(steady_window) >= 20 and steady_window[0]['camera_sample_time'] is not None and
            steady_window[-1]['camera_sample_time'] is not None and
            steady_window[-1]['camera_sample_time']-steady_window[0]['camera_sample_time'] >= 1.8)
        stable_speeds = [row['speed_1s_mps'] for row in steady_window if row['speed_1s_mps'] is not None]
        stable_turns = [row['yaw_rate_dps'] for row in steady_window if row['yaw_rate_dps'] is not None]
        if continuous and len(stable_speeds) >= 20:
            window = stable_speeds
            median = statistics.median(window)
            if statistics.pstdev(window) <= max(.01, abs(median)*.2):
                summary['steady_speed_mps'] = median
                summary['steady_reason'] = ''
        if continuous and len(stable_turns) >= 20:
            window = stable_turns
            median = statistics.median(window)
            if statistics.pstdev(window) <= max(2., abs(median)*.2):
                summary['steady_yaw_rate_dps'] = median
        if summary['mixed_settings']:
            warning = '本段包含不同驱动参数，不能作为单一配方标定；请按事件时间分别分析原始数据'
            summary['warnings'].append(warning)
            for key in ('speed_mps', 'yaw_rate_dps', 'steady_speed_mps', 'steady_yaw_rate_dps', 'turn_radius_m'):
                summary[key] = None
            summary.update(steady_reason=warning, fit_quality='mixed_settings', fit_reason=warning)
        return summary

    def _update_table(self):
        path = self.path.parent / 'calibration-table.csv'
        keys = ('segment_id', 'path', 'mode', 'motion', *PARAMETERS, 'mixed_settings', 'drive_count', 'speed_mps',
            'steady_speed_mps', 'yaw_rate_dps', 'steady_yaw_rate_dps', 'turn_radius_m',
            'fit_quality', 'displacement_m', 'lateral_drift_m', 'coast_distance_m', 'residual_turn_deg')
        rows = []
        if path.is_file():
            with path.open(encoding='utf-8-sig', newline='') as source:
                rows = [row for row in csv.DictReader(source) if row.get('segment_id') != self.segment_id]
        row = {'segment_id': self.segment_id, 'path': str(self.path), **self.settings, **self.summary}
        if self.summary.get('mixed_settings'):
            row.update(mode='mixed', motion='mixed', **{key: None for key in PARAMETERS})
        rows.append(row)
        temporary = path.with_suffix('.csv.pending')
        with temporary.open('w', encoding='utf-8-sig', newline='') as output:
            writer = csv.DictWriter(output, fieldnames=keys, extrasaction='ignore')
            writer.writeheader(); writer.writerows(rows); output.flush(); os.fsync(output.fileno())
        temporary.replace(path)

    def close(self):
        """Stop passive sampling; retain unfinished data for recovery."""
        with self.lock:
            self._stop.set()
            self.active = False
            for output in self.files.values():
                try: output.flush(); output.close()
                except OSError: pass
            self.files = {}


Recorder = DynamicsRecorder
