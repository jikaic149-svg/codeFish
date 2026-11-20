"""纯几何轨迹引导。

水池坐标沿用视觉标定：x 向画面右侧增加，y 向画面下方增加。
本模块不依赖 OpenCV、UI、网络或固件，只输出鱼体坐标下的引导误差。
"""

from __future__ import annotations

from collections import deque
import math

import numpy as np


def _unit(vector):
    value = np.asarray(vector, dtype=np.float64).reshape(2)
    length = float(np.linalg.norm(value))
    if not np.isfinite(value).all() or length <= 1e-9:
        raise ValueError("方向向量无效")
    return value / length


def wrap_to_pi(angle):
    """Return a scalar angle in the half-open interval [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def compensate_camera_latency(
    position, velocity, latency_s, max_prediction_m=0.12
):
    """按速度向前预测相机延迟，并限制最大补偿距离。"""
    current = np.asarray(position, dtype=np.float64).reshape(2)
    motion = np.asarray(velocity, dtype=np.float64).reshape(2)
    if not np.isfinite(current).all() or not np.isfinite(motion).all():
        raise ValueError("位置或速度无效")
    offset = motion * max(0.0, float(latency_s))
    offset_length = float(np.linalg.norm(offset))
    limit = max(0.0, float(max_prediction_m))
    if offset_length > limit and offset_length > 1e-9:
        offset *= limit / offset_length
    return current + offset


def resample_polyline(points, spacing_m=0.03):
    """按累计弧长等距重采样折线，保留首尾点并移除重复点。"""
    path = np.asarray(points, dtype=np.float64).reshape((-1, 2))
    if len(path) < 2 or not np.isfinite(path).all():
        raise ValueError("轨迹至少需要两个有限坐标点")
    spacing_m = float(spacing_m)
    if spacing_m <= 0.0:
        raise ValueError("轨迹采样间距必须大于零")

    kept = [path[0]]
    for point in path[1:]:
        if float(np.linalg.norm(point - kept[-1])) > 1e-6:
            kept.append(point)
    path = np.asarray(kept, dtype=np.float64)
    if len(path) < 2:
        raise ValueError("轨迹有效长度为零")

    lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    total = float(cumulative[-1])
    samples = np.arange(0.0, total, spacing_m, dtype=np.float64)
    if len(samples) == 0 or total - float(samples[-1]) > 1e-9:
        samples = np.append(samples, total)
    else:
        samples[-1] = total

    output = np.empty((len(samples), 2), dtype=np.float64)
    segment = 0
    for index, distance in enumerate(samples):
        while segment + 1 < len(cumulative) - 1 and distance > cumulative[segment + 1]:
            segment += 1
        segment_length = lengths[segment]
        ratio = 0.0 if segment_length <= 1e-9 else (
            distance - cumulative[segment]
        ) / segment_length
        output[index] = path[segment] + ratio * (path[segment + 1] - path[segment])
    return output


def round_polyline(points, spacing_m, desired_radius_m, max_offset_m):
    """Round only when the requested radius fits; never tighten a constrained U."""
    from curve_speed import _simplify

    original = resample_polyline(points, spacing_m)

    def curvature_metrics(path):
        segments = np.diff(path, axis=0)
        a, b = segments[:-1], segments[1:]
        denominator = np.linalg.norm(a, axis=1)*np.linalg.norm(b, axis=1)*np.linalg.norm(a+b, axis=1)
        if np.any(denominator < 1e-12):
            return None, 0.
        curvature = 2*np.abs(a[:, 0]*b[:, 1]-a[:, 1]*b[:, 0])/denominator
        maximum = float(np.max(curvature)) if len(curvature) else 0.
        return maximum, 1/maximum if maximum > 1e-6 else None

    source_curvature, source_radius = curvature_metrics(original)
    detail = dict(source="GEOMETRIC", desired_radius_m=desired_radius_m,
                  max_offset_m=max_offset_m, rounded=False, limited_corners=0,
                  status="unchanged", reason="already_smooth",
                  original_min_radius_m=source_radius, achieved_min_radius_m=source_radius,
                  max_curvature_per_m=source_curvature)
    if desired_radius_m <= 0 or max_offset_m <= 0:
        detail['reason'] = 'disabled'
        return original, detail
    if source_curvature is None:
        detail.update(status='unachievable', reason='degenerate_reversal_kept_original')
        return original, detail
    if len(original) < 3 or source_curvature <= 1 / desired_radius_m:
        return original, detail
    simplify_offset = min(.02, max_offset_m / 4)
    path = _simplify(original, simplify_offset)
    delta = np.diff(path, axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    units = delta / lengths[:, None]
    result = [path[0]]
    for i in range(1, len(path) - 1):
        u, v = units[i - 1], units[i]
        angle = math.atan2(float(u[0]*v[1] - u[1]*v[0]), float(u @ v))
        if abs(angle) < 1e-4 or abs(angle) > math.pi - .01:
            result.append(path[i])
            detail['limited_corners'] += int(abs(angle) > math.pi - .01)
            continue
        tangent = math.tan(abs(angle) / 2)
        available = min(.49 * min(lengths[i-1], lengths[i]) / tangent,
                        (max_offset_m-simplify_offset) / (1 / math.cos(angle / 2) - 1))
        if available < desired_radius_m - 1e-6:
            detail['limited_corners'] += 1
            continue
        radius = desired_radius_m
        entry = path[i] - radius * tangent * u
        centre = entry + math.copysign(radius, angle) * np.array([-u[1], u[0]])
        start = math.atan2(entry[1] - centre[1], entry[0] - centre[0])
        angles = np.linspace(start, start + angle, max(3, math.ceil(abs(angle)*radius/spacing_m)+1))
        result.extend(centre + radius * np.column_stack((np.cos(angles), np.sin(angles))))
        detail['rounded'] = True
    if detail['limited_corners']:
        detail.update(rounded=False, status='unachievable',
                      reason='radius_exceeds_local_space_or_offset_kept_original')
        return original, detail
    result.append(path[-1])
    result = np.asarray(result)
    achieved_curvature, achieved_radius = curvature_metrics(resample_polyline(result, spacing_m))
    if achieved_curvature is None or achieved_curvature > 1.05 / desired_radius_m:
        detail.update(rounded=False, status='unachievable',
                      reason='discrete_curve_constraint_kept_original')
        return original, detail
    detail.update(status='rounded' if detail['rounded'] else 'unchanged',
                  reason='requested_radius_fits' if detail['rounded'] else 'already_smooth',
                  achieved_min_radius_m=achieved_radius, max_curvature_per_m=achieved_curvature)
    return result, detail


class VelocityEstimator:
    """用短窗口直线拟合估计平移速度，抑制鱼尾逐帧摆动和单帧尖峰。"""

    def __init__(
        self,
        history_seconds=0.40,
        min_span_seconds=0.12,
        min_samples=3,
        blend=0.45,
        max_speed_mps=0.80,
        reject_overspeed=False,
        robust_floor_m=0.,
    ):
        self.history_seconds = float(history_seconds)
        self.min_span_seconds = float(min_span_seconds)
        self.min_samples = int(min_samples)
        self.blend = float(np.clip(blend, 0.0, 1.0))
        self.max_speed_mps = float(max_speed_mps)
        self.reject_overspeed = reject_overspeed
        self.robust_floor_m = robust_floor_m
        self.reset()

    def reset(self, position=None, timestamp=None):
        self._history = deque()
        self.velocity = None
        if position is not None and timestamp is not None:
            self._history.append(
                (float(timestamp), np.asarray(position, dtype=np.float64).reshape(2))
            )

    def update(self, position, timestamp):
        now = float(timestamp)
        current = np.asarray(position, dtype=np.float64).reshape(2)
        if not np.isfinite(current).all() or not math.isfinite(now):
            return None if self.velocity is None else self.velocity.copy()

        if self._history and now <= self._history[-1][0]:
            return None if self.velocity is None else self.velocity.copy()
        self._history.append((now, current.copy()))
        while self._history and now - self._history[0][0] > self.history_seconds:
            self._history.popleft()
        if len(self._history) < self.min_samples:
            return None if self.velocity is None else self.velocity.copy()

        times = np.asarray([sample[0] for sample in self._history], dtype=np.float64)
        positions = np.asarray([sample[1] for sample in self._history], dtype=np.float64)
        span = float(times[-1] - times[0])
        if span < self.min_span_seconds:
            return None if self.velocity is None else self.velocity.copy()

        centred_t = times - float(np.mean(times))
        denominator = float(np.dot(centred_t, centred_t))
        if denominator <= 1e-9:
            return None if self.velocity is None else self.velocity.copy()
        raw_velocity = np.sum(
            centred_t[:, None] * positions, axis=0
        ) / denominator
        if self.robust_floor_m and len(times)>=5:
            # One Huber re-fit: de-weight isolated position spikes without
            # lengthening the window or imposing an acceleration limit.
            residual=positions-positions.mean(axis=0)-centred_t[:,None]*raw_velocity
            errors=np.linalg.norm(residual-np.median(residual,axis=0),axis=1)
            cutoff=max(self.robust_floor_m,2.5*float(np.median(errors)))
            weights=cutoff/np.maximum(cutoff,errors)
            dt=times-np.average(times,weights=weights)
            raw_velocity=(weights*dt) @ positions / np.dot(weights,dt*dt)
        raw_speed = float(np.linalg.norm(raw_velocity))
        if not np.isfinite(raw_velocity).all():
            return None if self.velocity is None else self.velocity.copy()

        # 超过机器鱼物理可能范围的结果来自色块跳变；不让它污染制动判断。
        if raw_speed > self.max_speed_mps:
            if self.reject_overspeed:
                self.reset(current, now)
                return None
            if self.velocity is not None:
                return self.velocity.copy()
            raw_velocity *= self.max_speed_mps / max(raw_speed, 1e-9)

        if self.velocity is None:
            self.velocity = raw_velocity
        else:
            self.velocity = (
                self.blend * raw_velocity + (1.0 - self.blend) * self.velocity
            )
        return self.velocity.copy()


class PathGuidance:
    """Metric path association, heading reference and forward-speed target."""

    @classmethod
    def from_config(cls, control, **options):
        settings = dict(control['guidance'])
        return cls(k_f=settings.pop('cross_track_gain'),
                   min_speed_target_mps=settings.pop('min_speed_mps'),
                   tangent_tracking=control['tangent_tracking'], **settings, **options)

    def __init__(
        self,
        spacing_m=0.03,
        lookahead_m=0.24,
        min_lookahead_m=0.10,
        full_attraction_error_m=0.18,
        curve_preview_m=0.32,
        estimated_min_turn_radius_m=0.20,
        left_turn_radius_m=None,
        right_turn_radius_m=None,
        max_x_error_m=(1.0 / 3.0),
        arrival_radius_m=0.20,
        brake_radius_m=0.40,
        search_ahead_m=0.45,
        k_f=1.20,
        k_psi=2.40,
        k_kappa=0.80,
        k_d=0.25,
        r_max_rps=3.00,
        v_min_mps=0.05,
        cruise_speed_mps=0.30,
        min_speed_target_mps=0.12,
        curve_speed_planner=None,
        curve_heading_gain=1.1,
        max_curve_heading_deg=22.0,
        cross_track_filter_s=0.35,
        heading_reference_rate_deg_s=35.0,
        desired_turn_radius_m=0.0,
        path_smoothing_offset_m=0.15,
        curve_speed_ratio=0.68,
        tangent_tracking=None,
    ):
        self.spacing_m = float(spacing_m)
        self.lookahead_m = float(lookahead_m)
        self.min_lookahead_m = min(
            self.lookahead_m, float(min_lookahead_m)
        )
        self.full_attraction_error_m = max(
            1e-6, float(full_attraction_error_m)
        )
        self.curve_preview_m = max(self.spacing_m, float(curve_preview_m))
        # Nominal curvature scale only; this is not a measured turning envelope.
        self.estimated_min_turn_radius_m = max(
            0.01, float(estimated_min_turn_radius_m)
        )
        self.max_path_curvature_per_m = (
            1.0 / self.estimated_min_turn_radius_m
        )
        self.left_turn_radius_m = self._validated_turn_radius(
            left_turn_radius_m
        )
        self.right_turn_radius_m = self._validated_turn_radius(
            right_turn_radius_m
        )
        self.max_x_error_m = max(0.01, float(max_x_error_m))
        self.arrival_radius_m = float(arrival_radius_m)
        self.brake_radius_m = max(
            self.arrival_radius_m, float(brake_radius_m)
        )
        self.search_ahead_m = float(search_ahead_m)
        # LOS geometry produces a reference; yaw feedback comes only from IMU.
        self.k_f = max(0.0, float(k_f))
        self.k_psi = max(0.0, float(k_psi))
        self.k_kappa = max(0.0, float(k_kappa))
        self.k_d = max(0.0, float(k_d))
        self.r_max_rps = max(1e-3, float(r_max_rps))
        self.v_min_mps = max(1e-3, float(v_min_mps))
        self.cruise_speed_mps = max(0.0, float(cruise_speed_mps))
        self.min_speed_target_mps = max(0.0, float(min_speed_target_mps))
        self.body_heading = None
        self.body_yaw_rate_rps = 0.0
        self.curve_speed_planner = curve_speed_planner
        self.curve_heading_gain = max(0., float(curve_heading_gain))
        self.max_curve_heading_rad = math.radians(max(0., float(max_curve_heading_deg)))
        self.cross_track_filter_s = max(0., float(cross_track_filter_s))
        self.heading_reference_rate_rps = math.radians(max(0., float(heading_reference_rate_deg_s)))
        self.desired_turn_radius_m = max(0., float(desired_turn_radius_m))
        self.path_smoothing_offset_m = max(0., float(path_smoothing_offset_m))
        self.curve_speed_ratio = float(np.clip(curve_speed_ratio, 0., 1.))
        self.tangent_tracking = tangent_tracking
        self.clear()

    @staticmethod
    def _validated_turn_radius(radius_m):
        if radius_m is None:
            return None
        radius_m = float(radius_m)
        if not math.isfinite(radius_m) or radius_m <= 0.0:
            raise ValueError("实测转圈半径必须大于零")
        return radius_m

    def set_turn_radius(self, direction, radius_m):
        """运行中更新一侧 HYBRID 满转实测半径。"""
        direction = str(direction).upper()
        radius_m = self._validated_turn_radius(radius_m)
        if direction == "LEFT":
            self.left_turn_radius_m = radius_m
        elif direction == "RIGHT":
            self.right_turn_radius_m = radius_m
        else:
            raise ValueError(f"未知转向方向：{direction}")

    def _turn_radius_for_curvature(self, curvature_per_m):
        if curvature_per_m > 0.0 and self.left_turn_radius_m is not None:
            return self.left_turn_radius_m, "MEASURED_LEFT"
        if curvature_per_m < 0.0 and self.right_turn_radius_m is not None:
            return self.right_turn_radius_m, "MEASURED_RIGHT"
        return self.estimated_min_turn_radius_m, "ESTIMATED"

    def clear(self):
        if self.curve_speed_planner is not None:self.curve_speed_planner.reset()
        self.path = None
        self.cumulative = None
        self.progress_m = 0.0
        self.segment_index = 0
        self._turn_sign = 0.0
        self.last_result = None
        self.body_heading = None
        self.body_yaw_rate_rps = 0.0
        self._filtered_cross_track = None
        self._reference_timestamp = None
        self._heading_reference = None
        self._accepted_position = None
        self._accepted_speed = 0.
        self.path_geometry = dict(source="GEOMETRIC", rounded=False)

    @property
    def prepared(self):
        return self.path is not None

    def start(self, path_points, position, timestamp, initial_heading, *, initial_speed_mps=None):
        previous_path, previous_progress = self.path, self.progress_m
        self.path_geometry = dict(source="GEOMETRIC", rounded=False)
        if self.curve_speed_planner is None or self.curve_speed_planner.profile is None:
            path_points, self.path_geometry = round_polyline(
                path_points, self.spacing_m, self.desired_turn_radius_m,
                self.path_smoothing_offset_m)
        if self.curve_speed_planner is not None:
            path_points=self.curve_speed_planner.round_path(path_points,self.spacing_m,self.cruise_speed_mps)
        self.path = resample_polyline(path_points, self.spacing_m)
        segments = np.diff(self.path, axis=0)
        lengths = np.linalg.norm(segments, axis=1)
        self.cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
        if self.tangent_tracking is not None:
            self._arc_mid = .5*(self.cumulative[:-1]+self.cumulative[1:])
            self._path_angles = np.unwrap(np.arctan2(-segments[:, 1], segments[:, 0]))
            # A preserved hard vertex has no physical tangent radius.
            jumps = np.abs(np.diff(self._path_angles)) > math.radians(30)
            self._sharp_vertices = self.cumulative[1:-1][jumps]
        if self.curve_speed_planner is not None:
            self.curve_speed_planner.prepare(self.path,self.cumulative,self.cruise_speed_mps)
        self.progress_m = 0.0
        self.segment_index = 0
        self._turn_sign = 0.0
        self.last_result = None
        self.body_heading = _unit(initial_heading)
        self.body_yaw_rate_rps = 0.0
        self._initial_progress(position, self.body_heading)
        if (previous_path is not None and previous_path.shape == self.path.shape
                and np.allclose(previous_path, self.path) and previous_progress > self.progress_m
                and np.linalg.norm(np.asarray(position) - self._point_at(previous_progress))
                    <= np.linalg.norm(np.asarray(position) - self._point_at(self.progress_m)) + self.spacing_m):
            self.progress_m = previous_progress
        self._filtered_cross_track = None
        self._accepted_position = None
        self._accepted_speed = 0.
        self._reference_timestamp = float(timestamp)
        self._heading_reference = math.atan2(-float(self.body_heading[1]), float(self.body_heading[0]))
        return self.update(position,timestamp,body_heading=self.body_heading,speed_mps=initial_speed_mps)

    def accepts_position(self, position, timestamp):
        """Pure check against the last accepted fix, before fitting velocity."""
        if self.tangent_tracking is None or self._accepted_position is None:
            return True
        dt = timestamp - self._reference_timestamp
        # The low-speed trial bound must not impose a new cruise-speed limit.
        limit = max(self.tangent_tracking['fix_speed_mps'], 2*self.cruise_speed_mps,
                    self._accepted_speed + .04)
        return dt > 0 and np.linalg.norm(position-self._accepted_position) <= (
            self.tangent_tracking['fix_slack_m'] + dt*limit)

    def update(
        self,
        position,
        timestamp,
        speed_mps=None,
        velocity=None,
        body_heading=None,
        body_yaw_rate_rps=0.0,
    ):
        """Use a fresh metric position and explicitly supplied IMU heading."""
        if not self.prepared:
            raise RuntimeError("轨迹尚未准备")
        current = np.asarray(position, dtype=np.float64).reshape(2)
        if not np.isfinite(current).all():
            raise ValueError("当前位置无效")
        now = float(timestamp)
        if not math.isfinite(now):
            raise ValueError("采样时间无效")
        if body_heading is None:
            raise RuntimeError("IMU yaw is unavailable; position course is not a heading")
        heading = _unit(body_heading)
        if not self.accepts_position(current, now):
            return dict(self.last_result, position_accepted=False)
        dt = max(0., now - self._reference_timestamp) if self._reference_timestamp is not None else 0.
        measured_speed_mps=max(0.,float(speed_mps)) if speed_mps is not None and math.isfinite(float(speed_mps)) else 0.
        tuning = self.tangent_tracking
        if tuning is None:
            self._advance_progress(current)
        else:
            speed_cap = max(.18, 2*self.cruise_speed_mps, self._accepted_speed)
            step = min(self.search_ahead_m, dt*(min(measured_speed_mps, speed_cap)+.05))
            self._project_connected(current, step)
            self._accepted_position = current.copy()
            self._accepted_speed = measured_speed_mps
        closest = self._point_at(self.progress_m)
        cross_track_m = float(np.linalg.norm(current - closest))
        off_path_ratio = float(np.clip(
            cross_track_m / self.full_attraction_error_m, 0.0, 1.0
        ))

        # 用两段路径弦线而不是单个 3 cm 小段估算前方曲率，避免手绘轨迹
        # 的像素锯齿被放大成连续左右修舵。默认分别观察 4~12 cm 和
        # 20~32 cm 两个区间；临近终点时自动退化为末段切向。
        total_path_m = float(self.cumulative[-1])
        sharp_corner = tuning is not None and np.any(
            (self._sharp_vertices >= self.progress_m-.20) &
            (self._sharp_vertices <= self.progress_m+self.curve_preview_m))
        use_tangent = tuning is not None and not sharp_corner
        if use_tangent:
            curve_measure_span_m = .12
            angles = np.interp(self.progress_m + np.array([0., .01, .13, .25, .37]),
                               self._arc_mid, self._path_angles)
            near_tangent = np.array((math.cos(angles[0]), -math.sin(angles[0])))
            turns = np.diff(angles[1:])
            path_curve_angle_rad = float(turns[0])
            preview_curvature = float(np.max(np.abs(turns))) / curve_measure_span_m
        else:
            near_start_m = min(total_path_m, self.progress_m + 0.04)
            near_end_m = min(total_path_m, self.progress_m + 0.12)
            far_end_m = min(total_path_m, self.progress_m + self.curve_preview_m)
            far_start_m = min(far_end_m, max(self.progress_m + 0.12, far_end_m - 0.12))
            near_tangent = self._chord_direction(near_start_m, near_end_m)
            far_tangent = self._chord_direction(far_start_m, far_end_m)
            path_curve_angle_rad = math.atan2(near_tangent[1]*far_tangent[0]-near_tangent[0]*far_tangent[1],
                                              np.dot(near_tangent, far_tangent))
            curve_measure_span_m = max(self.spacing_m, .5*(far_start_m+far_end_m-near_start_m-near_end_m))
        path_curvature_per_m = path_curve_angle_rad / curve_measure_span_m
        turn_radius_m, turn_radius_source = self._turn_radius_for_curvature(
            path_curvature_per_m
        )
        curve_feedforward = float(np.clip(
            path_curvature_per_m * max(turn_radius_m, self.desired_turn_radius_m),
            -1.0,
            1.0,
        ))
        path_curve_severity = abs(curve_feedforward)

        # 根据前方曲率和离轨距离，在配置的最大／最小前视之间连续调整。
        lookahead_demand = max(path_curve_severity, off_path_ratio)
        active_lookahead_m = (
            self.lookahead_m
            - (self.lookahead_m - self.min_lookahead_m) * lookahead_demand
        )
        lookahead_distance = min(
            total_path_m, self.progress_m + active_lookahead_m
        )
        lookahead = self._point_at(lookahead_distance)
        self.body_heading=heading.copy()
        heading_source="IMU_YAW"
        target=lookahead-current
        body_forward_m=float(np.dot(heading,target))
        body_left_m=float(heading[1]*target[0]-heading[0]*target[1])
        psi_path=math.atan2(-float(near_tangent[1]),float(near_tangent[0]))
        psi_body=math.atan2(-float(heading[1]),float(heading[0]))
        v_along_mps = v_cross_mps = course_error_deg = sideslip_deg = None
        if velocity is not None:
            motion = np.asarray(velocity, dtype=float)
            if motion.shape == (2,) and np.isfinite(motion).all():
                # Same normal as signed_cross_track_error_m; positive means its error grows.
                v_along_mps = float(motion @ near_tangent)
                v_cross_mps = float(near_tangent[0]*motion[1] - near_tangent[1]*motion[0])
                if np.linalg.norm(motion) >= .02:
                    course = math.atan2(-float(motion[1]), float(motion[0]))
                    course_error_deg = math.degrees(wrap_to_pi(course-psi_path))
                    sideslip_deg = math.degrees(wrap_to_pi(course-psi_body))
        offset=current-closest
        signed_cross_track_error_m=float(near_tangent[0]*offset[1]-near_tangent[1]*offset[0])
        if self._filtered_cross_track is None:
            self._filtered_cross_track = signed_cross_track_error_m
        previous_cross = self._filtered_cross_track
        alpha = 1. if self.cross_track_filter_s <= 0 else -math.expm1(-dt/self.cross_track_filter_s)
        self._filtered_cross_track += alpha * (signed_cross_track_error_m - self._filtered_cross_track)
        cross_track_rate_mps = (self._filtered_cross_track - previous_cross) / dt if dt > 1e-6 else 0.
        if use_tangent:
            correction = math.atan2(signed_cross_track_error_m + tuning['lateral_damping_s']*(v_cross_mps or 0.),
                                    tuning['capture_length_m'])
            cross_track_feedback = float(np.clip(correction, -.9, .9))
            curve_heading = float(np.clip(tuning['curvature_lead_m']*path_curvature_per_m, -.48, .48))
            raw_psi_ref = wrap_to_pi(psi_path+curve_heading+cross_track_feedback)
        else:
            raw_psi_ref,curve_heading,cross_track_feedback = self._chord_reference(
                psi_path, path_curve_angle_rad, active_lookahead_m)
        change=wrap_to_pi(raw_psi_ref-self._heading_reference)
        allowed_change=self.heading_reference_rate_rps*dt
        psi_ref=wrap_to_pi(self._heading_reference+float(np.clip(change,-allowed_change,allowed_change)))
        reference_rate_limited=abs(change)>allowed_change+1e-9
        self._heading_reference=psi_ref
        self._reference_timestamp=max(now,self._reference_timestamp)
        heading_error_rad=wrap_to_pi(psi_ref-psi_body)
        heading_feedback=self.k_psi*heading_error_rad
        curve_term=self.k_kappa*measured_speed_mps*path_curvature_per_m
        curve_feedforward=float(np.clip(curve_term/self.r_max_rps,-1.,1.))
        r_cmd=heading_feedback+curve_term
        steering_demand=float(np.clip(r_cmd/self.r_max_rps,-1.,1.))
        # Speed and angle share the same curvature and heading error.
        if use_tangent:
            alignment = max(0., math.cos(raw_psi_ref-psi_body))
            curve_cap = min(self.cruise_speed_mps, tuning['turn_budget_rps']/max(.01, preview_curvature))
            speed_target_mps = curve_cap*(.45+.55*alignment**2)
        else:
            alignment=max(0.,math.cos(heading_error_rad))
            speed_target_mps=min(self.cruise_speed_mps,max(
                self.min_speed_target_mps,
                self.cruise_speed_mps*(1.-(1.-self.curve_speed_ratio)*path_curve_severity)*(.25+.75*alignment*alignment)))
        x_error_m = steering_demand * self.max_x_error_m
        control_severity = abs(steering_demand)
        endpoint_distance = float(np.linalg.norm(current - self.path[-1]))
        remaining = max(0.0, float(self.cumulative[-1]) - self.progress_m)
        drive_distance = max(remaining, endpoint_distance)
        speed_valid = speed_mps is not None and math.isfinite(float(speed_mps))
        arrived = (
            endpoint_distance <= self.arrival_radius_m
            and remaining <= self.arrival_radius_m
        )
        # Decelerate inside the approach radius; do not stop before reaching the arrival circle.
        if not arrived and drive_distance<self.brake_radius_m:
            speed_target_mps=min(speed_target_mps,max(self.min_speed_target_mps,
                self.cruise_speed_mps*drive_distance/self.brake_radius_m))
        curve_speed={'enabled':False,'active':False,'status':'disabled'}
        if self.curve_speed_planner is not None:
            speed_target_mps,curve_speed=self.curve_speed_planner.limit(self.progress_m,speed_target_mps,self.cruise_speed_mps)
        brake_request=arrived
        result = {
            "heading_policy": "tangent" if use_tangent else "corner_chord" if sharp_corner else "legacy",
            "position_accepted": True,
            "position_sample_time": now,
            "curve_speed": curve_speed,
            "path_geometry": self.path_geometry,
            "cross_m": x_error_m,
            "x_error_m": x_error_m,
            "body_left_m": body_left_m,
            "body_forward_m": body_forward_m,
            "heading_error_deg": math.degrees(heading_error_rad),
            "v_along_mps": v_along_mps,
            "v_cross_mps": v_cross_mps,
            "course_error_deg": course_error_deg,
            "sideslip_deg": sideslip_deg,
            "steering_error_deg": math.degrees(heading_error_rad),
            # 固件用这一无量纲量连续混合直线/满转动力；它与本次真正
            # 下发的总转向需求一致，不再单独用曲率把推进削到 20%。
            "curve_severity": control_severity,
            "path_curve_severity": path_curve_severity,
            "curve_speed_radius_scale_m": max(turn_radius_m, self.desired_turn_radius_m),
            "path_curve_angle_deg": math.degrees(path_curve_angle_rad),
            "path_curvature_per_m": path_curvature_per_m,
            "curve_measure_span_m": curve_measure_span_m,
            "curve_feedforward": curve_feedforward,
            "heading_feedback": heading_feedback,
            "cross_track_feedback": cross_track_feedback,
            "filtered_cross_track_error_m": self._filtered_cross_track,
            "curve_heading_feedforward_deg": math.degrees(curve_heading),
            "raw_psi_ref_rad": raw_psi_ref,
            "reference_rate_limited": reference_rate_limited,
            "psi_path_rad": psi_path,
            "psi_body_rad": psi_body,
            "psi_ref_rad": psi_ref,
            "r_cmd_rps": r_cmd,
            "cross_track_rate_mps": cross_track_rate_mps,
            "speed_target_mps": speed_target_mps,
            "steering_demand": steering_demand,
            "estimated_min_turn_radius_m": self.estimated_min_turn_radius_m,
            "turn_radius_m": turn_radius_m,
            "turn_radius_source": turn_radius_source,
            "cross_track_m": cross_track_m,
            "signed_cross_track_error_m": signed_cross_track_error_m,
            "control_severity": control_severity,
            "curve_direction": (
                "LEFT" if path_curve_angle_rad > math.radians(3.0)
                else "RIGHT" if path_curve_angle_rad < math.radians(-3.0)
                else "STRAIGHT"
            ),
            "heading": heading.copy(),
            "heading_source": heading_source,
            "lookahead_m": active_lookahead_m,
            "lookahead_demand": lookahead_demand,
            "off_path_ratio": off_path_ratio,
            "max_x_error_m": self.max_x_error_m,
            "lookahead_point": lookahead.copy(),
            "closest_point": closest.copy(),
            "along_m": self.progress_m,
            "dist_m": remaining,
            "drive_distance_m": drive_distance,
            "end_distance_m": endpoint_distance,
            "speed_mps": measured_speed_mps if speed_valid else None,
            "brake_distance_m": self.brake_radius_m,
            "brake_request": brake_request,
            "seg_index": self.segment_index,
            "arrived": arrived,
            "settled": arrived,
        }
        self.last_result = result
        return result

    def _project_connected(self, position, step):
        """Score only points *inside* the reachable, connected arc interval."""
        lo = max(0., self.progress_m - .5*step)
        hi = min(self.cumulative[-1], self.progress_m + step)
        i = min(len(self.path)-2, max(0, int(np.searchsorted(self.cumulative, lo, 'right'))-1))
        j = min(len(self.path)-1, int(np.searchsorted(self.cumulative, hi, 'right')))
        arc = self.cumulative[i:j]
        lengths = np.diff(self.cumulative[i:j+1])
        units = (self.path[i+1:j+1]-self.path[i:j])/lengths[:, None]
        s = np.clip(arc+np.sum((position-self.path[i:j])*units, axis=1),
                    np.maximum(lo, arc), np.minimum(hi, self.cumulative[i+1:j+1]))
        residual = self.path[i:j] + (s-arc)[:, None]*units - position
        nearest = int(np.argmin(np.sum(residual**2, axis=1)))
        self.progress_m = float(s[nearest])
        self.segment_index = i+nearest

    def _initial_progress(self, position, heading):
        """Acquire the nearby path branch when starting/resuming in mid-route."""
        current = np.asarray(position, dtype=float).reshape(2)
        if not np.isfinite(current).all():
            raise ValueError("当前位置无效")
        segments = np.diff(self.path, axis=0)
        lengths = np.linalg.norm(segments, axis=1)
        ratios = np.clip(np.sum((current-self.path[:-1])*segments, axis=1)/lengths**2, 0., 1.)
        projections = self.path[:-1]+ratios[:, None]*segments
        distances = np.linalg.norm(projections-current, axis=1)
        nearby = np.flatnonzero(distances <= float(np.min(distances))+self.spacing_m)
        alignment = (segments[nearby]/lengths[nearby, None]) @ heading
        scores = distances[nearby]**2+(self.spacing_m*(1.-alignment))**2
        index = int(nearby[int(np.argmin(scores))])
        self.progress_m = float(self.cumulative[index]+ratios[index]*lengths[index])
        self.segment_index = index

    def _advance_progress(self, position):
        total = float(self.cumulative[-1])
        search_end = min(total, self.progress_m + self.search_ahead_m)
        first = max(0, int(np.searchsorted(
            self.cumulative, self.progress_m, side="right"
        )) - 1)
        last = min(
            len(self.path) - 2,
            int(np.searchsorted(self.cumulative, search_end, side="right")),
        )
        best_distance_sq = None
        best_progress = self.progress_m
        for index in range(first, last + 1):
            p0 = self.path[index]
            segment = self.path[index + 1] - p0
            length_sq = float(np.dot(segment, segment))
            if length_sq <= 1e-12:
                continue
            projection = float(np.clip(
                np.dot(position - p0, segment) / length_sq, 0.0, 1.0
            ))
            candidate = p0 + projection * segment
            candidate_progress = float(
                self.cumulative[index] + projection * math.sqrt(length_sq)
            )
            if candidate_progress + 1e-9 < self.progress_m:
                continue
            distance_sq = float(np.dot(position - candidate, position - candidate))
            if best_distance_sq is None or distance_sq < best_distance_sq:
                best_distance_sq = distance_sq
                best_progress = candidate_progress

        self.progress_m = max(self.progress_m, min(best_progress, search_end))
        self.segment_index = max(
            self.segment_index,
            min(
                len(self.path) - 2,
                int(np.searchsorted(
                    self.cumulative, self.progress_m, side="right"
                )) - 1,
            ),
        )

    def _point_at(self, distance_m):
        distance = float(np.clip(distance_m, 0.0, self.cumulative[-1]))
        index = min(
            len(self.path) - 2,
            max(0, int(np.searchsorted(
                self.cumulative, distance, side="right"
            )) - 1),
        )
        segment_length = float(self.cumulative[index + 1] - self.cumulative[index])
        ratio = 0.0 if segment_length <= 1e-9 else (
            distance - float(self.cumulative[index])
        ) / segment_length
        return self.path[index] + ratio * (self.path[index + 1] - self.path[index])

    def _tangent_at(self, distance_m):
        distance = float(np.clip(distance_m, 0.0, self.cumulative[-1]))
        index = min(
            len(self.path) - 2,
            max(0, int(np.searchsorted(
                self.cumulative, distance, side="right"
            )) - 1),
        )
        tangent = self.path[index + 1] - self.path[index]
        return _unit(tangent)

    def _chord_reference(self, psi_path, curve_angle, lookahead):
        correction = math.atan2(self.k_f*self._filtered_cross_track, lookahead)
        lead = float(np.clip(self.curve_heading_gain*curve_angle,
                             -self.max_curve_heading_rad, self.max_curve_heading_rad))
        return wrap_to_pi(psi_path+lead+correction), lead, correction

    def _chord_direction(self, start_m, end_m):
        total = float(self.cumulative[-1])
        start = float(np.clip(start_m, 0.0, total))
        end = float(np.clip(end_m, start, total))
        chord = self._point_at(end) - self._point_at(start)
        if float(np.linalg.norm(chord)) <= 1e-9:
            return self._tangent_at(0.5 * (start + end))
        return _unit(chord)
