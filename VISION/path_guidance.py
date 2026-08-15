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


class VelocityEstimator:
    """用短窗口直线拟合估计平移速度，抑制鱼尾逐帧摆动和单帧尖峰。"""

    def __init__(
        self,
        history_seconds=0.40,
        min_span_seconds=0.12,
        min_samples=3,
        blend=0.45,
        max_speed_mps=0.80,
    ):
        self.history_seconds = float(history_seconds)
        self.min_span_seconds = float(min_span_seconds)
        self.min_samples = int(min_samples)
        self.blend = float(np.clip(blend, 0.0, 1.0))
        self.max_speed_mps = float(max_speed_mps)
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
        raw_speed = float(np.linalg.norm(raw_velocity))
        if not np.isfinite(raw_velocity).all():
            return None if self.velocity is None else self.velocity.copy()

        # 超过机器鱼物理可能范围的结果来自色块跳变；不让它污染制动判断。
        if raw_speed > self.max_speed_mps:
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


class HeadingEstimator:
    """以人工初始方向启动，运动稳定后用一段位移平滑更新航向。"""

    def __init__(
        self,
        history_seconds=0.60,
        min_span_seconds=0.25,
        min_displacement_m=0.020,
        min_speed_mps=0.08,
        blend=0.20,
    ):
        self.history_seconds = float(history_seconds)
        self.min_span_seconds = float(min_span_seconds)
        self.min_displacement_m = float(min_displacement_m)
        self.min_speed_mps = float(min_speed_mps)
        self.blend = float(blend)
        self.velocity_estimator = VelocityEstimator(
            history_seconds=history_seconds,
            min_span_seconds=min_span_seconds,
            blend=0.30,
        )
        self.heading = None
        self.source = "INVALID"

    def reset(self, initial_heading, position=None, timestamp=None):
        self.heading = _unit(initial_heading)
        self.source = "INITIAL"
        self._last_sample_time = (
            float(timestamp) if timestamp is not None else None
        )
        self.velocity_estimator.reset(position, timestamp)
        return self.heading.copy()

    def update(self, position, timestamp, allow_course_update=True):
        if self.heading is None:
            raise RuntimeError("必须先设置人工初始航向")
        if not allow_course_update:
            return self.heading.copy(), self.source

        now = float(timestamp)
        current = np.asarray(position, dtype=np.float64).reshape(2)
        if not np.isfinite(current).all() or not math.isfinite(now):
            return self.heading.copy(), self.source
        if self._last_sample_time is not None and now <= self._last_sample_time:
            return self.heading.copy(), self.source
        self._last_sample_time = now
        velocity = self.velocity_estimator.update(current, now)
        if velocity is None:
            return self.heading.copy(), self.source
        speed = float(np.linalg.norm(velocity))
        if speed < self.min_speed_mps:
            return self.heading.copy(), self.source

        course = velocity / speed
        # 精确 180° 更像色块跳变；正常急转会经过中间方向，允许快速跟随。
        if float(np.dot(course, self.heading)) <= -0.95:
            return self.heading.copy(), self.source
        blended = (1.0 - self.blend) * self.heading + self.blend * course
        self.heading = _unit(blended)
        self.source = "COURSE"
        return self.heading.copy(), self.source


class PathGuidance:
    """等距路径、单向进度、前视点和鱼体左右误差的统一 owner。"""

    def __init__(
        self,
        spacing_m=0.03,
        lookahead_m=0.24,
        min_lookahead_m=0.14,
        max_attraction_gain=1.25,
        full_attraction_error_m=0.18,
        curve_preview_m=0.32,
        curve_feedforward_gain=0.35,
        min_curve_steering_gain=0.55,
        max_curve_steering_gain=1.35,
        max_x_error_m=0.24,
        arrival_radius_m=0.20,
        brake_radius_m=0.40,
        search_ahead_m=0.45,
    ):
        self.spacing_m = float(spacing_m)
        self.lookahead_m = float(lookahead_m)
        self.min_lookahead_m = min(
            self.lookahead_m, float(min_lookahead_m)
        )
        self.max_attraction_gain = max(1.0, float(max_attraction_gain))
        self.full_attraction_error_m = max(
            1e-6, float(full_attraction_error_m)
        )
        self.curve_preview_m = max(self.spacing_m, float(curve_preview_m))
        self.curve_feedforward_gain = max(
            0.0, float(curve_feedforward_gain)
        )
        self.min_curve_steering_gain = max(
            0.0, float(min_curve_steering_gain)
        )
        self.max_curve_steering_gain = max(
            self.min_curve_steering_gain, float(max_curve_steering_gain)
        )
        self.max_x_error_m = min(
            self.lookahead_m, max(0.01, float(max_x_error_m))
        )
        self.arrival_radius_m = float(arrival_radius_m)
        self.brake_radius_m = max(
            self.arrival_radius_m, float(brake_radius_m)
        )
        self.search_ahead_m = float(search_ahead_m)
        self.heading_estimator = HeadingEstimator()
        self.clear()

    def clear(self):
        self.path = None
        self.cumulative = None
        self.progress_m = 0.0
        self.segment_index = 0
        self._turn_sign = 0.0
        self.last_result = None

    @property
    def prepared(self):
        return self.path is not None

    def start(self, path_points, position, timestamp, initial_heading):
        self.path = resample_polyline(path_points, self.spacing_m)
        lengths = np.linalg.norm(np.diff(self.path, axis=0), axis=1)
        self.cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
        self.progress_m = 0.0
        self.segment_index = 0
        self._turn_sign = 0.0
        self.last_result = None
        self.heading_estimator.reset(initial_heading, position, timestamp)
        return self.update(position, timestamp, allow_course_update=False)

    def update(
        self, position, timestamp, allow_course_update=True, speed_mps=None
    ):
        if not self.prepared:
            raise RuntimeError("轨迹尚未准备")
        current = np.asarray(position, dtype=np.float64).reshape(2)
        if not np.isfinite(current).all():
            raise ValueError("当前位置无效")

        self._advance_progress(current)
        closest = self._point_at(self.progress_m)
        cross_track_m = float(np.linalg.norm(current - closest))
        off_path_ratio = float(np.clip(
            cross_track_m / self.full_attraction_error_m, 0.0, 1.0
        ))
        active_lookahead_m = (
            self.lookahead_m
            - (self.lookahead_m - self.min_lookahead_m) * off_path_ratio
        )
        attraction_gain = 1.0 + (
            self.max_attraction_gain - 1.0
        ) * off_path_ratio
        lookahead_distance = min(
            float(self.cumulative[-1]), self.progress_m + active_lookahead_m
        )
        lookahead = self._point_at(lookahead_distance)
        # 用两段路径弦线而不是单个 3 cm 小段估算前方曲率，避免手绘轨迹
        # 的像素锯齿被放大成连续左右修舵。默认分别观察 4~12 cm 和
        # 20~32 cm 两个区间；临近终点时自动退化为末段切向。
        near_tangent = self._chord_direction(
            self.progress_m + 0.04,
            self.progress_m + 0.12,
        )
        far_end_m = self.progress_m + self.curve_preview_m
        far_tangent = self._chord_direction(
            max(self.progress_m + 0.12, far_end_m - 0.12),
            far_end_m,
        )
        tangent_dot = float(np.clip(
            np.dot(near_tangent, far_tangent), -1.0, 1.0
        ))
        # y 向下坐标中取“左转为正”的有符号预览角。
        path_curve_angle_rad = float(math.atan2(
            near_tangent[1] * far_tangent[0]
            - near_tangent[0] * far_tangent[1],
            tangent_dot,
        ))
        path_curve_severity = float(np.clip(
            abs(path_curve_angle_rad) / (math.pi / 2.0), 0.0, 1.0
        ))
        heading, heading_source = self.heading_estimator.update(
            current, timestamp, allow_course_update=allow_course_update
        )

        target = lookahead - current
        body_forward_m = float(np.dot(heading, target))
        # y 向下坐标中，鱼体左法向量为 (heading_y, -heading_x)。
        body_left_m = float(heading[1] * target[0] - heading[0] * target[1])
        target_distance = float(np.linalg.norm(target))
        if target_distance <= 1e-9:
            heading_error_rad = 0.0
        else:
            raw_heading_error_rad = float(math.atan2(
                body_left_m, body_forward_m
            ))
            if (
                body_forward_m < 0.0
                and abs(raw_heading_error_rad) >= math.radians(120.0)
            ):
                # 后方目标跨过 atan2 的 +/-180° 边界时，保持首次选定的
                # 掉头方向，避免相邻帧在满左/满右之间翻转。离开后方
                # 120°锥区后恢复普通最短角闭环。
                if self._turn_sign == 0.0:
                    self._turn_sign = (
                        1.0 if raw_heading_error_rad >= 0.0 else -1.0
                    )
                heading_error_rad = (
                    self._turn_sign * abs(raw_heading_error_rad)
                )
            else:
                heading_error_rad = raw_heading_error_rad
                if abs(body_left_m) >= 0.005:
                    self._turn_sign = 1.0 if body_left_m > 0.0 else -1.0

        # 曲率连续决定闭环转向比例：小曲率降低轻微纠偏，抑制画龙；
        # 大曲率提高转向比例，减小转弯半径。max_x_error_m 仅作安全兜底，
        # 不再用固定 ±0.18 m 充当常规控制量。
        normalized_heading_error = float(np.clip(
            heading_error_rad / (math.pi / 2.0),
            -1.0,
            1.0,
        ))
        curve_steering_gain = (
            self.min_curve_steering_gain
            + (
                self.max_curve_steering_gain
                - self.min_curve_steering_gain
            ) * path_curve_severity
        )
        # 即使路径本身近似直线，鱼头已经严重偏转时也不能变成软弱掉头。
        # 仅在曲率增益低于 1 时，按航向误差连续恢复到基础转向比例；
        # 小误差仍保持低增益，大误差则不会因直线路径而失去纠偏能力。
        steering_gain = curve_steering_gain
        if steering_gain < 1.0:
            steering_gain += (
                1.0 - steering_gain
            ) * abs(normalized_heading_error)
        curve_feedforward = float(np.clip(
            path_curve_angle_rad / (math.pi / 2.0), -1.0, 1.0
        )) * self.curve_feedforward_gain * (1.0 - off_path_ratio)
        x_error_m = float(np.clip(
            self.lookahead_m * (
                normalized_heading_error
                * attraction_gain
                * steering_gain
                + curve_feedforward
            ),
            -self.max_x_error_m,
            self.max_x_error_m,
        ))
        control_severity = float(np.clip(
            abs(x_error_m) / max(self.max_x_error_m, 1e-6), 0.0, 1.0
        ))
        slowdown_severity = float(np.clip(max(
            path_curve_severity,
            control_severity,
        ), 0.0, 1.0))
        endpoint_distance = float(np.linalg.norm(current - self.path[-1]))
        remaining = max(0.0, float(self.cumulative[-1]) - self.progress_m)
        drive_distance = max(remaining, endpoint_distance)
        speed_valid = speed_mps is not None and math.isfinite(float(speed_mps))
        measured_speed_mps = max(0.0, float(speed_mps)) if speed_valid else 0.0
        arrived = (
            endpoint_distance <= self.arrival_radius_m
            and remaining <= self.arrival_radius_m
        )
        brake_request = (
            not arrived
            and endpoint_distance <= self.brake_radius_m
            and remaining <= self.brake_radius_m
        )
        result = {
            "cross_m": x_error_m,
            "x_error_m": x_error_m,
            "body_left_m": body_left_m,
            "body_forward_m": body_forward_m,
            "heading_error_deg": math.degrees(heading_error_rad),
            "steering_error_deg": math.degrees(heading_error_rad),
            "curve_severity": slowdown_severity,
            "path_curve_severity": path_curve_severity,
            "path_curve_angle_deg": math.degrees(path_curve_angle_rad),
            "curve_feedforward": curve_feedforward,
            "curve_steering_gain": curve_steering_gain,
            "steering_gain": steering_gain,
            "cross_track_m": cross_track_m,
            "control_severity": control_severity,
            "curve_direction": (
                "LEFT" if path_curve_angle_rad > math.radians(3.0)
                else "RIGHT" if path_curve_angle_rad < math.radians(-3.0)
                else "STRAIGHT"
            ),
            "heading": heading.copy(),
            "heading_source": heading_source,
            "lookahead_m": active_lookahead_m,
            "attraction_gain": attraction_gain,
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

    def _chord_direction(self, start_m, end_m):
        total = float(self.cumulative[-1])
        start = float(np.clip(start_m, 0.0, total))
        end = float(np.clip(end_m, start, total))
        chord = self._point_at(end) - self._point_at(start)
        if float(np.linalg.norm(chord)) <= 1e-9:
            return self._tangent_at(0.5 * (start + end))
        return _unit(chord)
