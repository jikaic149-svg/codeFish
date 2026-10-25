"""Synthetic calibration only. No camera, network or actuator calls."""
import copy
import csv
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'vision'), str(ROOT / 'upper')]
from control_config import CONTROL
from curve_speed import CurveSpeedPlanner, DisplacementSpeedCheck
from path_guidance import PathGuidance, resample_polyline


SETTINGS = dict(enabled=True, profile_file='fixture.json', gentle_speed_mps=.07,
                tight_speed_mps=.045, simplify_tolerance_m=.02, max_rounding_offset_m=.15)
PROFILE = dict(schema=1, drive_mode='HYBRID', profile_id='SYNTHETIC_TEST_ONLY',
               coast_method='zero_forward_keep_steering', source_segments=['synthetic-fixture'], samples=[
                   dict(speed_mps=.04, coast_distance_m=.08, left_radius_m=.12, right_radius_m=.16),
                   dict(speed_mps=.07, coast_distance_m=.25, left_radius_m=.25, right_radius_m=.30),
                   dict(speed_mps=.10, coast_distance_m=.60, left_radius_m=.40, right_radius_m=.45)])


class CurveSpeedTests(unittest.TestCase):
    def planner(self, folder, profile=None, **settings):
        (Path(folder) / 'fixture.json').write_text(json.dumps(PROFILE if profile is None else profile), encoding='utf-8')
        return CurveSpeedPlanner(dict(SETTINGS, **settings), folder)

    def test_curve_calibration_is_disabled_and_trace_measurements_are_enabled(self):
        self.assertFalse(CONTROL['curve_speed']['enabled'])
        self.assertTrue(CONTROL['trace_validation']['enabled'])
        self.assertIsNone(CONTROL['curve_speed']['profile_file'])

    def test_disabled_planner_preserves_path_and_control_results(self):
        with tempfile.TemporaryDirectory() as folder:
            planner = self.planner(folder, enabled=False)
            legacy = PathGuidance(cruise_speed_mps=.10, min_speed_target_mps=.04)
            prepared = PathGuidance(cruise_speed_mps=.10, min_speed_target_mps=.04, curve_speed_planner=planner)
            path = [[0, 0], [1, 0], [1, -1]]
            for guide in (legacy, prepared):guide.start(path, [0, 0], 1., [1, 0])
            np.testing.assert_array_equal(legacy.path, prepared.path)
            for position in ([.2, .03], [.7, 0], [1, -.6]):
                a = legacy.update(position, 2., speed_mps=.1, body_heading=[1, 0])
                b = prepared.update(position, 2., speed_mps=.1, body_heading=[1, 0])
                for key in a:
                    if key == 'curve_speed':continue
                    if isinstance(a[key], np.ndarray):np.testing.assert_array_equal(a[key], b[key])
                    else:self.assertEqual(a[key], b[key], key)
            self.assertIsNone(planner.budget)

    def test_missing_invalid_or_wrong_mode_data_never_activates(self):
        with tempfile.TemporaryDirectory() as folder:
            missing = CurveSpeedPlanner(SETTINGS, folder)
            self.assertEqual(missing.status, 'missing_calibration')
            for key, value in [('drive_mode', 'PROP'), ('source_segments', []), ('coast_method', 'emergency_stop')]:
                bad = copy.deepcopy(PROFILE);bad[key] = value
                planner = self.planner(folder, bad)
                self.assertEqual(planner.status, 'invalid_calibration')
                self.assertIsNone(planner.profile)
            bad = copy.deepcopy(PROFILE);bad['samples'][1]['coast_distance_m'] = .01
            self.assertEqual(self.planner(folder, bad).status, 'invalid_calibration')

    def test_rounding_preserves_endpoints_and_precedes_speed_reduction(self):
        with tempfile.TemporaryDirectory() as folder:
            planner = self.planner(folder)
            guide = PathGuidance(cruise_speed_mps=.1, min_speed_target_mps=.04, curve_speed_planner=planner)
            guide.start([[0, 1], [1, 1], [1, 0]], [0, 1], 1., [1, 0])
            np.testing.assert_allclose(guide.path[[0, -1]], [[0, 1], [1, 0]], atol=1e-10)
            self.assertTrue(planner.rounded)
            self.assertTrue(np.all(guide.path >= -1e-8) and np.all(guide.path <= 1 + 1e-8))
            cap, detail = planner.limit(.5, .1, .1)
            self.assertLess(cap, .1)
            self.assertGreater(detail['distance_to_curve_m'], 0)
            self.assertTrue(detail['active'])
            self.assertTrue(np.all(planner.budget[:-1] <= np.diff(guide.cumulative) + planner.budget[1:] + 1e-9))
            self.assertEqual(planner.limit(guide.cumulative[-1], .04, .1)[0], 0.)

    def test_left_and_right_measurements_generate_different_rounding(self):
        with tempfile.TemporaryDirectory() as folder:
            left, right = self.planner(folder), self.planner(folder)
            a = left.round_path([[0, 0], [1, 0], [1, -1]], .03, .1)
            b = right.round_path([[0, 0], [1, 0], [1, 1]], .03, .1)
            self.assertNotAlmostEqual(float(a[1, 0]), float(b[1, 0]))
            with self.assertRaisesRegex(ValueError, '半径'):
                left.round_path([[0, 0], [.05, 0], [.05, -.05]], .03, .1)
            with self.assertRaisesRegex(ValueError, '折返'):
                left.round_path([[0, 0], [1, 0], [.1, 0]], .03, .1)

    def test_speed_above_calibration_range_does_not_activate(self):
        with tempfile.TemporaryDirectory() as folder:
            planner = self.planner(folder)
            guide = PathGuidance(cruise_speed_mps=.2, curve_speed_planner=planner)
            guide.start([[0, 0], [2, 0]], [0, 0], 1., [1, 0])
            self.assertEqual(planner.status, 'insufficient_speed_range')
            self.assertIsNone(planner.budget)

    def test_one_second_check_rejects_gaps_identity_changes_and_repeated_frames(self):
        check = DisplacementSpeedCheck()
        for i in range(11):speed = check.update([i * .01, 0], 10 + i * .1, ('pool', 1))
        self.assertAlmostEqual(speed, .1)
        self.assertIsNone(check.update([.1, 0], 11., ('pool', 1)))
        self.assertIsNone(check.update([.2, 0], 12., ('pool', 1)))
        self.assertIsNone(check.update([.3, 0], 12.1, ('pool', 2)))
        self.assertIsNone(check.update(None, 12.2, ('pool', 2)))
        self.assertFalse(check.samples)

    def test_output_logs_use_past_native_snapshot_and_reach_csv(self):
        from web_runtime import WebBridge
        from observer_runtime import ObserverSession
        from pool_perspective import PoolPerspective
        with tempfile.TemporaryDirectory() as folder, patch.dict(os.environ, {
                'ROBOFISH_PARENT_TOKEN': 'fixture', 'ROBOFISH_VISION_TOKEN': 'fixture'}), \
                patch.dict(CONTROL, {'trace_validation': {'enabled': True}}):
            bridge = WebBridge(observer=True)
            point = [30., 30.]
            detector = SimpleNamespace(device='cpu', last_speed={}, update=lambda frame, stamp:
                SimpleNamespace(position=tuple(point), confidence=.9, metrics={'track_id': 1}))
            session = ObserverSession('0', 'fixture.pt', bridge, detector=detector, output=folder,
                                      calibration=Path(folder) / 'not-saved.json')
            image = np.zeros((65, 101, 3), np.uint8)
            session.field = PoolPerspective(1., .64).calibrate([[0, 0], [100, 0], [100, 64], [0, 64]], image.shape)
            now = time.time()
            for i in range(11):
                point[0] = 30 + i
                session.accept_frame(i + 1, now - 1 + i * .1, image)
            stamp = session.sample_time
            for dt, common in [(-.1, 12.), (.1, 99.)]:
                bridge.yaw_history.append({'sample_time': stamp + dt, 'telemetry_time': stamp + dt,
                    'source_id': 100 + int((dt + .1) * 1000), 'yaw_cw_deg': 0.,
                    'allocation': {'written_common_pct': common, 'written_differential_pct': 20.,
                                   'tail_drive_pct': 0., 'tail_frequency_hz': 0.}})
            session.samples.start({});session.publish(write_sample=True);session.samples.close()
            self.assertEqual(bridge.result['validation']['common_pct'], 12.)
            self.assertAlmostEqual(bridge.result['validation']['speed_1s_mps'], .1, places=5)
            with Path(session.samples.status['file']).open(encoding='utf-8-sig', newline='') as source:
                row = next(csv.DictReader(source))
            self.assertEqual(float(row['output_common_pct']), 12.)
            self.assertEqual(float(row['output_tail_drive_pct']), 0.)
            self.assertAlmostEqual(float(row['speed_1s_mps']), .1, places=5)
            bridge.close()


if __name__ == '__main__':unittest.main()
