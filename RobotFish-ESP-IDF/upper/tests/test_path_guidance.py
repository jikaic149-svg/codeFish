"""Geometry and heading-reference regressions; no device or camera needed."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'vision'))
from path_guidance import PathGuidance, resample_polyline, round_polyline, wrap_to_pi


class GuidanceTests(unittest.TestCase):
    def test_mid_route_start_uses_nearby_branch_and_heading_at_intersection(self):
        guide = PathGuidance()
        result = guide.start([[0, 0], [2, 0], [2, 1], [0, 1]], [.8, 1], 10., [-1, 0])
        self.assertGreater(result['along_m'], 4.)
        self.assertLess(result['cross_track_m'], .001)
        self.assertLess(abs(result['heading_error_deg']), 1.)
        crossing = [[0, 0], [2, 2], [2, 0], [0, 2]]
        result = guide.start(crossing, [1, 1], 11., [-1, 1])
        self.assertGreater(result['along_m'], 4.8)
        self.assertLess(result['cross_track_m'], .03)

    def test_curve_feedforward_reaches_heading_target_and_wrap_limiter_is_time_based(self):
        theta = np.linspace(0, math.pi / 2, 100)
        arc = np.column_stack((np.sin(theta), np.cos(theta)))
        guide = PathGuidance(curve_preview_m=.5, curve_heading_gain=1.1)
        guide.start(arc, arc[0], 10., [1, 0])
        result = guide.update(arc[0], 10.5, speed_mps=.1, body_heading=[1, 0])
        self.assertGreater(result['curve_heading_feedforward_deg'], 10.)
        self.assertGreater(result['psi_ref_rad'], result['psi_path_rad'])
        self.assertLessEqual(abs(result['heading_error_deg']), 17.5 + 1e-8)
        # Across +/-180 degrees the command takes the short two-degree turn.
        guide = PathGuidance()
        heading = math.radians(179)
        target = math.radians(-179)
        direction = [math.cos(target), -math.sin(target)]
        guide.start([[0, 0], np.asarray(direction)*2], [0, 0], 20., [math.cos(heading), -math.sin(heading)])
        result = guide.update([0, 0], 20.1, body_heading=[math.cos(heading), -math.sin(heading)])
        self.assertAlmostEqual(math.degrees(wrap_to_pi(result['psi_ref_rad']-heading)), 2., places=5)
        # A real fast IMU turn remains visible; only the command is rate limited.
        result = guide.update([0, 0], 20.2, body_heading=[0, -1])
        self.assertAlmostEqual(result['psi_body_rad'], math.pi/2)

    def test_cross_track_noise_is_filtered_and_duplicate_frames_do_not_advance_reference(self):
        guide = PathGuidance(cross_track_filter_s=.35)
        guide.start([[0, 0], [2, 0]], [0, 0], 1., [1, 0])
        result = guide.update([.1, .1], 1.1, body_heading=[1, 0])
        self.assertGreater(result['filtered_cross_track_error_m'], 0.)
        self.assertLess(result['filtered_cross_track_error_m'], .04)
        reference = result['psi_ref_rad']
        result = guide.update([.1, -.1], 1.1, body_heading=[1, 0])
        self.assertEqual(result['psi_ref_rad'], reference)

    def test_geometric_rounding_keeps_constrained_u_instead_of_tightening_it(self):
        theta = np.linspace(0, math.pi, 301)
        original = .5*np.column_stack((np.sin(theta), np.cos(theta)))
        path, info = round_polyline(original, .03, .6, .15)
        np.testing.assert_array_equal(path, resample_polyline(original, .03))
        self.assertEqual(info['status'], 'unachievable')
        self.assertFalse(info['rounded'])
        self.assertGreater(info['limited_corners'], 0)
        self.assertEqual(info['achieved_min_radius_m'], info['original_min_radius_m'])
        self.assertGreater(info['achieved_min_radius_m'], .49)
        # A short right angle cannot hold the requested fillet inside the offset budget.
        path, info = round_polyline([[0, 0], [1, 0], [1, 1]], .03, .6, .15)
        np.testing.assert_allclose(path[[0, -1]], [[0, 0], [1, 1]])
        self.assertFalse(info['rounded'])
        self.assertEqual(info['source'], 'GEOMETRIC')
        self.assertGreater(info['limited_corners'], 0)
        # Straight tiny segments remain a straight line, and a spacious shallow corner rounds.
        straight = np.column_stack((np.arange(0., 1.01, .01), np.zeros(101)))
        path, info = round_polyline(straight, .03, .6, .15)
        self.assertFalse(info['rounded'])
        self.assertEqual(info['max_curvature_per_m'], 0.)
        path, info = round_polyline([[0, 0], [1, 0], [2, 1]], .03, .6, .15)
        self.assertTrue(info['rounded'])
        self.assertGreaterEqual(info['achieved_min_radius_m'], .6/1.05)

    def test_trial_curve_speed_does_not_override_forward_output(self):
        theta = np.linspace(0, math.pi, 100)
        arc = .4*np.column_stack((np.sin(theta), np.cos(theta)))
        guide = PathGuidance(cruise_speed_mps=.12, min_speed_target_mps=.04,
                             desired_turn_radius_m=.6, curve_speed_ratio=.05/.12)
        result = guide.start(arc, arc[0], 1., [1, 0])
        self.assertGreaterEqual(result['speed_target_mps'], .04)
        self.assertLessEqual(result['speed_target_mps'], .06)
        self.assertEqual(result['curve_speed_radius_scale_m'], .6)
        self.assertNotIn('power', result)

    def test_velocity_projections_use_motion_instead_of_heading_error(self):
        guide = PathGuidance()
        guide.start([[0, 0], [2, 0]], [0, 0], 1., [0, -1])
        result = guide.update([0, 0], 1.1, body_heading=[0, -1],
                              speed_mps=.1, velocity=[.06, -.08])
        self.assertAlmostEqual(result['v_along_mps'], .06)
        self.assertAlmostEqual(result['v_cross_mps'], -.08)
        self.assertAlmostEqual(result['course_error_deg'], 53.130102354)
        self.assertAlmostEqual(result['sideslip_deg'], -36.869897646)
        result = guide.update([0, 0], 1.2, body_heading=[1, 0], velocity=[0, 0])
        self.assertEqual(result['v_along_mps'], 0.)
        self.assertIsNone(result['course_error_deg'])
        self.assertIsNone(result['sideslip_deg'])
        for velocity in (None, [float('nan'), 0]):
            result = guide.update([0, 0], 1.3, body_heading=[1, 0], velocity=velocity)
            for key in ('v_along_mps', 'v_cross_mps', 'course_error_deg', 'sideslip_deg'):
                self.assertIsNone(result[key])


if __name__ == '__main__':
    unittest.main()
