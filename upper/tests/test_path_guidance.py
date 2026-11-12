"""Geometry and heading-reference regressions; no device or camera needed."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'vision'))
from path_guidance import PathGuidance, resample_polyline, round_polyline, wrap_to_pi
from control_config import CONTROL


class GuidanceTests(unittest.TestCase):
    def test_return_updates_projection_without_chasing_old_point_backwards(self):
        guide = PathGuidance(lookahead_m=.5,min_lookahead_m=.35,
                             cruise_speed_mps=.12,min_speed_target_mps=.04)
        guide.start([[0,0],[1,0],[1,-1]],[.5,0],1.,[1,0])
        for stamp,position in [(1.1,[.6,.3]),(1.2,[.7,.25]),(1.3,[.8,.16])]:
            result = guide.update(position,stamp,body_heading=[1,0],speed_mps=.12)
            self.assertEqual(result['phase'],'RETURN_TO_PATH')
            self.assertAlmostEqual(result['along_m'],position[0])
            self.assertFalse(result['progress_held'])
            self.assertTrue(result['lookahead_held'])
            self.assertEqual(result['lookahead_m'],0.)
            np.testing.assert_allclose(result['lookahead_point'],[position[0],0],atol=1e-8)
            np.testing.assert_allclose(result['rejoin_point'],result['projection_point'])
            self.assertEqual(result['curve_heading_feedforward_deg'],0.)
            self.assertEqual(result['curve_feedforward'],0.)
            self.assertLess(result['raw_psi_ref_rad'],math.pi/2)
            self.assertGreater(result['raw_psi_ref_rad'],0.)
            self.assertEqual(result['speed_target_mps'],.04)
        # Return distance is measured from the CURRENT projection, not the old anchor.
        result=guide.update([.95,.04],1.4,body_heading=[0,-1])
        self.assertEqual(result['phase'],'ALIGN_WITH_PATH')
        self.assertLess(result['rejoin_distance_m'],.05)
        self.assertAlmostEqual(result['along_m'],.95)

    def test_alignment_precedes_target_advance_and_lookahead_restores_gradually(self):
        guide = PathGuidance(lookahead_m=.5,min_lookahead_m=.35,
                             cruise_speed_mps=.12,min_speed_target_mps=.04)
        guide.start([[0,0],[2,0]],[.5,.3],1.,[1,0])
        held = guide.progress_m
        result = guide.update([.5,.05],1.1,body_heading=[0,-1])
        self.assertEqual(result['phase'],'ALIGN_WITH_PATH')
        self.assertEqual(result['along_m'],held)
        result = guide.update([.53,.04],1.2,body_heading=[0,-1])
        self.assertAlmostEqual(result['along_m'],.53)
        result = guide.update([.53,.04],1.3,body_heading=[1,0])
        self.assertEqual(result['phase'],'TRACK')
        self.assertGreater(result['along_m'],held)
        self.assertGreater(result['lookahead_m'],0.)
        self.assertLessEqual(result['lookahead_m'],.12*.1+1e-9)
        result = guide.update([.54,.04],1.4,body_heading=[1,0])
        self.assertLessEqual(result['lookahead_m'],.12*.2+1e-9)

    def test_rejoin_hysteresis_and_heading_only_capture_do_not_chatter(self):
        guide = PathGuidance()
        guide.start([[0,0],[2,0]],[.5,0],1.,[0,-1])
        for i,y in enumerate((.1,.13,.09,.11),1):
            result = guide.update([.5,y],1+i*.1,body_heading=[0,-1])
            self.assertEqual(result['phase'],'ALIGN_WITH_PATH')
            self.assertAlmostEqual(result['along_m'],.5)
        result = guide.update([.5,.16],1.5,body_heading=[0,-1])
        self.assertEqual(result['phase'],'RETURN_TO_PATH')
        result = guide.update([.5,.10],1.6,body_heading=[1,0])
        self.assertEqual(result['phase'],'RETURN_TO_PATH')
        self.assertEqual(result['lookahead_m'],0.)

    def test_rejoin_closed_loop_with_synthetic_yaw_response(self):
        # A bounded toy response tests sequencing, not real fish hydrodynamics.
        for initial,yaw in (([.5,.4],0.),([.5,-.4],0.),([.5,0.],math.pi/2),([.5,.4],math.pi)):
            with self.subTest(initial=initial,yaw=yaw):
                guide = PathGuidance(lookahead_m=.5,min_lookahead_m=.35,
                                     cruise_speed_mps=.12,min_speed_target_mps=.04)
                position=np.array(initial,dtype=float);angle=yaw;dt=.05
                guide.start([[0,0],[3,0]],position,0.,[math.cos(angle),-math.sin(angle)])
                seen_track=False;previous=guide.progress_m
                for step in range(1,1601):
                    result=guide.update(position,step*dt,body_heading=[math.cos(angle),-math.sin(angle)])
                    self.assertGreaterEqual(result['along_m']+1e-9,previous)
                    previous=result['along_m']
                    if not seen_track:
                        if result['phase']=='TRACK':
                            self.assertLessEqual(abs(position[1]),guide.rejoin_exit_m)
                            self.assertLessEqual(abs(wrap_to_pi(angle)),guide.rejoin_heading_rad)
                            self.assertLessEqual(result['lookahead_m'],guide.cruise_speed_mps*dt+1e-8)
                            seen_track=True
                        else:
                            self.assertLess(abs(wrap_to_pi(result['raw_psi_ref_rad']-result['psi_path_rad'])),math.pi/2)
                            self.assertEqual(result['lookahead_m'],0.)
                    turn=np.clip(2*wrap_to_pi(result['psi_ref_rad']-angle),
                                 -math.radians(60),math.radians(60))
                    angle=wrap_to_pi(angle+float(turn)*dt)
                    position+=result['speed_target_mps']*dt*np.array([math.cos(angle),-math.sin(angle)])
                    if result['arrived']:break
                self.assertTrue(seen_track)
                self.assertTrue(result['arrived'],(position,result['phase']))
                self.assertLess(abs(position[1]),.02)

    def test_deployed_parameters_shorten_lookahead_and_strengthen_ten_cm_correction(self):
        settings=CONTROL['guidance']
        results=[]
        for error in (0.,.05,.10):
            guide=PathGuidance(lookahead_m=settings['lookahead_m'],
                min_lookahead_m=settings['min_lookahead_m'],k_f=settings['cross_track_gain'],
                full_attraction_error_m=settings['full_attraction_error_m'],
                rejoin_enter_m=settings['rejoin_enter_m'],rejoin_exit_m=settings['rejoin_exit_m'])
            result=guide.start([[0,0],[3,0]],[.5,error],0.,[1,0])
            results.append(result)
        self.assertGreater(results[0]['lookahead_m'],results[1]['lookahead_m'])
        self.assertGreater(results[1]['lookahead_m'],results[2]['lookahead_m'])
        self.assertAlmostEqual(results[2]['lookahead_m'],.20)
        self.assertAlmostEqual(math.degrees(results[2]['cross_track_feedback']),30.9637565,places=5)
        # The increased geometry gain must not bypass the heading slew limit.
        result=guide.update([.5,.10],.1,body_heading=[1,0])
        self.assertAlmostEqual(math.degrees(result['psi_ref_rad']),3.5)

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
