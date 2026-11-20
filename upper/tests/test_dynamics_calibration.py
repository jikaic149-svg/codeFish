"""Offline dynamics capture checks: no camera, serial, Wi-Fi or motion writes."""
import csv
import json
import math
from pathlib import Path
import shutil
import sys
import unittest
from unittest.mock import Mock, patch
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import dynamics_calibration as dynamics


class DynamicsTests(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parents[2] / 'build' / ('dynamics-test-' + uuid.uuid4().hex)
        self.root.mkdir(parents=True)
        self.recorder = dynamics.DynamicsRecorder(Mock(), Mock(), output_dir=self.root)
        self.clock = 2000000000.
        self.time = patch.object(dynamics.time, 'time', side_effect=lambda: self.clock)
        self.thread = patch.object(dynamics.threading.Thread, 'start')
        self.time.start(); self.thread.start()
        self.settings = dict(mode=3, motion='right', tail_drive_pct=20, tail_bias_pct=10,
            prop_surge_pct=0, prop_diff_pct=20, stationary_confirmed=True, head_confirmed=True)

    def tearDown(self):
        self.recorder.close()
        self.time.stop(); self.thread.stop()
        workspace = Path(__file__).resolve().parents[2]
        if self.root.resolve().is_relative_to(workspace / 'build'):
            shutil.rmtree(self.root)

    def fish(self, uptime=1000, zero=False):
        return {'online': True, 'checked_at': self.clock, 'imu': {'yaw': 4., 'sensor_yaw': 5., 'valid': True},
            'diagnostics': {'uptime_ms': uptime, 'wifi': {'session': 'secret-test'},
                'dynamics_calibration': {'active': not zero, **{key: 0 for key in dynamics.PARAMETERS}},
                'allocation': {'tail_drive_pct': 0 if zero else 20, 'tail_bias_pct': 0,
                    'written_common_pct': 0, 'written_differential_pct': 0 if zero else 20}},
            'actuators': {'left_esc': 0 if zero else 20, 'right_esc': 0 if zero else -20,
                          'tail1': 90., 'tail2': 90.}}

    def vision(self, frame, x, yaw=0.):
        return {'running': True, 'result': {'frame': frame, 'camera_sample_time': self.clock,
            'camera_receipt_time': self.clock, 'position': {'x_m': x, 'y_m': 0., 'valid': True,
                'sample_time': self.clock}, 'optical': {'valid': True, 'world_deg': yaw},
            'initial_heading': {'calibration_id': 'keep-this-id'}, 'aligned_imu': {'sample_time': self.clock-.01}}}

    def test_streaming_preserves_data_release_coasts_and_next_start_is_clean(self):
        state = self.recorder.start(self.settings)
        self.recorder._ingest('fish', self.fish())
        self.recorder._ingest('vision', self.vision(1, 0.))
        first_path = Path(state['path'])
        self.recorder.drive(self.settings)
        self.recorder.note_event('drive_ack', {'session': 'must-redact', 'sequence': 1})
        for index in range(1, 36):
            self.clock += .1
            self.recorder._ingest('vision', self.vision(index+1, index*.01))
        self.recorder.release()
        self.recorder.note_event('stop_ack', {'sequence': 2})
        self.clock += .1
        self.recorder._ingest('fish', self.fish(5000, zero=True))
        for index in range(5):
            self.clock += .1
            self.recorder._ingest('vision', self.vision(index+40, .36+index*.001))
        self.assertTrue(self.recorder.state()['active'])
        self.assertEqual(self.recorder.state()['phase'], 'coast')
        state = self.recorder.finish()
        self.assertTrue(state['saved'], state['error'])
        self.assertAlmostEqual(state['summary']['speed_mps'], .1, places=5)
        self.assertIsNotNone(state['summary']['output_zero_time_s'])
        self.assertGreater(state['summary']['coast_distance_m'], 0)
        self.assertEqual(state['summary']['fit_quality'], 'insufficient')
        self.assertEqual(json.loads((first_path/'metadata.json').read_text(encoding='utf-8'))['scene']['height_m'], 1.6)
        fish = (first_path/'fish.jsonl').read_text(encoding='utf-8')
        self.assertNotIn('secret-test', fish)
        self.assertIn('[redacted]', fish)
        events = (first_path/'events.jsonl').read_text(encoding='utf-8')
        self.assertNotIn('must-redact', events)
        self.assertIn('motion_start_estimated', events)
        with (first_path/'samples.csv').open(encoding='utf-8', newline='') as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(rows[0]['raw_yaw_deg'], '5.0')
        self.assertEqual(rows[0]['filtered_yaw_deg'], '4.0')
        self.assertEqual(rows[0]['output_prop_diff_pct'], '20')
        self.assertEqual(rows[-1]['phase'], 'coast')
        new = self.recorder.start(self.settings)
        self.assertNotEqual(new['segment_id'], state['segment_id'])
        self.assertEqual(new['samples'], {'vision': 0, 'fish': 0})
        self.assertEqual(new['recent_samples'], [])
        self.assertTrue(first_path.exists())

    def test_no_velocity_across_source_gap_and_source_dedup(self):
        self.recorder.start(self.settings)
        for index in range(12):
            self.clock += .1
            sample = self.vision(index, index*.01)
            self.recorder._ingest('vision', sample)
            self.recorder._ingest('vision', sample)
        self.assertEqual(self.recorder.samples['vision'], 12)
        self.assertAlmostEqual(self.recorder.recent[-1]['speed_1s_mps'], .1, places=5)
        self.clock += 1.
        self.recorder._ingest('vision', self.vision(20, 5.))
        self.assertIsNone(self.recorder.recent[-1].get('speed_1s_mps'))

    def test_save_failure_is_retryable_and_blocks_silent_new_segment(self):
        self.recorder.start(self.settings)
        with patch.object(dynamics, '_atomic_json', side_effect=OSError('disk-full')):
            state = self.recorder.finish()
        self.assertFalse(state['saved'])
        self.assertIn('disk-full', state['error'])
        with self.assertRaises(ValueError): self.recorder.start(self.settings)
        state = self.recorder.finish()
        self.assertTrue(state['saved'], state['error'])
        self.assertEqual(state['error'], '')
        with (self.root/'calibration-table.csv').open(encoding='utf-8-sig') as stream:
            self.assertEqual(len(list(csv.DictReader(stream))), 1)

    def test_circle_fit_accepts_quarter_arc_and_rejects_short_arc(self):
        points = [(1+.4*math.cos(i*math.pi/48), .5+.4*math.sin(i*math.pi/48)) for i in range(25)]
        result = dynamics.fit_circle(points)
        self.assertAlmostEqual(result['turn_radius_m'], .4, places=6)
        self.assertEqual(result['fit_quality'], 'good')
        short = [(1+.4*math.cos(i*.01), .5+.4*math.sin(i*.01)) for i in range(25)]
        result = dynamics.fit_circle(short)
        self.assertIsNone(result['turn_radius_m'])

    def test_mixed_parameter_segment_never_presents_single_recipe_results(self):
        self.recorder.start(self.settings)
        self.recorder.drive(self.settings)
        for index in range(35):
            self.clock += .1
            self.recorder._ingest('vision', self.vision(index, index*.01))
        self.recorder.release()
        changed = dict(self.settings, tail_drive_pct=40, prop_diff_pct=30)
        self.recorder.drive(changed)
        for index in range(35, 70):
            self.clock += .1
            self.recorder._ingest('vision', self.vision(index, index*.01))
        self.recorder.release()
        state = self.recorder.finish()
        self.assertTrue(state['saved'], state['error'])
        summary = state['summary']
        self.assertTrue(summary['mixed_settings'])
        self.assertEqual(len(summary['parameter_sets']), 2)
        self.assertEqual(summary['drive_count'], 2)
        for key in ('speed_mps', 'yaw_rate_dps', 'steady_speed_mps', 'steady_yaw_rate_dps', 'turn_radius_m'):
            self.assertIsNone(summary[key], key)
        self.assertTrue(summary['warnings'])
        with (self.root/'calibration-table.csv').open(encoding='utf-8-sig') as source:
            row = list(csv.DictReader(source))[0]
        self.assertEqual(row['mode'], 'mixed')
        self.assertEqual(row['tail_drive_pct'], '')
        with (self.recorder.path/'samples.csv').open(encoding='utf-8') as source:
            self.assertEqual({row['tail_drive_pct'] for row in csv.DictReader(source)}, {'20', '40'})


if __name__ == '__main__': unittest.main()
