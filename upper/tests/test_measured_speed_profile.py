"""Recorded dynamics feedforward checks; no devices, network or camera."""
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'vision'))
from control_config import CONTROL
from vision_speed import VisualSpeedLoop


class MeasuredSpeedTests(unittest.TestCase):
    def test_receipt_freshness_keeps_capture_timestamps_for_derivatives(self):
        loop=VisualSpeedLoop(max_age_s=.35)
        self.assertGreater(loop.update(.1,.08,10.,now=10.4,received_at=10.3),0.)
        self.assertEqual(loop.status['observation_age_ms'],400)
        self.assertEqual(loop.status['receipt_age_ms'],100)
        loop.update(.1,.1,10.1,now=10.45,received_at=10.38)
        self.assertEqual(loop.last_time,10.1)
        self.assertAlmostEqual(loop.acceleration,.08)
        self.assertEqual(loop.status['observation_age_ms'],350)
        self.assertEqual(loop.status['receipt_age_ms'],70)

    def test_old_or_invalid_receipt_is_rejected_and_legacy_age_gate_is_unchanged(self):
        loop=VisualSpeedLoop(max_age_s=.35)
        self.assertEqual(loop.update(.1,.08,10.,now=10.4),0.)
        self.assertEqual(loop.status['state'],'stale_feedback')
        for received in (10.04,9.9,10.41,0.,float('nan'),float('inf')):
            with self.subTest(received_at=received):
                self.assertEqual(loop.update(.1,.08,10.,now=10.4,received_at=received),0.)
                self.assertEqual(loop.status['state'],'stale_feedback')
        self.assertGreater(loop.update(.1,.08,10.2,now=10.4),0.)
        self.assertEqual(loop.status['observation_age_ms'],200)
        self.assertIsNone(loop.status['receipt_age_ms'])

    def test_measurements_feed_runtime_and_extrapolation_is_identified(self):
        loop = VisualSpeedLoop(**CONTROL['speed'])
        self.assertEqual(loop.profile_id, 'robotfish-20261025-hybrid-gait-tail-bias')
        first, second, last = loop.feedforward_samples
        for speed,command in loop.feedforward_samples:
            self.assertAlmostEqual(loop.update(speed, speed, 10., now=10.), command)
        middle_speed = (first[0] + second[0]) / 2
        middle_command, coverage = loop._feedforward(middle_speed)
        self.assertAlmostEqual(middle_command, (first[1] + second[1]) / 2)
        self.assertEqual(coverage, 'measured_range')
        low_command, coverage = loop._feedforward(.04)
        self.assertAlmostEqual(low_command, first[1] * .04 / first[0])
        self.assertEqual(coverage, 'below_measured_range')
        self.assertEqual(loop._feedforward(last[0] + .05), (last[1], 'above_measured_range'))
        loop.reset()
        self.assertEqual(loop.update(.10, .40, 11., now=11.), 0.)
        self.assertEqual(loop.update(0., .10, 11.1, now=11.1), 0.)
        self.assertEqual(loop.status['feedforward_percent'], 0.)

    def test_malformed_or_escaping_profile_is_rejected(self):
        with self.assertRaises(ValueError):
            VisualSpeedLoop(profile_file='../vision_speed.py')
        bad = dict(schema=1, profile_id='bad', propulsion=dict(samples=[
            dict(speed_mps=.2, command_pct=20), dict(speed_mps=.1, command_pct=30)]))
        with patch.object(Path, 'read_text', return_value=json.dumps(bad)):
            with self.assertRaises(ValueError):
                VisualSpeedLoop(**CONTROL['speed'])


if __name__ == '__main__':
    unittest.main()
