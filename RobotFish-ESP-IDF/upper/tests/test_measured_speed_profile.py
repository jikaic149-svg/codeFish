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
    def test_measurements_feed_runtime_and_extrapolation_is_identified(self):
        loop = VisualSpeedLoop(**CONTROL['speed'])
        self.assertEqual(loop.profile_id, 'robotfish-20261025-prop-surge-tail-angle')
        first, last = loop.feedforward_samples
        self.assertAlmostEqual(loop.update(first[0], first[0], 10., now=10.), first[1])
        middle_speed = (first[0] + last[0]) / 2
        middle_command, coverage = loop._feedforward(middle_speed)
        self.assertAlmostEqual(middle_command, (first[1] + last[1]) / 2)
        self.assertEqual(coverage, 'measured_range')
        low_command, coverage = loop._feedforward(.10)
        self.assertAlmostEqual(low_command, first[1] * .10 / first[0])
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
