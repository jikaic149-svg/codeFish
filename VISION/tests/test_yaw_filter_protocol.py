import math
import os
import struct
import sys
import unittest


VISION_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RUNTIME_DIR = os.path.join(VISION_DIR, "runtime")
sys.path.insert(0, VISION_DIR)
sys.path.insert(0, RUNTIME_DIR)

from vision_protocol import VISION_UDP_FRAME, crc16_ccitt, pack_vision_udp_frame
from yaw_filter import BodyYawKalman
from path_guidance import PathGuidance


class YawAndProtocolTests(unittest.TestCase):
    def test_yaw_filter_wraps_and_expires_prediction(self):
        kalman = BodyYawKalman()
        first = kalman.step(0.0, math.radians(179.0))
        second = kalman.step(0.05, math.radians(-179.0))
        self.assertTrue(first["usable"])
        self.assertLess(abs(second["yaw_deg"]), 190.0)
        predicted = kalman.step(0.15, None, measurement_valid=False)
        self.assertEqual(predicted["source"], "PREDICTION")
        expired = kalman.step(0.50, None, measurement_valid=False)
        self.assertFalse(expired["usable"])
        self.assertEqual(expired["source"], "EXPIRED")

    def test_yaw_filter_corrects_keypoint_polarity_flip(self):
        kalman = BodyYawKalman()
        first = kalman.step(0.0, math.radians(20.0))
        flipped = kalman.step(0.05, math.radians(200.0))
        self.assertTrue(first["usable"])
        self.assertFalse(flipped["measurement_rejected"])
        self.assertTrue(flipped["polarity_corrected"])
        self.assertEqual(flipped["source"], "MEASUREMENT")
        self.assertLess(abs(flipped["yaw_deg"] - 20.0), 5.0)

    def test_yaw_filter_freezes_after_expiry_and_reacquires_stably(self):
        kalman = BodyYawKalman()
        kalman.step(0.0, math.radians(20.0))
        kalman.step(0.10, math.radians(45.0))

        # The missed-result window expires.  The old angular-rate prediction
        # must no longer be integrated while the detector is offline.
        expired = kalman.step(0.60, None, measurement_valid=False)
        self.assertEqual(expired["source"], "EXPIRED")
        self.assertFalse(expired["usable"])
        self.assertTrue(expired["reacquire_pending"])

        # One frame alone cannot re-anchor polarity.  The first sample is
        # intentionally the opposite vector (20° + 180°).
        pending = kalman.step(0.65, math.radians(200.0))
        self.assertEqual(pending["source"], "REACQUIRE")
        self.assertFalse(pending["usable"])
        self.assertEqual(pending["reacquire_samples"], 1)

        kalman.step(0.70, math.radians(21.0))
        reacquired = kalman.step(0.75, math.radians(22.0))
        self.assertEqual(reacquired["source"], "REACQUIRED")
        self.assertTrue(reacquired["usable"])
        self.assertAlmostEqual(reacquired["yaw_deg"], 21.0, delta=2.0)
        self.assertFalse(reacquired["polarity_corrected"])

    def test_udp_frame_is_fixed_size_and_crc_protected(self):
        frame = pack_vision_udp_frame(7, 123, 80, -25, True)
        self.assertEqual(len(frame), VISION_UDP_FRAME.size)
        fields = struct.unpack("<BBBBIIBbBBH", frame)
        self.assertEqual(fields[:4], (0xA5, 1, 1, 12))
        self.assertEqual(fields[4:10], (7, 123, 80, -25, 1, 0))
        self.assertEqual(fields[-1], crc16_ccitt(frame[:-2]))

    def test_path_control_uses_filtered_body_yaw(self):
        guidance = PathGuidance(k_psi=2.0, r_max_rps=3.0)
        guidance.start([(0.0, 0.0), (1.0, 0.0)], (0.0, 0.0), 0.0,
                       (1.0, 0.0))
        result = guidance.update(
            (0.0, 0.0), 0.05, allow_course_update=False,
            speed_mps=0.20, body_heading=(0.0, -1.0),
        )
        self.assertAlmostEqual(result["psi_body_rad"], math.pi / 2.0)
        self.assertAlmostEqual(result["psi_path_rad"], 0.0)
        self.assertAlmostEqual(result["psi_ref_rad"], 0.0)
        self.assertAlmostEqual(result["r_cmd_rps"], -math.pi)
        self.assertAlmostEqual(result["steering_demand"], -1.0)

    def test_path_control_matches_single_control_law(self):
        guidance = PathGuidance(
            k_f=1.2, k_psi=2.4, k_kappa=0.8, k_d=0.25,
            r_max_rps=3.0, v_min_mps=0.05,
        )
        guidance.start([(0.0, 0.0), (1.0, 0.0)], (0.0, 0.0), 0.0,
                       (1.0, 0.0))
        result = guidance.update(
            (0.0, 0.10), 0.05, allow_course_update=False,
            speed_mps=0.20, velocity=(0.20, 0.02),
            body_heading=(1.0, 0.0),
        )
        f = result["signed_cross_track_error_m"]
        psi_ref = result["psi_path_rad"] + math.atan2(1.2 * f, 0.20)
        e_psi = (psi_ref - result["psi_body_rad"] + math.pi) % (2.0 * math.pi) - math.pi
        expected_r = 2.4 * e_psi - 0.25 * result["cross_track_rate_mps"]
        expected_demand = max(-1.0, min(1.0, expected_r / 3.0))
        self.assertAlmostEqual(result["psi_ref_rad"], psi_ref)
        self.assertAlmostEqual(result["r_cmd_rps"], expected_r)
        self.assertAlmostEqual(result["steering_demand"], expected_demand)


if __name__ == "__main__":
    unittest.main()
