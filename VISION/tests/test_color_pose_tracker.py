import os
import sys
import unittest

import numpy as np


VISION_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if VISION_DIR not in sys.path:
    sys.path.insert(0, VISION_DIR)

try:
    import cv2
    from color_pose_tracker import ColorPointTracker, WhitePointTracker
except ImportError:  # pragma: no cover - depends on the runtime vision env
    cv2 = None
    ColorPointTracker = None
    WhitePointTracker = None


@unittest.skipUnless(cv2 is not None, "OpenCV is required for colour tracking tests")
class ColorPointTrackerTests(unittest.TestCase):
    @staticmethod
    def _frame(point):
        frame = np.full((120, 160, 3), 25, dtype=np.uint8)
        cv2.circle(frame, tuple(int(value) for value in point), 7, (255, 0, 255), -1)
        return frame

    def test_calibrate_track_predict_and_expire(self):
        tracker = ColorPointTracker(
            profile_path=None,
            search_radius_px=24.0,
            prediction_horizon_s=0.30,
        )
        tracker.calibrate_from_point(
            self._frame((50, 60)), (50, 60), sample_radius_px=10, save=False
        )

        detected = tracker.update(self._frame((54, 60)), 1.0, hint=(54, 60))
        self.assertTrue(detected["valid"])
        self.assertEqual(detected["source"], "COLOR")
        self.assertAlmostEqual(detected["position"][0], 54.0, delta=2.0)

        missing = np.full((120, 160, 3), 25, dtype=np.uint8)
        predicted = tracker.update(missing, 1.1)
        self.assertTrue(predicted["valid"])
        self.assertEqual(predicted["source"], "PREDICTION")

        expired = tracker.update(missing, 1.5)
        self.assertFalse(expired["valid"])
        self.assertEqual(expired["source"], "UNAVAILABLE")

    def test_calibration_weights_led_centre_over_patch_background(self):
        # A small LED surrounded by a differently coloured border is the
        # failure mode seen when the click patch is too large.  The profile
        # should follow the clicked magenta centre, not the border hue.
        frame = np.full((80, 80, 3), 25, dtype=np.uint8)
        cv2.circle(frame, (40, 40), 3, (255, 0, 255), -1)
        cv2.rectangle(frame, (35, 35), (45, 45), (255, 100, 0), 2)
        tracker = ColorPointTracker()
        profile = tracker.calibrate_from_point(frame, (40, 40), save=False)
        self.assertAlmostEqual(profile.hue_center, 150.0, delta=4.0)
        self.assertLessEqual(profile.hue_tolerance, 16.0)


@unittest.skipUnless(cv2 is not None and WhitePointTracker is not None,
                     "OpenCV is required for white tracking tests")
class WhitePointTrackerTests(unittest.TestCase):
    @staticmethod
    def _frame(point):
        frame = np.full((240, 360, 3), 25, dtype=np.uint8)
        cv2.circle(frame, tuple(int(value) for value in point), 8,
                   (255, 255, 255), -1)
        return frame

    def test_white_head_is_selected_inside_yolo_fish_box(self):
        tracker = WhitePointTracker(search_radius_px=40.0)
        # The semantic white region is searched inside the fish bbox.  The
        # orange tail remains a separate endpoint within the same fish box.
        result = tracker.update(
            self._frame((125, 120)), 1.0,
            hint=None, bbox=(90, 105, 150, 135), tail=(100, 120),
        )
        self.assertTrue(result["valid"])
        self.assertEqual(result["source"], "WHITE")
        self.assertAlmostEqual(result["position"][0], 125.0, delta=3.0)
        self.assertAlmostEqual(result["position"][1], 120.0, delta=3.0)

    def test_outside_box_is_not_reacquired_without_tail_anchor(self):
        tracker = WhitePointTracker(search_radius_px=40.0)
        frame = self._frame((125, 120))
        first = tracker.update(
            frame, 1.0,
            bbox=None,
            tail=(100, 120),
        )
        self.assertEqual(first["source"], "WHITE")
        second = tracker.update(
            frame, 1.03,
            bbox=(90, 105, 110, 135),
            tail=None,
        )
        self.assertEqual(second["source"], "PREDICTION")

    def test_border_glare_is_not_used_as_head(self):
        tracker = WhitePointTracker(search_radius_px=40.0)
        frame = self._frame((125, 120))
        # Simulate the large clipped reflection seen in the replay logs.
        frame[:62, 250:] = 255
        result = tracker.update(
            frame, 1.0, bbox=(90, 105, 150, 135), tail=(100, 120)
        )
        self.assertTrue(result["valid"])
        self.assertAlmostEqual(result["position"][0], 125.0, delta=3.0)

    def test_dynamic_value_floor_tracks_bright_background(self):
        tracker = WhitePointTracker(search_radius_px=40.0)
        frame = np.full((240, 360, 3), 180, dtype=np.uint8)
        cv2.circle(frame, (125, 120), 8, (240, 240, 240), -1)
        result = tracker.update(
            frame, 1.0, bbox=(90, 105, 150, 135), tail=(100, 120)
        )
        self.assertTrue(result["valid"])
        self.assertGreaterEqual(result["dynamic_value_min"], 135)
        self.assertAlmostEqual(result["position"][0], 125.0, delta=3.0)

    def test_distant_white_strip_outside_yolo_box_is_rejected(self):
        tracker = WhitePointTracker(search_radius_px=84.0)
        frame = np.full((480, 640, 3), 25, dtype=np.uint8)
        # This reproduces the replay geometry: the fish/tail are near the
        # upper-left while a narrow bright strip is hundreds of pixels away.
        cv2.rectangle(frame, (514, 391), (552, 400), (255, 255, 255), -1)
        result = tracker.update(
            frame, 1.0,
            bbox=(384, 107, 439, 177),
            tail=(416, 146),
        )
        self.assertFalse(result["valid"])
        self.assertEqual(result["source"], "UNAVAILABLE")
        self.assertIsNone(result["bbox"])

    def test_white_blob_attached_to_tail_is_rejected(self):
        tracker = WhitePointTracker(search_radius_px=40.0)
        frame = np.full((240, 360, 3), 25, dtype=np.uint8)
        cv2.circle(frame, (106, 120), 7, (255, 255, 255), -1)
        result = tracker.update(
            frame, 1.0,
            bbox=(90, 105, 150, 135),
            tail=(100, 120),
            tail_bbox=(95, 115, 105, 125),
        )
        self.assertFalse(result["valid"])
        self.assertEqual(result["source"], "UNAVAILABLE")

    def test_yolo_head_is_validated_against_selected_white_region(self):
        frame = self._frame((125, 120))
        kwargs = dict(
            bbox=(90, 105, 150, 135),
            tail=(100, 120),
            tail_bbox=(95, 115, 105, 125),
        )
        inside = WhitePointTracker(search_radius_px=40.0).update(
            frame, 1.0, head_hint=(125, 120), **kwargs
        )
        self.assertTrue(inside["head_hint_inside"])

        # A pose head that lands on the tail is not accepted as a white-head
        # point; the selected white centroid remains the semantic endpoint.
        on_tail = WhitePointTracker(search_radius_px=40.0).update(
            frame, 1.0, head_hint=(100, 120), **kwargs
        )
        self.assertTrue(on_tail["valid"])
        self.assertFalse(on_tail["head_hint_inside"])
        self.assertAlmostEqual(on_tail["position"][0], 125.0, delta=3.0)

    def test_largest_white_component_inside_yolo_box_wins(self):
        tracker = WhitePointTracker(search_radius_px=40.0)
        frame = np.full((240, 360, 3), 25, dtype=np.uint8)
        cv2.circle(frame, (125, 120), 9, (255, 255, 255), -1)
        cv2.circle(frame, (145, 120), 4, (255, 255, 255), -1)
        result = tracker.update(
            frame, 1.0,
            bbox=(90, 105, 160, 135),
            tail=(100, 120),
            tail_bbox=(95, 115, 105, 125),
        )
        self.assertTrue(result["valid"])
        self.assertAlmostEqual(result["position"][0], 125.0, delta=2.0)
        self.assertGreater(result["area"], 150.0)

    def test_white_fin_near_tail_is_rejected_when_larger_than_head(self):
        tracker = WhitePointTracker(search_radius_px=60.0)
        frame = np.full((240, 360, 3), 25, dtype=np.uint8)
        # The fin is deliberately larger than the real head marker, but sits
        # close to the orange tail.  Area-only selection would choose it.
        cv2.rectangle(frame, (108, 130), (132, 150), (255, 255, 255), -1)
        cv2.circle(frame, (165, 92), 7, (255, 255, 255), -1)
        result = tracker.update(
            frame, 1.0,
            bbox=(90, 70, 190, 170),
            tail=(100, 120),
            tail_bbox=(94, 114, 106, 126),
        )
        self.assertTrue(result["valid"])
        self.assertAlmostEqual(result["position"][0], 165.0, delta=3.0)
        self.assertLess(result["position"][1], 110.0)

    def test_yolo_head_hint_can_select_smaller_head_over_fin(self):
        tracker = WhitePointTracker(search_radius_px=60.0)
        frame = np.full((240, 360, 3), 25, dtype=np.uint8)
        cv2.rectangle(frame, (145, 125), (165, 150), (255, 255, 255), -1)
        cv2.circle(frame, (175, 82), 5, (255, 255, 255), -1)
        result = tracker.update(
            frame, 1.0,
            bbox=(90, 60, 210, 175),
            tail=(100, 120),
            tail_bbox=(94, 114, 106, 126),
            head_hint=(175, 82),
        )
        self.assertTrue(result["valid"])
        self.assertTrue(result["head_hint_inside"])
        self.assertEqual(result["selection_mode"], "YOLO_HEAD_HINT")
        self.assertAlmostEqual(result["position"][0], 175.0, delta=3.0)
        self.assertLess(result["position"][1], 100.0)


if __name__ == "__main__":
    unittest.main()
