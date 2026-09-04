"""Extract a small, diverse human-reviewed YOLO Pose seed batch."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[4]
PROJECT = ROOT / "VISION/training/fish_pose_v1"
VIDEO = ROOT / "output/vision/auv_record_20260831_161512.mp4"
CSV_PATH = ROOT / "output/vision/auv_data_20260831_161512.csv"
CAL_PATH = ROOT / "output/vision/auv_calibration_20260831_161512.json"
CHECK_PATH = ROOT / "output/vision/auv_yaw_checks_20260831_161512.json"
PENDING = PROJECT / "review/pending/seed_161512"
IMAGES = PENDING / "images"
LABELS = PENDING / "labels"
BATCH_PATH = PENDING / "batch.json"


def load_manual_seeds():
    calibration = json.loads(CAL_PATH.read_text(encoding="utf-8"))
    seeds = {
        int(calibration["body_frame_index"]): [
            [*calibration["tail_pixel"], 2],
            [*calibration["head_pixel"], 2],
        ]
    }
    checks = json.loads(CHECK_PATH.read_text(encoding="utf-8"))
    for item in checks["targets"]:
        seeds[int(item["frame"])] = [[*point, 2] for point in item["points"]]
    return seeds


def derived_box(points, width, height):
    rear = np.asarray(points[0][:2], dtype=np.float64)
    head = np.asarray(points[1][:2], dtype=np.float64)
    vector = head - rear
    length = max(float(np.linalg.norm(vector)), 16.0)
    axis = vector / max(float(np.linalg.norm(vector)), 1e-9)
    normal = np.asarray([-axis[1], axis[0]])
    support = np.vstack(
        [
            rear - 0.90 * length * axis + 0.65 * length * normal,
            rear - 0.90 * length * axis - 0.65 * length * normal,
            head + 0.25 * length * axis + 0.65 * length * normal,
            head + 0.25 * length * axis - 0.65 * length * normal,
        ]
    )
    x1, y1 = np.min(support, axis=0)
    x2, y2 = np.max(support, axis=0)
    return [
        float(np.clip(x1, 0, width - 1)),
        float(np.clip(y1, 0, height - 1)),
        float(np.clip(x2, 1, width)),
        float(np.clip(y2, 1, height)),
    ]


def main():
    with CSV_PATH.open("r", newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    first_time = float(rows[0]["timestamp"])
    timestamps = np.asarray([float(row["timestamp"]) - first_time for row in rows])
    valid = np.asarray([bool(row["pixel_u"] and row["pixel_v"]) for row in rows])

    # Six early-run poses, eighteen main-run poses, plus the eight already
    # reviewed manual frames. Adjacent video frames are intentionally avoided.
    target_seconds = [3.0, 5.0, 7.0, 9.0, 12.0, 15.0]
    target_seconds.extend(np.linspace(30.0, 97.0, 18).tolist())
    selected = set()
    valid_indices = np.flatnonzero(valid)
    for seconds in target_seconds:
        nearest = int(valid_indices[np.argmin(np.abs(timestamps[valid_indices] - seconds))])
        selected.add(nearest)
    manual = load_manual_seeds()
    selected.update(manual)
    selected = sorted(selected)

    IMAGES.mkdir(parents=True, exist_ok=True)
    LABELS.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(str(VIDEO))
    if not cap.isOpened():
        raise SystemExit(f"Cannot open {VIDEO}")
    items = []
    for frame_index in selected:
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
        ok, frame = cap.read()
        if not ok:
            continue
        height, width = frame.shape[:2]
        name = f"auv_record_20260831_161512_f{frame_index:06d}.jpg"
        image_path = IMAGES / name
        cv2.imwrite(str(image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 97])
        points = manual.get(frame_index, [])
        row = rows[frame_index]
        if points:
            box = derived_box(points, width, height)
            status = "seeded_from_human"
        else:
            cx = float(row["pixel_u"])
            cy = float(row["pixel_v"])
            box = [
                float(np.clip(cx - 58, 0, width - 1)),
                float(np.clip(cy - 58, 0, height - 1)),
                float(np.clip(cx + 58, 1, width)),
                float(np.clip(cy + 58, 1, height)),
            ]
            status = "pending"
        items.append(
            {
                "name": name,
                "frame": frame_index,
                "elapsed_s": float(timestamps[frame_index]),
                "width": width,
                "height": height,
                "box": box,
                "keypoints": points,
                "status": status,
            }
        )
    cap.release()
    payload = {
        "version": 1,
        "source_video": str(VIDEO),
        "keypoint_order": ["rigid_rear", "head"],
        "items": items,
    }
    BATCH_PATH.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"batch": str(BATCH_PATH), "images": len(items), "human_seeded": sum(bool(x["keypoints"]) for x in items)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
