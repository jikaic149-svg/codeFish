"""Causal full-video replay with anatomy and yaw-continuity association."""

from __future__ import annotations

import csv
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

from evaluate_parts_association import candidates, center, inside, yaw_deg


ROOT = Path(__file__).resolve().parents[4]
PROJECT = ROOT / "VISION/training/fish_pose_v1"
EXPERIMENT = PROJECT / "runs/yolo11n-parts-960-v1"
BEST = EXPERIMENT / "train/weights/best.pt"
VIDEO = ROOT / "output/vision/auv_record_20260831_161512.mp4"
OUTPUT = EXPERIMENT / "predictions"
MAX_ADJACENT_YAW_DELTA_DEG = 25.0
MAX_TRACK_GAP_FRAMES = 10


def wrapped_delta_deg(current, previous):
    return (current - previous + 180.0) % 360.0 - 180.0


def anatomy_candidates(result):
    fish_items = candidates(result, 0)
    if not fish_items:
        return None, []
    fish = max(fish_items, key=lambda item: item["conf"])
    fish_box = fish["box"]
    diagonal = float(np.linalg.norm(fish_box[2:] - fish_box[:2]))
    margin = 0.08 * diagonal
    rears = [item for item in candidates(result, 1) if inside(center(item["box"]), fish_box, margin)]
    heads = [item for item in candidates(result, 2) if inside(center(item["box"]), fish_box, margin)]
    pairs = []
    for rear in rears:
        for head in heads:
            rear_point = center(rear["box"])
            head_point = center(head["box"])
            ratio = float(np.linalg.norm(head_point - rear_point)) / max(diagonal, 1e-6)
            if not 0.18 <= ratio <= 0.55:
                continue
            geometry = math.exp(-((ratio - 0.354) / 0.18) ** 2)
            score = math.sqrt(rear["conf"] * head["conf"]) * geometry
            pairs.append(
                {
                    "rear": rear,
                    "head": head,
                    "yaw": yaw_deg(rear_point, head_point),
                    "score": score,
                    "distance_ratio": ratio,
                }
            )
    return fish, pairs


def choose_pair(pairs, previous_yaw, track_gap):
    if not pairs:
        return None
    if previous_yaw is None or track_gap > MAX_TRACK_GAP_FRAMES:
        return max(pairs, key=lambda pair: pair["score"])
    continuous = [
        pair
        for pair in pairs
        if abs(wrapped_delta_deg(pair["yaw"], previous_yaw)) <= MAX_ADJACENT_YAW_DELTA_DEG
    ]
    return max(continuous, key=lambda pair: pair["score"]) if continuous else None


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    capture = cv2.VideoCapture(str(VIDEO))
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open {VIDEO}")
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    expected_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    capture.release()

    video_path = OUTPUT / "auv_record_20260831_161512_parts_temporal.mp4"
    writer = cv2.VideoWriter(str(video_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"Cannot create {video_path}")
    model = YOLO(str(BEST))
    rows = []
    sample_cells = []
    previous_yaw = None
    previous_valid_frame = None
    invalid_streak = 0
    longest_invalid_streak = 0
    started = time.perf_counter()
    results = model.predict(
        source=str(VIDEO), stream=True, imgsz=960, conf=0.01, iou=0.7, max_det=100, device=0, verbose=False
    )
    for frame_index, result in enumerate(results):
        image = result.orig_img.copy()
        fish, pairs = anatomy_candidates(result)
        track_gap = frame_index - previous_valid_frame if previous_valid_frame is not None else MAX_TRACK_GAP_FRAMES + 1
        pair = choose_pair(pairs, previous_yaw, track_gap)
        row = {
            "frame": frame_index,
            "elapsed_s": frame_index / fps,
            "valid_yaw": int(pair is not None),
            "yaw_deg": "",
            "adjacent_delta_deg": "",
            "fish_conf": fish["conf"] if fish is not None else "",
            "rear_conf": "",
            "head_conf": "",
            "association_score": "",
            "distance_ratio": "",
            "candidate_pairs": len(pairs),
        }
        if fish is not None:
            x1, y1, x2, y2 = np.round(fish["box"]).astype(int)
            cv2.rectangle(image, (x1, y1), (x2, y2), (255, 255, 0), 1, cv2.LINE_AA)
        if pair is not None:
            invalid_streak = 0
            yaw = pair["yaw"]
            if previous_yaw is not None and previous_valid_frame == frame_index - 1:
                row["adjacent_delta_deg"] = wrapped_delta_deg(yaw, previous_yaw)
            row.update(
                {
                    "yaw_deg": yaw,
                    "rear_conf": pair["rear"]["conf"],
                    "head_conf": pair["head"]["conf"],
                    "association_score": pair["score"],
                    "distance_ratio": pair["distance_ratio"],
                }
            )
            previous_yaw = yaw
            previous_valid_frame = frame_index
            rear = center(pair["rear"]["box"])
            head = center(pair["head"]["box"])
            rear_i = tuple(np.round(rear).astype(int))
            head_i = tuple(np.round(head).astype(int))
            cv2.circle(image, rear_i, 4, (0, 165, 255), -1, cv2.LINE_AA)
            cv2.circle(image, head_i, 4, (255, 0, 255), -1, cv2.LINE_AA)
            cv2.arrowedLine(image, rear_i, head_i, (0, 255, 0), 2, cv2.LINE_AA, tipLength=0.25)
            status = f"yaw={yaw:+.1f}  score={pair['score']:.3f}"
            color = (0, 255, 0)
        else:
            invalid_streak += 1
            longest_invalid_streak = max(longest_invalid_streak, invalid_streak)
            status = "YAW INVALID (temporal gate)"
            color = (0, 0, 255)
        rows.append(row)
        cv2.rectangle(image, (0, 0), (370, 30), (0, 0, 0), -1)
        cv2.putText(image, status, (7, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.57, color, 1, cv2.LINE_AA)
        writer.write(image)
        if frame_index % max(1, int(round(5.0 * fps))) == 0:
            sample_cells.append(cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA))
    writer.release()
    runtime_s = time.perf_counter() - started

    csv_path = OUTPUT / "auv_record_20260831_161512_yaw_temporal.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        csv_writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        csv_writer.writeheader()
        csv_writer.writerows(rows)
    valid_rows = [row for row in rows if row["valid_yaw"]]
    deltas = np.asarray(
        [abs(float(row["adjacent_delta_deg"])) for row in rows if row["adjacent_delta_deg"] != ""], dtype=np.float64
    )
    metrics = {
        "source_video": str(VIDEO),
        "weights": str(BEST),
        "frames_expected": expected_frames,
        "frames_processed": len(rows),
        "fps": fps,
        "temporal_max_adjacent_delta_deg": MAX_ADJACENT_YAW_DELTA_DEG,
        "temporal_max_track_gap_frames": MAX_TRACK_GAP_FRAMES,
        "valid_yaw_frames": len(valid_rows),
        "invalid_yaw_frames": len(rows) - len(valid_rows),
        "valid_yaw_rate": len(valid_rows) / len(rows),
        "longest_invalid_streak_frames": longest_invalid_streak,
        "longest_invalid_streak_s": longest_invalid_streak / fps,
        "adjacent_valid_pairs": int(deltas.size),
        "adjacent_yaw_delta_p95_deg": float(np.percentile(deltas, 95)) if deltas.size else None,
        "adjacent_yaw_delta_max_deg": float(deltas.max()) if deltas.size else None,
        "adjacent_jumps_over_45_deg": int(np.sum(deltas > 45.0)),
        "adjacent_jumps_over_90_deg": int(np.sum(deltas > 90.0)),
        "runtime_s": runtime_s,
        "processing_fps": len(rows) / runtime_s,
        "limitation": "Unlabeled full-video frames are checked by causal continuity; only the 32 reviewed frames have absolute yaw ground truth.",
    }
    metrics_path = OUTPUT / "auv_record_20260831_161512_video_temporal_metrics.json"
    metrics_path.write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
    while len(sample_cells) % 4:
        sample_cells.append(np.zeros((240, 320, 3), dtype=np.uint8))
    contact = np.vstack([np.hstack(sample_cells[index : index + 4]) for index in range(0, len(sample_cells), 4)])
    contact_path = OUTPUT / "auv_record_20260831_161512_video_temporal_contact.jpg"
    cv2.imwrite(str(contact_path), contact, [cv2.IMWRITE_JPEG_QUALITY, 95])
    print(json.dumps(metrics, ensure_ascii=False, indent=2))
    print(f"video={video_path}")
    print(f"csv={csv_path}")
    print(f"contact_sheet={contact_path}")


if __name__ == "__main__":
    main()
