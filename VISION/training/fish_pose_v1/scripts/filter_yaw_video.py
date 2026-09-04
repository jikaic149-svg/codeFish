"""Apply and evaluate the body-yaw Kalman filter on the replayed video CSV."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np

import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "VISION/runtime"))
from yaw_filter import BodyYawKalman, BodyYawKalmanConfig  # noqa: E402


PROJECT = ROOT / "VISION/training/fish_pose_v1"
EXPERIMENT = PROJECT / "runs/yolo11n-parts-960-v1"
PREDICTIONS = EXPERIMENT / "predictions"
INPUT_VIDEO = PREDICTIONS / "auv_record_20260831_161512_parts_temporal.mp4"
INPUT_CSV = PREDICTIONS / "auv_record_20260831_161512_yaw_temporal.csv"
GT_AUDIT = PROJECT / "dataset_parts_v1/audit.json"


def wrapped_error_deg(predicted: float, expected: float) -> float:
    return (predicted - expected + 180.0) % 360.0 - 180.0


def main():
    rows = list(csv.DictReader(INPUT_CSV.open(encoding="utf-8", newline="")))
    capture = cv2.VideoCapture(str(INPUT_VIDEO))
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open {INPUT_VIDEO}")
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    output_video = PREDICTIONS / "auv_record_20260831_161512_parts_temporal_kalman.mp4"
    writer = cv2.VideoWriter(str(output_video), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"Cannot create {output_video}")

    # Selected on train/val blocks by lowest train+val yaw MAE; the later test
    # block is only reported below, never used for this selection.
    # Keep replay validation identical to the real-time runtime.
    measurement_std_deg = 3.0
    yaw_acceleration_std_rps2 = 10.0
    kalman = BodyYawKalman(
        BodyYawKalmanConfig(
            measurement_std_deg=measurement_std_deg,
            yaw_acceleration_std_rps2=yaw_acceleration_std_rps2,
        )
    )
    filtered_rows = []
    previous_filtered = None
    for row in rows:
        raw_valid = row["valid_yaw"] == "1" and row["yaw_deg"] != ""
        raw_yaw = float(row["yaw_deg"]) if raw_valid else None
        if raw_valid:
            rear_conf = float(row["rear_conf"] or 0.0)
            head_conf = float(row["head_conf"] or 0.0)
            confidence = math.sqrt(max(0.0, rear_conf * head_conf))
        else:
            confidence = 0.0
        filtered = kalman.step(
            float(row["elapsed_s"]),
            # BodyYawKalman operates in radians; the replay CSV stores
            # human-readable degrees.  Passing degrees here creates fake
            # 180-degree jumps and invalidates the replay video/metrics.
            math.radians(raw_yaw) if raw_yaw is not None else None,
            confidence=confidence,
            measurement_valid=raw_valid,
        )
        filtered_row = dict(row)
        filtered_row["raw_yaw_deg"] = raw_yaw if raw_valid else ""
        filtered_row["filtered_yaw_deg"] = filtered["yaw_deg"] if filtered["yaw_deg"] is not None else ""
        filtered_row["filtered_yaw_rate_rps"] = filtered["yaw_rate_rps"] if filtered["yaw_rate_rps"] is not None else ""
        filtered_row["filtered_usable"] = int(bool(filtered["usable"]))
        filtered_row["filtered_source"] = filtered["source"]
        filtered_row["filtered_prediction_age_s"] = filtered["prediction_age_s"] if filtered["prediction_age_s"] is not None else ""
        if filtered["yaw_deg"] is not None and previous_filtered is not None:
            filtered_row["filtered_adjacent_delta_deg"] = wrapped_error_deg(float(filtered["yaw_deg"]), previous_filtered)
        else:
            filtered_row["filtered_adjacent_delta_deg"] = ""
        if filtered["yaw_deg"] is not None:
            previous_filtered = float(filtered["yaw_deg"])
        filtered_rows.append(filtered_row)

    # Render the source replay again, adding raw and filtered values. The source
    # already contains the endpoint arrow; this makes the filter's effect clear
    # without rerunning the detector.
    capture = cv2.VideoCapture(str(INPUT_VIDEO))
    for row in filtered_rows:
        ok, image = capture.read()
        if not ok:
            break
        if row["filtered_yaw_deg"] != "":
            raw_text = f"{float(row['yaw_deg']):+.1f}" if row["yaw_deg"] != "" else "MISS"
            text = f"raw={raw_text}  KF={float(row['filtered_yaw_deg']):+.1f}"
            color = (0, 255, 255) if row["filtered_source"] == "PREDICTION" else (0, 255, 0)
        else:
            text = "YAW INVALID"
            color = (0, 0, 255)
        cv2.rectangle(image, (0, 30), (380, 58), (0, 0, 0), -1)
        cv2.putText(image, text, (7, 51), cv2.FONT_HERSHEY_SIMPLEX, 0.54, color, 1, cv2.LINE_AA)
        writer.write(image)
    capture.release()
    writer.release()

    output_csv = PREDICTIONS / "auv_record_20260831_161512_yaw_temporal_kalman.csv"
    with output_csv.open("w", newline="", encoding="utf-8") as handle:
        csv_writer = csv.DictWriter(handle, fieldnames=list(filtered_rows[0]))
        csv_writer.writeheader()
        csv_writer.writerows(filtered_rows)

    raw_deltas = np.asarray([abs(float(row["adjacent_delta_deg"])) for row in rows if row["adjacent_delta_deg"] != ""], dtype=np.float64)
    filtered_deltas = np.asarray([abs(float(row["filtered_adjacent_delta_deg"])) for row in filtered_rows if row["filtered_adjacent_delta_deg"] != ""], dtype=np.float64)
    metrics = {
        "input_video": str(INPUT_VIDEO),
        "output_video": str(output_video),
        "filter": {
            "measurement_std_deg": measurement_std_deg,
            "yaw_acceleration_std_rps2": yaw_acceleration_std_rps2,
            "max_prediction_s": 0.40,
            "max_innovation_deg": 85.0,
            "reacquire_frames": 3,
            "reacquire_consistency_deg": 35.0,
        },
        "frames": len(filtered_rows),
        "raw_valid_frames": sum(row["valid_yaw"] == "1" for row in rows),
        "filtered_usable_frames": sum(row["filtered_usable"] == 1 for row in filtered_rows),
        "raw_adjacent_delta_p95_deg": float(np.percentile(raw_deltas, 95)),
        "filtered_adjacent_delta_p95_deg": float(np.percentile(filtered_deltas, 95)),
        "raw_adjacent_delta_max_deg": float(raw_deltas.max()),
        "filtered_adjacent_delta_max_deg": float(filtered_deltas.max()),
        "raw_jumps_over_45_deg": int(np.sum(raw_deltas > 45.0)),
        "filtered_jumps_over_45_deg": int(np.sum(filtered_deltas > 45.0)),
        "raw_jumps_over_90_deg": int(np.sum(raw_deltas > 90.0)),
        "filtered_jumps_over_90_deg": int(np.sum(filtered_deltas > 90.0)),
    }

    audit = json.loads(GT_AUDIT.read_text(encoding="utf-8"))
    reviewed = []
    by_frame = {int(row["frame"]): row for row in filtered_rows}
    for item in audit["items"]:
        row = by_frame[int(item["frame"])]
        fish = item["boxes"]
        rear = fish["rigid_rear"]
        head = fish["head"]
        gt = math.degrees(math.atan2((head[1] + head[3]) * 0.5 - (rear[1] + rear[3]) * 0.5, (head[0] + head[2]) * 0.5 - (rear[0] + rear[2]) * 0.5))
        if row["filtered_yaw_deg"] == "":
            continue
        error = abs(wrapped_error_deg(float(row["filtered_yaw_deg"]), gt))
        reviewed.append({"frame": int(item["frame"]), "split": item["split"], "abs_error_deg": error})
    metrics["reviewed_32"] = {
        "valid": len(reviewed),
        "yaw_mae_deg": float(np.mean([row["abs_error_deg"] for row in reviewed])),
        "yaw_max_error_deg": float(np.max([row["abs_error_deg"] for row in reviewed])),
        "head_tail_flips": sum(row["abs_error_deg"] > 90.0 for row in reviewed),
        "test_mae_deg": float(np.mean([row["abs_error_deg"] for row in reviewed if row["split"] == "test"])),
        "test_max_error_deg": float(np.max([row["abs_error_deg"] for row in reviewed if row["split"] == "test"])),
        "test_head_tail_flips": sum(row["abs_error_deg"] > 90.0 for row in reviewed if row["split"] == "test"),
    }
    metrics_path = PREDICTIONS / "auv_record_20260831_161512_yaw_kalman_metrics.json"
    metrics_path.write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(metrics, ensure_ascii=False, indent=2))
    print(f"video={output_video}")
    print(f"csv={output_csv}")
    print(f"metrics={metrics_path}")


if __name__ == "__main__":
    main()
