"""Tune yaw Kalman parameters on train/val and report untouched test metrics."""

from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
PROJECT = ROOT / "VISION/training/fish_pose_v1"
PREDICTIONS = PROJECT / "runs/yolo11n-parts-960-v1/predictions"
CSV_PATH = PREDICTIONS / "auv_record_20260831_161512_yaw_temporal.csv"
AUDIT_PATH = PROJECT / "dataset_parts_v1/audit.json"
sys.path.insert(0, str(ROOT / "VISION/runtime"))
from yaw_filter import BodyYawKalman, BodyYawKalmanConfig  # noqa: E402


def wrap_error(predicted, expected):
    return (predicted - expected + 180.0) % 360.0 - 180.0


def ground_truth(audit_item):
    rear = audit_item["boxes"]["rigid_rear"]
    head = audit_item["boxes"]["head"]
    return math.degrees(
        math.atan2((head[1] + head[3]) * 0.5 - (rear[1] + rear[3]) * 0.5,
                   (head[0] + head[2]) * 0.5 - (rear[0] + rear[2]) * 0.5)
    )


def run(rows, measurement_std_deg, acceleration_std_rps2):
    kalman = BodyYawKalman(
        BodyYawKalmanConfig(
            measurement_std_deg=measurement_std_deg,
            yaw_acceleration_std_rps2=acceleration_std_rps2,
        )
    )
    output = []
    previous = None
    for row in rows:
        raw_valid = row["valid_yaw"] == "1" and row["yaw_deg"] != ""
        raw = float(row["yaw_deg"]) if raw_valid else None
        confidence = math.sqrt(float(row["rear_conf"] or 0.0) * float(row["head_conf"] or 0.0)) if raw_valid else 0.0
        # The CSV is in degrees while the filter state is in radians.
        result = kalman.step(
            float(row["elapsed_s"]),
            math.radians(raw) if raw is not None else None,
            confidence=confidence,
            measurement_valid=raw_valid,
        )
        item = dict(row)
        item["filtered"] = result["yaw_deg"]
        item["delta"] = wrap_error(float(result["yaw_deg"]), previous) if result["yaw_deg"] is not None and previous is not None else None
        if result["yaw_deg"] is not None:
            previous = float(result["yaw_deg"])
        output.append(item)
    return output


def score(filtered, audit):
    by_frame = {int(row["frame"]): row for row in filtered}
    split_errors = {"train": [], "val": [], "test": []}
    for item in audit["items"]:
        row = by_frame[int(item["frame"])]
        if row["filtered"] is not None:
            split_errors[item["split"]].append(abs(wrap_error(float(row["filtered"]), ground_truth(item))))
    deltas = np.asarray([abs(float(row["delta"])) for row in filtered if row["delta"] is not None], dtype=np.float64)
    return {
        split: {
            "mae_deg": float(np.mean(values)) if values else None,
            "max_deg": float(np.max(values)) if values else None,
        }
        for split, values in split_errors.items()
    } | {
        "delta_p95_deg": float(np.percentile(deltas, 95)),
        "delta_max_deg": float(deltas.max()),
        "jumps_over_10_deg": int(np.sum(deltas > 10.0)),
    }


def main():
    rows = list(csv.DictReader(CSV_PATH.open(encoding="utf-8", newline="")))
    audit = json.loads(AUDIT_PATH.read_text(encoding="utf-8"))
    results = []
    for measurement_std in (0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0):
        for acceleration_std in (8.0, 12.0, 20.0, 30.0, 50.0, 100.0):
            filtered = run(rows, measurement_std, acceleration_std)
            metrics = score(filtered, audit)
            train_val_mae = np.mean([metrics[split]["mae_deg"] for split in ("train", "val")])
            # Prefer low train/val error, then low jitter; test remains report-only.
            metrics.update({"measurement_std_deg": measurement_std, "acceleration_std_rps2": acceleration_std, "train_val_mae_deg": float(train_val_mae)})
            results.append(metrics)
    results.sort(key=lambda item: (item["train_val_mae_deg"], item["delta_p95_deg"]))
    output = PREDICTIONS / "yaw_kalman_tuning.json"
    output.write_text(json.dumps({"selection": "lowest train+val MAE; test not used for selection", "results": results}, ensure_ascii=False, indent=2), encoding="utf-8")
    for item in results[:10]:
        print(json.dumps(item, ensure_ascii=False))
    print(f"output={output}")


if __name__ == "__main__":
    main()
