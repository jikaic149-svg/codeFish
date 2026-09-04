"""Evaluate best.pt on the held-out block and all 32 reviewed labels."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parents[4]
PROJECT = ROOT / "VISION/training/fish_pose_v1"
DATASET = PROJECT / "dataset_v1"
SCENE = "auv_record_20260831_161512"


def wrapped_error_deg(predicted: float, expected: float) -> float:
    return (predicted - expected + 180.0) % 360.0 - 180.0


def yaw_deg(rear, head) -> float:
    return math.degrees(math.atan2(head[1] - rear[1], head[0] - rear[0]))


def serializable_metrics(metrics):
    return {key: float(value) for key, value in metrics.results_dict.items()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--experiment", default="yolo11n-pose-960-seed32-v1")
    args = parser.parse_args()
    experiment = PROJECT / "runs" / args.experiment
    best = experiment / "train/weights/best.pt"
    eval_dir = experiment / "eval"
    eval_dir.mkdir(parents=True, exist_ok=True)
    model = YOLO(str(best))
    if model.task != "pose" or list(model.model.yaml["kpt_shape"]) != [2, 3]:
        raise RuntimeError("best.pt is not the expected two-keypoint pose model")

    test_metrics = model.val(
        data=str(DATASET / "dataset.yaml"),
        split="test",
        imgsz=960,
        batch=8,
        device=0,
        project=str(eval_dir),
        name="test",
        exist_ok=True,
        plots=True,
        verbose=True,
    )
    test_results = serializable_metrics(test_metrics)
    (eval_dir / "test_metrics.json").write_text(
        json.dumps(test_results, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    audit = json.loads((DATASET / "audit.json").read_text(encoding="utf-8"))
    images = [DATASET / "images" / SCENE / f"{item['name']}.jpg" for item in audit["items"]]
    predictions = model.predict(images, imgsz=960, conf=0.05, device=0, verbose=False)

    rows = []
    cells = []
    for item, image_path, result in zip(audit["items"], images, predictions, strict=True):
        gt_rear = np.asarray(item["rigid_rear"], dtype=np.float64)
        gt_head = np.asarray(item["head"], dtype=np.float64)
        gt_yaw = yaw_deg(gt_rear, gt_head)
        record = {
            "name": item["name"],
            "elapsed_s": item["elapsed_s"],
            "split": item["split"],
            "detected": False,
            "gt_yaw_deg": round(gt_yaw, 3),
        }
        image = cv2.imread(str(image_path))
        gt_rear_i = tuple(np.round(gt_rear).astype(int))
        gt_head_i = tuple(np.round(gt_head).astype(int))
        cv2.arrowedLine(image, gt_rear_i, gt_head_i, (0, 180, 0), 2, cv2.LINE_AA, tipLength=0.25)

        if result.boxes is not None and len(result.boxes) and result.keypoints is not None:
            index = int(result.boxes.conf.argmax().item())
            keypoints = result.keypoints.data[index].cpu().numpy()
            pred_rear = keypoints[0, :2].astype(np.float64)
            pred_head = keypoints[1, :2].astype(np.float64)
            pred_yaw = yaw_deg(pred_rear, pred_head)
            yaw_error = wrapped_error_deg(pred_yaw, gt_yaw)
            rear_error = float(np.linalg.norm(pred_rear - gt_rear))
            head_error = float(np.linalg.norm(pred_head - gt_head))
            record.update(
                {
                    "detected": True,
                    "box_conf": round(float(result.boxes.conf[index]), 6),
                    "keypoint_conf": [round(float(value), 6) for value in keypoints[:, 2]],
                    "predicted_rear": [round(float(value), 3) for value in pred_rear],
                    "predicted_head": [round(float(value), 3) for value in pred_head],
                    "predicted_yaw_deg": round(pred_yaw, 3),
                    "yaw_error_deg": round(yaw_error, 3),
                    "abs_yaw_error_deg": round(abs(yaw_error), 3),
                    "rear_error_px": round(rear_error, 3),
                    "head_error_px": round(head_error, 3),
                    "head_tail_flip": abs(yaw_error) > 90.0,
                }
            )
            pred_rear_i = tuple(np.round(pred_rear).astype(int))
            pred_head_i = tuple(np.round(pred_head).astype(int))
            cv2.circle(image, pred_rear_i, 4, (255, 255, 0), -1, cv2.LINE_AA)
            cv2.circle(image, pred_head_i, 4, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.arrowedLine(image, pred_rear_i, pred_head_i, (255, 255, 0), 2, cv2.LINE_AA, tipLength=0.25)
            label = f"{item['split']} err={abs(yaw_error):.1f}deg conf={record['box_conf']:.2f}"
        else:
            label = f"{item['split']} MISS"
        rows.append(record)
        cv2.rectangle(image, (0, 0), (320, 24), (0, 0, 0), -1)
        cv2.putText(image, label, (5, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (255, 255, 255), 1, cv2.LINE_AA)
        cells.append(cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA))

    detected = [row for row in rows if row["detected"]]
    by_split = {}
    for split in ("train", "val", "test"):
        selected = [row for row in rows if row["split"] == split]
        valid = [row for row in selected if row["detected"]]
        by_split[split] = {
            "frames": len(selected),
            "detected": len(valid),
            "yaw_mae_deg": round(float(np.mean([row["abs_yaw_error_deg"] for row in valid])), 3) if valid else None,
            "yaw_max_error_deg": round(float(np.max([row["abs_yaw_error_deg"] for row in valid])), 3) if valid else None,
            "head_tail_flips": sum(bool(row["head_tail_flip"]) for row in valid),
        }
    summary = {
        "weights": str(best),
        "schema": ["rigid_rear", "head"],
        "frames": len(rows),
        "detected": len(detected),
        "detection_rate": round(len(detected) / len(rows), 6),
        "yaw_mae_deg": round(float(np.mean([row["abs_yaw_error_deg"] for row in detected])), 3),
        "yaw_median_error_deg": round(float(np.median([row["abs_yaw_error_deg"] for row in detected])), 3),
        "yaw_max_error_deg": round(float(np.max([row["abs_yaw_error_deg"] for row in detected])), 3),
        "head_tail_flips": sum(bool(row["head_tail_flip"]) for row in detected),
        "by_split": by_split,
        "test_ultralytics": test_results,
        "items": rows,
    }
    (eval_dir / "labeled_predictions.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    while len(cells) % 4:
        cells.append(np.zeros((240, 320, 3), dtype=np.uint8))
    contact = np.vstack([np.hstack(cells[index : index + 4]) for index in range(0, len(cells), 4)])
    contact_path = eval_dir / "labeled_predictions_contact.jpg"
    cv2.imwrite(str(contact_path), contact, [cv2.IMWRITE_JPEG_QUALITY, 96])
    print(json.dumps({key: value for key, value in summary.items() if key != "items"}, ensure_ascii=False, indent=2))
    print(f"contact_sheet={contact_path}")


if __name__ == "__main__":
    main()
