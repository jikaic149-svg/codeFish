"""Evaluate semantic fish/rear/head detection and its derived yaw."""

from __future__ import annotations

import json
import math
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parents[4]
PROJECT = ROOT / "VISION/training/fish_pose_v1"
DATASET = PROJECT / "dataset_parts_v1"
EXPERIMENT = PROJECT / "runs/yolo11n-parts-960-v1"
BEST = EXPERIMENT / "train/weights/best.pt"
EVAL = EXPERIMENT / "eval"
SCENE = "auv_record_20260831_161512"
CLASS_IDS = {"fish": 0, "rigid_rear": 1, "head": 2}


def center(box):
    x1, y1, x2, y2 = box
    return np.asarray([(x1 + x2) * 0.5, (y1 + y2) * 0.5], dtype=np.float64)


def yaw_deg(rear, head):
    return math.degrees(math.atan2(head[1] - rear[1], head[0] - rear[0]))


def wrapped_error_deg(predicted, expected):
    return (predicted - expected + 180.0) % 360.0 - 180.0


def best_class_box(result, class_id):
    if result.boxes is None or not len(result.boxes):
        return None
    indices = (result.boxes.cls == class_id).nonzero(as_tuple=False).flatten()
    if not len(indices):
        return None
    index = int(indices[result.boxes.conf[indices].argmax()].item())
    return {
        "box": result.boxes.xyxy[index].cpu().numpy().astype(np.float64),
        "conf": float(result.boxes.conf[index]),
    }


def main():
    EVAL.mkdir(parents=True, exist_ok=True)
    model = YOLO(str(BEST))
    if model.task != "detect" or model.names != {0: "fish", 1: "rigid_rear", 2: "head"}:
        raise RuntimeError(f"Unexpected model schema: task={model.task}, names={model.names}")

    metrics = model.val(
        data=str(DATASET / "dataset.yaml"),
        split="test",
        imgsz=960,
        batch=8,
        device=0,
        project=str(EVAL),
        name="test",
        exist_ok=True,
        plots=True,
        verbose=True,
    )
    test_metrics = {key: float(value) for key, value in metrics.results_dict.items()}
    (EVAL / "test_metrics.json").write_text(
        json.dumps(test_metrics, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    audit = json.loads((DATASET / "audit.json").read_text(encoding="utf-8"))
    images = [DATASET / "images" / SCENE / f"{item['name']}.jpg" for item in audit["items"]]
    predictions = model.predict(images, imgsz=960, conf=0.05, device=0, verbose=False)

    records = []
    cells = []
    for item, image_path, result in zip(audit["items"], images, predictions, strict=True):
        gt_rear = center(item["boxes"]["rigid_rear"])
        gt_head = center(item["boxes"]["head"])
        gt_yaw = yaw_deg(gt_rear, gt_head)
        found = {name: best_class_box(result, class_id) for name, class_id in CLASS_IDS.items()}
        record = {
            "name": item["name"],
            "elapsed_s": item["elapsed_s"],
            "split": item["split"],
            "detected_classes": [name for name, value in found.items() if value is not None],
            "valid_yaw": found["rigid_rear"] is not None and found["head"] is not None,
            "gt_yaw_deg": round(gt_yaw, 3),
        }
        image = cv2.imread(str(image_path))
        cv2.arrowedLine(
            image,
            tuple(np.round(gt_rear).astype(int)),
            tuple(np.round(gt_head).astype(int)),
            (0, 180, 0),
            2,
            cv2.LINE_AA,
            tipLength=0.25,
        )
        for name, detection in found.items():
            if detection is None:
                continue
            record[f"{name}_conf"] = round(detection["conf"], 6)
            x1, y1, x2, y2 = np.round(detection["box"]).astype(int)
            color = {"fish": (255, 255, 0), "rigid_rear": (0, 165, 255), "head": (255, 0, 255)}[name]
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 1, cv2.LINE_AA)

        if record["valid_yaw"]:
            pred_rear = center(found["rigid_rear"]["box"])
            pred_head = center(found["head"]["box"])
            predicted_yaw = yaw_deg(pred_rear, pred_head)
            yaw_error = wrapped_error_deg(predicted_yaw, gt_yaw)
            record.update(
                {
                    "predicted_rear": [round(float(value), 3) for value in pred_rear],
                    "predicted_head": [round(float(value), 3) for value in pred_head],
                    "predicted_yaw_deg": round(predicted_yaw, 3),
                    "yaw_error_deg": round(yaw_error, 3),
                    "abs_yaw_error_deg": round(abs(yaw_error), 3),
                    "head_tail_flip": abs(yaw_error) > 90.0,
                }
            )
            cv2.arrowedLine(
                image,
                tuple(np.round(pred_rear).astype(int)),
                tuple(np.round(pred_head).astype(int)),
                (255, 255, 0),
                2,
                cv2.LINE_AA,
                tipLength=0.25,
            )
            label = f"{item['split']} err={abs(yaw_error):.1f}deg"
        else:
            missing = "/".join(name for name in ("rigid_rear", "head") if found[name] is None)
            label = f"{item['split']} MISS {missing}"
        records.append(record)
        cv2.rectangle(image, (0, 0), (260, 24), (0, 0, 0), -1)
        cv2.putText(image, label, (5, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (255, 255, 255), 1, cv2.LINE_AA)
        cells.append(cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA))

    by_split = {}
    for split in ("train", "val", "test"):
        selected = [record for record in records if record["split"] == split]
        valid = [record for record in selected if record["valid_yaw"]]
        by_split[split] = {
            "frames": len(selected),
            "valid_yaw": len(valid),
            "rear_detected": sum("rigid_rear" in record["detected_classes"] for record in selected),
            "head_detected": sum("head" in record["detected_classes"] for record in selected),
            "yaw_mae_deg": round(float(np.mean([record["abs_yaw_error_deg"] for record in valid])), 3) if valid else None,
            "yaw_max_error_deg": round(float(np.max([record["abs_yaw_error_deg"] for record in valid])), 3) if valid else None,
            "head_tail_flips": sum(bool(record["head_tail_flip"]) for record in valid),
        }
    valid = [record for record in records if record["valid_yaw"]]
    summary = {
        "weights": str(BEST),
        "frames": len(records),
        "valid_yaw": len(valid),
        "valid_yaw_rate": round(len(valid) / len(records), 6),
        "yaw_mae_deg": round(float(np.mean([record["abs_yaw_error_deg"] for record in valid])), 3),
        "yaw_median_error_deg": round(float(np.median([record["abs_yaw_error_deg"] for record in valid])), 3),
        "yaw_max_error_deg": round(float(np.max([record["abs_yaw_error_deg"] for record in valid])), 3),
        "head_tail_flips": sum(bool(record["head_tail_flip"]) for record in valid),
        "by_split": by_split,
        "test_ultralytics": test_metrics,
        "items": records,
    }
    (EVAL / "labeled_predictions.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    while len(cells) % 4:
        cells.append(np.zeros((240, 320, 3), dtype=np.uint8))
    contact = np.vstack([np.hstack(cells[index : index + 4]) for index in range(0, len(cells), 4)])
    contact_path = EVAL / "labeled_predictions_contact.jpg"
    cv2.imwrite(str(contact_path), contact, [cv2.IMWRITE_JPEG_QUALITY, 96])
    print(json.dumps({key: value for key, value in summary.items() if key != "items"}, ensure_ascii=False, indent=2))
    print(f"contact_sheet={contact_path}")


if __name__ == "__main__":
    main()
