"""Associate low-confidence semantic parts inside the fish using anatomy constraints."""

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


def center(box):
    return np.asarray([(box[0] + box[2]) * 0.5, (box[1] + box[3]) * 0.5], dtype=np.float64)


def yaw_deg(rear, head):
    return math.degrees(math.atan2(head[1] - rear[1], head[0] - rear[0]))


def wrapped_error_deg(predicted, expected):
    return (predicted - expected + 180.0) % 360.0 - 180.0


def candidates(result, class_id):
    if result.boxes is None:
        return []
    output = []
    for box, conf, cls in zip(result.boxes.xyxy, result.boxes.conf, result.boxes.cls):
        if int(cls) == class_id:
            output.append({"box": box.cpu().numpy().astype(np.float64), "conf": float(conf)})
    return output


def inside(point, box, margin):
    return (
        box[0] - margin <= point[0] <= box[2] + margin
        and box[1] - margin <= point[1] <= box[3] + margin
    )


def associate(result):
    fish_candidates = candidates(result, 0)
    if not fish_candidates:
        return None
    fish = max(fish_candidates, key=lambda item: item["conf"])
    fish_box = fish["box"]
    diagonal = float(np.linalg.norm(fish_box[2:] - fish_box[:2]))
    margin = 0.08 * diagonal
    rears = [item for item in candidates(result, 1) if inside(center(item["box"]), fish_box, margin)]
    heads = [item for item in candidates(result, 2) if inside(center(item["box"]), fish_box, margin)]
    pairs = []
    for rear in rears:
        for head in heads:
            distance = float(np.linalg.norm(center(head["box"]) - center(rear["box"])))
            ratio = distance / max(diagonal, 1e-6)
            if not 0.18 <= ratio <= 0.55:
                continue
            geometry = math.exp(-((ratio - 0.354) / 0.18) ** 2)
            score = math.sqrt(rear["conf"] * head["conf"]) * geometry
            pairs.append((score, ratio, rear, head))
    if not pairs:
        return {"fish": fish, "pair": None, "rear_candidates": len(rears), "head_candidates": len(heads)}
    score, ratio, rear, head = max(pairs, key=lambda item: item[0])
    return {
        "fish": fish,
        "pair": {"rear": rear, "head": head, "score": score, "distance_ratio": ratio},
        "rear_candidates": len(rears),
        "head_candidates": len(heads),
    }


def main():
    model = YOLO(str(BEST))
    audit = json.loads((DATASET / "audit.json").read_text(encoding="utf-8"))
    image_paths = [DATASET / "images" / SCENE / f"{item['name']}.jpg" for item in audit["items"]]
    predictions = model.predict(image_paths, imgsz=960, conf=0.01, iou=0.7, device=0, verbose=False, max_det=100)
    records = []
    cells = []
    for item, image_path, result in zip(audit["items"], image_paths, predictions, strict=True):
        gt_rear = center(item["boxes"]["rigid_rear"])
        gt_head = center(item["boxes"]["head"])
        gt_yaw = yaw_deg(gt_rear, gt_head)
        association = associate(result)
        record = {
            "name": item["name"],
            "elapsed_s": item["elapsed_s"],
            "split": item["split"],
            "valid_yaw": association is not None and association["pair"] is not None,
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
        if association is not None:
            record["rear_candidates"] = association["rear_candidates"]
            record["head_candidates"] = association["head_candidates"]
            x1, y1, x2, y2 = np.round(association["fish"]["box"]).astype(int)
            cv2.rectangle(image, (x1, y1), (x2, y2), (255, 255, 0), 1, cv2.LINE_AA)
        if record["valid_yaw"]:
            pair = association["pair"]
            pred_rear = center(pair["rear"]["box"])
            pred_head = center(pair["head"]["box"])
            pred_yaw = yaw_deg(pred_rear, pred_head)
            error = wrapped_error_deg(pred_yaw, gt_yaw)
            record.update(
                {
                    "rear_conf": round(pair["rear"]["conf"], 6),
                    "head_conf": round(pair["head"]["conf"], 6),
                    "association_score": round(pair["score"], 6),
                    "distance_ratio": round(pair["distance_ratio"], 6),
                    "predicted_yaw_deg": round(pred_yaw, 3),
                    "abs_yaw_error_deg": round(abs(error), 3),
                    "head_tail_flip": abs(error) > 90.0,
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
            label = f"{item['split']} err={abs(error):.1f}deg"
        else:
            label = f"{item['split']} INVALID"
        records.append(record)
        cv2.rectangle(image, (0, 0), (230, 24), (0, 0, 0), -1)
        cv2.putText(image, label, (5, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (255, 255, 255), 1, cv2.LINE_AA)
        cells.append(cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA))

    by_split = {}
    for split in ("train", "val", "test"):
        selected = [record for record in records if record["split"] == split]
        valid = [record for record in selected if record["valid_yaw"]]
        by_split[split] = {
            "frames": len(selected),
            "valid_yaw": len(valid),
            "yaw_mae_deg": round(float(np.mean([record["abs_yaw_error_deg"] for record in valid])), 3) if valid else None,
            "yaw_max_error_deg": round(float(np.max([record["abs_yaw_error_deg"] for record in valid])), 3) if valid else None,
            "head_tail_flips": sum(bool(record["head_tail_flip"]) for record in valid),
        }
    valid = [record for record in records if record["valid_yaw"]]
    summary = {
        "weights": str(BEST),
        "association": {
            "candidate_conf_min": 0.01,
            "part_centers_inside_fish_margin_diagonal": 0.08,
            "endpoint_distance_ratio_range": [0.18, 0.55],
            "expected_distance_ratio": 0.354,
        },
        "frames": len(records),
        "valid_yaw": len(valid),
        "valid_yaw_rate": round(len(valid) / len(records), 6),
        "yaw_mae_deg": round(float(np.mean([record["abs_yaw_error_deg"] for record in valid])), 3),
        "yaw_median_error_deg": round(float(np.median([record["abs_yaw_error_deg"] for record in valid])), 3),
        "yaw_max_error_deg": round(float(np.max([record["abs_yaw_error_deg"] for record in valid])), 3),
        "head_tail_flips": sum(bool(record["head_tail_flip"]) for record in valid),
        "by_split": by_split,
        "items": records,
    }
    output_json = EVAL / "associated_predictions.json"
    output_json.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    while len(cells) % 4:
        cells.append(np.zeros((240, 320, 3), dtype=np.uint8))
    contact = np.vstack([np.hstack(cells[index : index + 4]) for index in range(0, len(cells), 4)])
    contact_path = EVAL / "associated_predictions_contact.jpg"
    cv2.imwrite(str(contact_path), contact, [cv2.IMWRITE_JPEG_QUALITY, 96])
    print(json.dumps({key: value for key, value in summary.items() if key != "items"}, ensure_ascii=False, indent=2))
    print(f"contact_sheet={contact_path}")


if __name__ == "__main__":
    main()
