"""Measure detector-to-label box fit for one-object-per-class fish datasets."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import yaml
from ultralytics import YOLO


def box_iou(first: np.ndarray, second: np.ndarray) -> float:
    left_top = np.maximum(first[:2], second[:2])
    right_bottom = np.minimum(first[2:], second[2:])
    intersection = float(np.prod(np.maximum(0.0, right_bottom - left_top)))
    first_area = float(np.prod(np.maximum(0.0, first[2:] - first[:2])))
    second_area = float(np.prod(np.maximum(0.0, second[2:] - second[:2])))
    return intersection / max(1e-9, first_area + second_area - intersection)


def percentile(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"p05": None, "p25": None, "median": None, "p75": None, "p95": None}
    found = np.percentile(np.asarray(values), [5, 25, 50, 75, 95])
    return dict(zip(("p05", "p25", "median", "p75", "p95"), (float(item) for item in found)))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--split", choices=("train", "val", "test"), required=True)
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--device", default="0")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    config = yaml.safe_load(args.data.read_text(encoding="utf-8"))
    names = {int(key): value for key, value in config["names"].items()}
    split_file = args.data.parent / config[args.split]
    images = [str((split_file.parent / line).resolve()) for line in split_file.read_text(encoding="utf-8").splitlines() if line]
    per_class = {
        class_id: {"best_iou": [], "top_conf_iou": [], "best_area_ratio": [], "best_center_error_px": [], "best_conf": []}
        for class_id in names
    }

    model = YOLO(str(args.model))
    results = model.predict(images, conf=0.001, iou=0.7, max_det=30, imgsz=args.imgsz, device=args.device, verbose=False)
    for image_path, result in zip((Path(item) for item in images), results):
        height, width = result.orig_shape
        label_path = args.data.parent / "labels" / image_path.parent.name / (image_path.stem + ".txt")
        ground_truth = {}
        for line in label_path.read_text(encoding="utf-8").splitlines():
            class_id_text, x_text, y_text, width_text, height_text = line.split()
            class_id = int(class_id_text)
            x, y, box_width, box_height = map(float, (x_text, y_text, width_text, height_text))
            ground_truth[class_id] = np.asarray(
                [
                    (x - box_width / 2) * width,
                    (y - box_height / 2) * height,
                    (x + box_width / 2) * width,
                    (y + box_height / 2) * height,
                ],
                dtype=np.float64,
            )
        if result.boxes is None:
            continue
        predicted_boxes = result.boxes.xyxy.detach().cpu().numpy()
        predicted_classes = result.boxes.cls.detach().cpu().numpy().astype(int)
        predicted_confidences = result.boxes.conf.detach().cpu().numpy()
        for class_id, target in ground_truth.items():
            indices = np.flatnonzero(predicted_classes == class_id)
            if len(indices) == 0:
                continue
            ious = np.asarray([box_iou(predicted_boxes[index], target) for index in indices])
            best_index = int(indices[int(np.argmax(ious))])
            top_conf_index = int(indices[int(np.argmax(predicted_confidences[indices]))])
            best_box = predicted_boxes[best_index]
            best_area = float(np.prod(best_box[2:] - best_box[:2]))
            target_area = float(np.prod(target[2:] - target[:2]))
            best_centre = (best_box[:2] + best_box[2:]) * 0.5
            target_centre = (target[:2] + target[2:]) * 0.5
            values = per_class[class_id]
            values["best_iou"].append(float(ious.max()))
            values["top_conf_iou"].append(box_iou(predicted_boxes[top_conf_index], target))
            values["best_area_ratio"].append(best_area / max(1e-9, target_area))
            values["best_center_error_px"].append(float(np.linalg.norm(best_centre - target_centre)))
            values["best_conf"].append(float(predicted_confidences[best_index]))

    report = {"model": str(args.model.resolve()), "split": args.split, "images": len(images), "classes": {}}
    for class_id, values in per_class.items():
        report["classes"][names[class_id]] = {
            "matched": len(values["best_iou"]),
            "best_iou": percentile(values["best_iou"]),
            "top_conf_iou": percentile(values["top_conf_iou"]),
            "best_area_ratio": percentile(values["best_area_ratio"]),
            "best_center_error_px": percentile(values["best_center_error_px"]),
            "best_confidence": percentile(values["best_conf"]),
            "fraction_best_iou_ge_0_5": float(np.mean(np.asarray(values["best_iou"]) >= 0.5)) if values["best_iou"] else 0.0,
        }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
