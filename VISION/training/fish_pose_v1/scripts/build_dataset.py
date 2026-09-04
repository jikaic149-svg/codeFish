"""Audit reviewed X-AnyLabeling regions and build a two-keypoint YOLO Pose dataset."""

from __future__ import annotations

import json
import shutil
from collections import Counter
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[4]
PROJECT = ROOT / "VISION/training/fish_pose_v1"
PENDING = PROJECT / "review/pending/seed_161512"
RAW_XLABEL = PENDING / "xlabel"
RAW_IMAGES = PENDING / "images"
CONFIRMED = PROJECT / "review/confirmed/seed_161512"
DATASET = PROJECT / "dataset_v1"
SCENE = "auv_record_20260831_161512"


def rectangle_bounds(shape):
    points = np.asarray(shape["points"], dtype=np.float64)
    return [
        float(np.min(points[:, 0])),
        float(np.min(points[:, 1])),
        float(np.max(points[:, 0])),
        float(np.max(points[:, 1])),
    ]


def rectangle_center(shape):
    x1, y1, x2, y2 = rectangle_bounds(shape)
    return [0.5 * (x1 + x2), 0.5 * (y1 + y2)]


def canonical_rectangle(label, box, group_id=1):
    x1, y1, x2, y2 = box
    return {
        "label": label,
        "score": None,
        "points": [[x1, y1], [x2, y1], [x2, y2], [x1, y2]],
        "group_id": group_id,
        "description": "",
        "difficult": False,
        "shape_type": "rectangle",
        "flags": {},
        "attributes": {},
        "kie_linking": [],
    }


def canonical_point(label, point, group_id=1):
    return {
        "label": label,
        "score": None,
        "points": [[float(point[0]), float(point[1])]],
        "group_id": group_id,
        "description": "",
        "difficult": False,
        "shape_type": "point",
        "flags": {},
        "attributes": {},
        "kie_linking": [],
    }


def split_for_time(seconds):
    # Contiguous time blocks prevent neighboring-frame leakage.
    if seconds < 70.0:
        return "train"
    if seconds < 85.0:
        return "val"
    return "test"


def main():
    batch = json.loads((PENDING / "batch.json").read_text(encoding="utf-8"))
    metadata = {Path(item["name"]).stem: item for item in batch["items"]}
    json_files = sorted(RAW_XLABEL.glob("*.json"))
    if not json_files:
        raise SystemExit("No reviewed X-AnyLabeling JSON files")

    confirmed_xlabels = CONFIRMED / "xlabel"
    image_dir = DATASET / "images" / SCENE
    label_dir = DATASET / "labels" / SCENE
    split_dir = DATASET / "splits"
    for directory in (confirmed_xlabels, image_dir, label_dir, split_dir):
        directory.mkdir(parents=True, exist_ok=True)

    split_items = {"train": [], "val": [], "test": []}
    audit_items = []
    preview_rows = []
    failures = []

    for json_path in json_files:
        data = json.loads(json_path.read_text(encoding="utf-8"))
        labels = Counter(shape.get("label") for shape in data.get("shapes", []))
        expected = Counter({"fish": 1, "rigid_rear": 1, "head": 1})
        if labels != expected:
            failures.append({"file": json_path.name, "reason": f"labels={dict(labels)}"})
            continue
        by_label = {shape["label"]: shape for shape in data["shapes"]}
        if any(by_label[name].get("shape_type") != "rectangle" for name in expected):
            failures.append({"file": json_path.name, "reason": "expected three reviewed rectangles"})
            continue

        width = int(data["imageWidth"])
        height = int(data["imageHeight"])
        fish_box = rectangle_bounds(by_label["fish"])
        rear = rectangle_center(by_label["rigid_rear"])
        head = rectangle_center(by_label["head"])

        # A keypoint centre must belong to its instance. Expand a human box by
        # only the minimum required amount if its marked region touches an edge.
        fish_box[0] = max(0.0, min(fish_box[0], rear[0] - 1.0, head[0] - 1.0))
        fish_box[1] = max(0.0, min(fish_box[1], rear[1] - 1.0, head[1] - 1.0))
        fish_box[2] = min(float(width), max(fish_box[2], rear[0] + 1.0, head[0] + 1.0))
        fish_box[3] = min(float(height), max(fish_box[3], rear[1] + 1.0, head[1] + 1.0))
        x1, y1, x2, y2 = fish_box
        if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
            failures.append({"file": json_path.name, "reason": "invalid fish box"})
            continue
        if not all(0 <= point[0] <= width and 0 <= point[1] <= height for point in (rear, head)):
            failures.append({"file": json_path.name, "reason": "keypoint outside image"})
            continue

        stem = json_path.stem
        meta = metadata.get(stem)
        if meta is None:
            failures.append({"file": json_path.name, "reason": "missing batch metadata"})
            continue
        image_source = RAW_IMAGES / f"{stem}.jpg"
        if not image_source.exists():
            failures.append({"file": json_path.name, "reason": "missing image"})
            continue

        canonical = dict(data)
        canonical["shapes"] = [
            canonical_rectangle("fish", fish_box),
            canonical_point("rigid_rear", rear),
            canonical_point("head", head),
        ]
        canonical["imagePath"] = f"../../../../dataset_v1/images/{SCENE}/{stem}.jpg"
        (confirmed_xlabels / json_path.name).write_text(
            json.dumps(canonical, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        shutil.copy2(image_source, image_dir / image_source.name)

        box_cx = 0.5 * (x1 + x2) / width
        box_cy = 0.5 * (y1 + y2) / height
        box_w = (x2 - x1) / width
        box_h = (y2 - y1) / height
        line = (
            f"0 {box_cx:.6f} {box_cy:.6f} {box_w:.6f} {box_h:.6f} "
            f"{rear[0] / width:.6f} {rear[1] / height:.6f} 2 "
            f"{head[0] / width:.6f} {head[1] / height:.6f} 2\n"
        )
        (label_dir / f"{stem}.txt").write_text(line, encoding="utf-8")

        split = split_for_time(float(meta["elapsed_s"]))
        split_items[split].append(f"./../images/{SCENE}/{stem}.jpg")
        distance = float(np.linalg.norm(np.asarray(head) - np.asarray(rear)))
        audit_items.append(
            {
                "name": stem,
                "frame": int(meta["frame"]),
                "elapsed_s": float(meta["elapsed_s"]),
                "split": split,
                "box": [round(value, 3) for value in fish_box],
                "rigid_rear": [round(value, 3) for value in rear],
                "head": [round(value, 3) for value in head],
                "keypoint_distance_px": round(distance, 3),
            }
        )
        preview_rows.append((image_dir / image_source.name, fish_box, rear, head, split))

    if failures:
        raise SystemExit(json.dumps({"failures": failures}, ensure_ascii=False, indent=2))

    for split, entries in split_items.items():
        (split_dir / f"{split}.txt").write_text("\n".join(entries) + "\n", encoding="utf-8")
    (DATASET / "dataset.yaml").write_text(
        "train: splits/train.txt\n"
        "val: splits/val.txt\n"
        "test: splits/test.txt\n\n"
        "kpt_shape: [2, 3]\n"
        "flip_idx: [0, 1]\n\n"
        "kpt_oks_sigmas: [0.05, 0.05]\n\n"
        "names:\n"
        "  0: fish\n",
        encoding="utf-8",
    )
    audit = {
        "source_video": batch["source_video"],
        "schema": {"box": "fish", "keypoints": ["rigid_rear", "head"]},
        "counts": {split: len(items) for split, items in split_items.items()},
        "total": len(audit_items),
        "failures": failures,
        "items": audit_items,
        "limitation": "All labels come from one video; test is a held-out contiguous time block, not an independent scene.",
    }
    (DATASET / "audit.json").write_text(json.dumps(audit, ensure_ascii=False, indent=2), encoding="utf-8")

    cells = []
    for image_path, box, rear, head, split in preview_rows:
        image = cv2.imread(str(image_path))
        x1, y1, x2, y2 = [int(round(value)) for value in box]
        cv2.rectangle(image, (x1, y1), (x2, y2), (255, 255, 0), 2, cv2.LINE_AA)
        rear_i = tuple(np.round(rear).astype(int))
        head_i = tuple(np.round(head).astype(int))
        cv2.circle(image, rear_i, 6, (0, 165, 255), -1, cv2.LINE_AA)
        cv2.circle(image, head_i, 6, (255, 0, 255), -1, cv2.LINE_AA)
        cv2.arrowedLine(image, rear_i, head_i, (0, 255, 0), 2, cv2.LINE_AA, tipLength=0.25)
        cv2.rectangle(image, (0, 0), (160, 24), (0, 0, 0), -1)
        cv2.putText(image, split, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
        cells.append(cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA))
    while len(cells) % 4:
        cells.append(np.zeros((240, 320, 3), dtype=np.uint8))
    sheet = np.vstack([np.hstack(cells[index : index + 4]) for index in range(0, len(cells), 4)])
    sheet_path = PROJECT / "review/confirmed/seed_161512_contact.jpg"
    cv2.imwrite(str(sheet_path), sheet, [cv2.IMWRITE_JPEG_QUALITY, 96])
    print(json.dumps({"dataset": str(DATASET), "counts": audit["counts"], "preview": str(sheet_path)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
