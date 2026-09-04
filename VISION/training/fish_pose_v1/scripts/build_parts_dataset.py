"""Convert the reviewed three rectangles into a semantic part-detection dataset."""

from __future__ import annotations

import json
import shutil
from collections import Counter
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[4]
PROJECT = ROOT / "VISION/training/fish_pose_v1"
PENDING = PROJECT / "review/pending/seed_161512"
RAW_XLABEL = PENDING / "xlabel"
RAW_IMAGES = PENDING / "images"
DATASET = PROJECT / "dataset_parts_v1"
SCENE = "auv_record_20260831_161512"
CLASS_IDS = {"fish": 0, "rigid_rear": 1, "head": 2}


def bounds(shape):
    points = np.asarray(shape["points"], dtype=np.float64)
    return (
        float(points[:, 0].min()),
        float(points[:, 1].min()),
        float(points[:, 0].max()),
        float(points[:, 1].max()),
    )


def split_for_time(seconds):
    if seconds < 70.0:
        return "train"
    if seconds < 85.0:
        return "val"
    return "test"


def main():
    batch = json.loads((PENDING / "batch.json").read_text(encoding="utf-8"))
    metadata = {Path(item["name"]).stem: item for item in batch["items"]}
    image_dir = DATASET / "images" / SCENE
    label_dir = DATASET / "labels" / SCENE
    split_dir = DATASET / "splits"
    for directory in (image_dir, label_dir, split_dir):
        directory.mkdir(parents=True, exist_ok=True)

    split_items = {"train": [], "val": [], "test": []}
    audit_items = []
    for json_path in sorted(RAW_XLABEL.glob("*.json")):
        data = json.loads(json_path.read_text(encoding="utf-8"))
        labels = Counter(shape.get("label") for shape in data.get("shapes", []))
        if labels != Counter({"fish": 1, "rigid_rear": 1, "head": 1}):
            raise RuntimeError(f"Unexpected labels in {json_path.name}: {dict(labels)}")
        if any(shape.get("shape_type") != "rectangle" for shape in data["shapes"]):
            raise RuntimeError(f"Expected rectangles in {json_path.name}")

        width = int(data["imageWidth"])
        height = int(data["imageHeight"])
        lines = []
        boxes = {}
        for shape in data["shapes"]:
            label = shape["label"]
            x1, y1, x2, y2 = bounds(shape)
            if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
                raise RuntimeError(f"Invalid {label} box in {json_path.name}")
            boxes[label] = [round(x1, 3), round(y1, 3), round(x2, 3), round(y2, 3)]
            lines.append(
                f"{CLASS_IDS[label]} {(x1 + x2) / (2 * width):.6f} {(y1 + y2) / (2 * height):.6f} "
                f"{(x2 - x1) / width:.6f} {(y2 - y1) / height:.6f}"
            )

        stem = json_path.stem
        meta = metadata.get(stem)
        source_image = RAW_IMAGES / f"{stem}.jpg"
        if meta is None or not source_image.exists():
            raise RuntimeError(f"Missing source metadata/image for {stem}")
        shutil.copy2(source_image, image_dir / source_image.name)
        (label_dir / f"{stem}.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
        split = split_for_time(float(meta["elapsed_s"]))
        split_items[split].append(f"./../images/{SCENE}/{stem}.jpg")
        audit_items.append(
            {
                "name": stem,
                "frame": int(meta["frame"]),
                "elapsed_s": float(meta["elapsed_s"]),
                "split": split,
                "boxes": boxes,
            }
        )

    for split, entries in split_items.items():
        (split_dir / f"{split}.txt").write_text("\n".join(entries) + "\n", encoding="utf-8")
    (DATASET / "dataset.yaml").write_text(
        "train: splits/train.txt\n"
        "val: splits/val.txt\n"
        "test: splits/test.txt\n\n"
        "names:\n"
        "  0: fish\n"
        "  1: rigid_rear\n"
        "  2: head\n",
        encoding="utf-8",
    )
    audit = {
        "source_video": batch["source_video"],
        "class_ids": CLASS_IDS,
        "counts": {split: len(entries) for split, entries in split_items.items()},
        "total": len(audit_items),
        "items": audit_items,
        "limitation": "Test is a later block of the same source video, not an independent scene.",
    }
    (DATASET / "audit.json").write_text(
        json.dumps(audit, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps({"dataset": str(DATASET), "counts": audit["counts"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
