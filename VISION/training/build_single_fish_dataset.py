"""Bootstrap a reviewed single-fish detection dataset from a clean recording.

The existing YOLO box is only a weak spatial seed.  A GrabCut foreground mask
and the orange-tail/magenta-head colour markers tighten and validate each box.
Outputs are kept under ``review/pending`` until the generated contact sheets
have been inspected; ``--promote`` then creates the formal YOLO dataset.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


@dataclass(frozen=True)
class PoseMarkers:
    tail: tuple[float, float] | None
    head: tuple[float, float] | None

    @property
    def midpoint(self) -> tuple[float, float] | None:
        if self.tail is None or self.head is None:
            return None
        return ((self.tail[0] + self.head[0]) * 0.5, (self.tail[1] + self.head[1]) * 0.5)


def _components(mask: np.ndarray, *, min_area: int, max_area: int) -> list[tuple[float, float, int]]:
    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    found: list[tuple[float, float, int]] = []
    for index in range(1, count):
        area = int(stats[index, cv2.CC_STAT_AREA])
        if min_area <= area <= max_area:
            found.append((float(centroids[index, 0]), float(centroids[index, 1]), area))
    return found


def detect_pose_markers(frame: np.ndarray, previous_midpoint=None) -> PoseMarkers:
    """Find the existing orange tail and magenta head markers in the pool ROI."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    height, width = hsv.shape[:2]
    pool = np.zeros((height, width), np.uint8)
    # Exclude the blue rim, clamps and deck.  Those contain small orange and
    # magenta highlights that resemble the fish markers but never enter the
    # actual swimming area.
    pool[int(height * 0.16) : int(height * 0.87), int(width * 0.07) : int(width * 0.93)] = 255

    orange = (
        (hsv[:, :, 0] <= 26)
        & (hsv[:, :, 1] >= 65)
        & (hsv[:, :, 2] >= 45)
        & (pool > 0)
    ).astype(np.uint8) * 255
    magenta = (
        (hsv[:, :, 0] >= 125)
        & (hsv[:, :, 0] <= 175)
        & (hsv[:, :, 1] >= 35)
        & (hsv[:, :, 2] >= 55)
        & (pool > 0)
    ).astype(np.uint8) * 255
    kernel = np.ones((3, 3), np.uint8)
    orange = cv2.morphologyEx(orange, cv2.MORPH_OPEN, kernel)
    magenta = cv2.morphologyEx(magenta, cv2.MORPH_OPEN, kernel)
    tails = _components(orange, min_area=5, max_area=900)
    heads = _components(magenta, min_area=3, max_area=500)

    best = None
    best_score = float("-inf")
    for tx, ty, tail_area in tails:
        for hx, hy, head_area in heads:
            distance = float(np.hypot(hx - tx, hy - ty))
            if not 7.0 <= distance <= 115.0:
                continue
            midpoint = np.asarray([(tx + hx) * 0.5, (ty + hy) * 0.5])
            temporal = 0.0
            if previous_midpoint is not None:
                temporal = -0.025 * float(np.linalg.norm(midpoint - previous_midpoint))
            score = min(tail_area, 120) / 30.0 + min(head_area, 80) / 25.0 + temporal
            if score > best_score:
                best_score = score
                best = PoseMarkers((tx, ty), (hx, hy))
    if best is not None:
        return best
    return PoseMarkers(None, None)


def _clip_box(box, width: int, height: int) -> tuple[float, float, float, float]:
    x1, y1, x2, y2 = [float(value) for value in box]
    return (
        float(np.clip(x1, 0, width - 1)),
        float(np.clip(y1, 0, height - 1)),
        float(np.clip(x2, 1, width)),
        float(np.clip(y2, 1, height)),
    )


def _point_box_distance(point, box) -> float:
    x, y = point
    x1, y1, x2, y2 = box
    return float(np.hypot(max(x1 - x, 0.0, x - x2), max(y1 - y, 0.0, y - y2)))


def select_seed_box(result, pose: PoseMarkers, previous_center, shape):
    height, width = shape[:2]
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return None, 0.0
    xyxy = boxes.xyxy.detach().cpu().numpy()
    confidence = boxes.conf.detach().cpu().numpy()
    best = None
    best_confidence = 0.0
    best_score = float("-inf")
    for raw_box, conf in zip(xyxy, confidence):
        box = _clip_box(raw_box, width, height)
        x1, y1, x2, y2 = box
        area = (x2 - x1) * (y2 - y1)
        centre = np.asarray([(x1 + x2) * 0.5, (y1 + y2) * 0.5])
        if not (100.0 <= area <= 25000.0 and 45 <= centre[0] <= width - 35 and 65 <= centre[1] <= height - 35):
            continue
        score = 2.0 * float(conf)
        if pose.tail is not None:
            score += max(0.0, 2.2 - _point_box_distance(pose.tail, box) / 20.0)
        if pose.head is not None:
            score += max(0.0, 2.2 - _point_box_distance(pose.head, box) / 20.0)
        if previous_center is not None:
            score -= 0.012 * float(np.linalg.norm(centre - previous_center))
        if score > best_score:
            best_score = score
            best = box
            best_confidence = float(conf)
    return best, best_confidence


def refine_box(frame: np.ndarray, seed_box, pose: PoseMarkers):
    """Tighten a weak detector box using the existing foreground segmentation."""
    height, width = frame.shape[:2]
    x1, y1, x2, y2 = seed_box
    pad_x = max(8.0, (x2 - x1) * 0.28)
    pad_y = max(8.0, (y2 - y1) * 0.28)
    ex1, ey1, ex2, ey2 = _clip_box((x1 - pad_x, y1 - pad_y, x2 + pad_x, y2 + pad_y), width, height)
    ix1, iy1, ix2, iy2 = [int(round(value)) for value in (ex1, ey1, ex2, ey2)]
    crop = frame[iy1:iy2, ix1:ix2]
    if crop.shape[0] < 8 or crop.shape[1] < 8:
        return seed_box, "seed"

    rx = max(1, int(round(x1 - ix1)))
    ry = max(1, int(round(y1 - iy1)))
    rw = min(crop.shape[1] - rx - 1, max(2, int(round(x2 - x1))))
    rh = min(crop.shape[0] - ry - 1, max(2, int(round(y2 - y1))))
    if rw <= 1 or rh <= 1:
        return seed_box, "seed"
    mask = np.zeros(crop.shape[:2], np.uint8)
    background = np.zeros((1, 65), np.float64)
    foreground = np.zeros((1, 65), np.float64)
    try:
        cv2.grabCut(crop, mask, (rx, ry, rw, rh), background, foreground, 4, cv2.GC_INIT_WITH_RECT)
    except cv2.error:
        return seed_box, "seed"
    binary = np.where((mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD), 255, 0).astype(np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return seed_box, "seed"

    points = [point for point in (pose.tail, pose.head) if point is not None]
    selected = []
    for contour in contours:
        if cv2.contourArea(contour) < 15.0:
            continue
        bx, by, bw, bh = cv2.boundingRect(contour)
        global_box = (ix1 + bx, iy1 + by, ix1 + bx + bw, iy1 + by + bh)
        contains_marker = any(_point_box_distance(point, global_box) <= 8.0 for point in points)
        centre = ((x1 + x2) * 0.5, (y1 + y2) * 0.5)
        contains_seed = _point_box_distance(centre, global_box) <= 6.0
        if contains_marker or contains_seed:
            selected.append(global_box)
    if not selected:
        return seed_box, "seed"

    fx1 = min(item[0] for item in selected)
    fy1 = min(item[1] for item in selected)
    fx2 = max(item[2] for item in selected)
    fy2 = max(item[3] for item in selected)
    for point in points:
        fx1, fy1 = min(fx1, point[0]), min(fy1, point[1])
        fx2, fy2 = max(fx2, point[0]), max(fy2, point[1])
    refined = _clip_box((fx1 - 4, fy1 - 4, fx2 + 4, fy2 + 4), width, height)
    seed_area = max(1.0, (x2 - x1) * (y2 - y1))
    refined_area = (refined[2] - refined[0]) * (refined[3] - refined[1])
    if not 0.22 <= refined_area / seed_area <= 1.80:
        return seed_box, "seed"
    return refined, "grabcut"


def split_for_time(seconds: float, duration: float) -> str | None:
    fraction = seconds / max(duration, 1e-6)
    if 0.55 <= fraction <= 0.64:
        return "val"
    if 0.76 <= fraction <= 0.86:
        return "test"
    if 0.52 <= fraction <= 0.67 or 0.73 <= fraction <= 0.89:
        return None
    return "train"


def yolo_line(box, width: int, height: int) -> str:
    x1, y1, x2, y2 = box
    return (
        f"0 {((x1 + x2) * 0.5 / width):.8f} {((y1 + y2) * 0.5 / height):.8f} "
        f"{((x2 - x1) / width):.8f} {((y2 - y1) / height):.8f}\n"
    )


def make_contact_sheets(items, destination: Path, page_size: int = 48) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    for page_start in range(0, len(items), page_size):
        page = items[page_start : page_start + page_size]
        thumbs = []
        for image_path, box, source, confidence in page:
            image = cv2.imread(str(image_path))
            x1, y1, x2, y2 = [int(round(value)) for value in box]
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(image, f"{source} {confidence:.2f}", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            thumbs.append(cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA))
        columns = 6
        rows = (len(thumbs) + columns - 1) // columns
        sheet = np.zeros((rows * 240, columns * 320, 3), np.uint8)
        for index, thumb in enumerate(thumbs):
            row, column = divmod(index, columns)
            sheet[row * 240 : (row + 1) * 240, column * 320 : (column + 1) * 320] = thumb
        cv2.imwrite(str(destination / f"labels_{page_start // page_size:02d}.jpg"), sheet)


def promote(root: Path) -> int:
    pending = root / "review" / "pending"
    dataset = root / "dataset"
    if not (pending / "metadata.csv").is_file():
        raise SystemExit("pending dataset not found")
    if dataset.exists():
        shutil.rmtree(dataset)
    for split in ("train", "val", "test"):
        (dataset / "images" / split).mkdir(parents=True, exist_ok=True)
        (dataset / "labels" / split).mkdir(parents=True, exist_ok=True)
    split_paths = {name: [] for name in ("train", "val", "test")}
    with (pending / "metadata.csv").open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            split = row["split"]
            name = row["name"]
            shutil.copy2(pending / "images" / name, dataset / "images" / split / name)
            label_name = Path(name).with_suffix(".txt").name
            shutil.copy2(pending / "labels" / label_name, dataset / "labels" / split / label_name)
            split_paths[split].append(f"./../images/{split}/{name}")
    (dataset / "splits").mkdir(parents=True, exist_ok=True)
    for split, paths in split_paths.items():
        (dataset / "splits" / f"{split}.txt").write_text("\n".join(paths) + "\n", encoding="utf-8")
    (dataset / "dataset.yaml").write_text(
        "train: splits/train.txt\nval: splits/val.txt\ntest: splits/test.txt\n\nnames:\n  0: fish\n",
        encoding="utf-8",
    )
    print(json.dumps({name: len(paths) for name, paths in split_paths.items()}))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", type=Path)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--stride", type=int, default=8)
    parser.add_argument("--promote", action="store_true")
    args = parser.parse_args()
    if args.promote:
        return promote(args.root)
    if args.video is None or args.model is None:
        parser.error("--video and --model are required unless --promote is used")

    pending = args.root / "review" / "pending"
    if pending.exists():
        shutil.rmtree(pending)
    images_dir = pending / "images"
    labels_dir = pending / "labels"
    images_dir.mkdir(parents=True)
    labels_dir.mkdir(parents=True)

    capture = cv2.VideoCapture(str(args.video))
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    duration = frame_count / max(fps, 1e-6)
    model = YOLO(str(args.model))
    rows = []
    review_items = []
    previous_center = None
    previous_midpoint = None
    batch_frames, batch_indices = [], []

    def process_batch():
        nonlocal previous_center, previous_midpoint
        if not batch_frames:
            return
        results = model.predict(batch_frames, conf=0.005, imgsz=640, device=0, max_det=10, verbose=False)
        for frame, frame_index, result in zip(batch_frames, batch_indices, results):
            seconds = frame_index / max(fps, 1e-6)
            split = split_for_time(seconds, duration)
            if split is None:
                continue
            pose = detect_pose_markers(frame, previous_midpoint)
            if pose.midpoint is not None:
                previous_midpoint = np.asarray(pose.midpoint)
            seed, confidence = select_seed_box(result, pose, previous_center, frame.shape)
            if seed is None:
                continue
            box, source = refine_box(frame, seed, pose)
            centre = np.asarray([(box[0] + box[2]) * 0.5, (box[1] + box[3]) * 0.5])
            if previous_center is not None and np.linalg.norm(centre - previous_center) > 145.0:
                continue
            previous_center = centre
            name = f"clean_20260831_f{frame_index:06d}.jpg"
            image_path = images_dir / name
            cv2.imwrite(str(image_path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
            (labels_dir / Path(name).with_suffix(".txt").name).write_text(
                yolo_line(box, frame.shape[1], frame.shape[0]), encoding="utf-8"
            )
            rows.append({
                "name": name,
                "frame": frame_index,
                "seconds": f"{seconds:.3f}",
                "split": split,
                "source": source,
                "seed_confidence": f"{confidence:.5f}",
                "x1": f"{box[0]:.2f}", "y1": f"{box[1]:.2f}",
                "x2": f"{box[2]:.2f}", "y2": f"{box[3]:.2f}",
            })
            review_items.append((image_path, box, source, confidence))

    index = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if index % max(1, args.stride) == 0:
            batch_frames.append(frame.copy())
            batch_indices.append(index)
            if len(batch_frames) >= 32:
                process_batch()
                batch_frames.clear()
                batch_indices.clear()
        index += 1
    process_batch()
    capture.release()

    fieldnames = list(rows[0]) if rows else []
    with (pending / "metadata.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    digest = hashlib.sha256(args.video.read_bytes()).hexdigest()
    manifest = {
        "video": str(args.video.resolve()),
        "video_sha256": digest,
        "model_seed": str(args.model.resolve()),
        "stride": args.stride,
        "frames": frame_count,
        "accepted": len(rows),
        "split_counts": {name: sum(row["split"] == name for row in rows) for name in ("train", "val", "test")},
        "segmentation_counts": {name: sum(row["source"] == name for row in rows) for name in ("grabcut", "seed")},
    }
    (pending / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    make_contact_sheets(review_items, args.root / "review" / "contact_sheets")
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
