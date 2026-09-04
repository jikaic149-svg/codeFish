"""Build and audit the two-class RoboFish dataset from a fixed-camera video.

The labels are generated without a detector model.  Red pixels identify the
``red_fish`` and dark, highly saturated green pixels identify the
``green_fish`` class.  A temporal nearest-neighbour gate removes
stationary pool hardware and ripple components.  Generated labels stay in a
review directory until the explicit ``promote`` command is run.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
from dataclasses import dataclass, replace
from pathlib import Path

import cv2
import numpy as np


CLASS_NAMES = {0: "red_fish", 1: "green_fish"}
SCENE_NAME = "auv_record_20260831_170013"
BACKGROUND_SAMPLES = 64
CONTACT_SHEET_SIZE = 48


@dataclass(frozen=True)
class Detection:
    class_id: int
    bbox: tuple[int, int, int, int]
    colour_pixels: int
    median_value: float
    median_background_drop: float
    centre: tuple[float, float]
    score: float


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def split_for_seconds(seconds: float) -> str | None:
    """Use separated time windows so adjacent frames cannot cross splits."""
    if 0.0 <= seconds < 50.0 or 90.0 <= seconds:
        return "train"
    if 55.0 <= seconds < 67.0:
        return "val"
    if 72.0 <= seconds < 84.0:
        return "test"
    return None


def read_frame(capture: cv2.VideoCapture, index: int) -> np.ndarray:
    capture.set(cv2.CAP_PROP_POS_FRAMES, int(index))
    ok, frame = capture.read()
    if not ok:
        raise RuntimeError(f"cannot decode frame {index}")
    return frame


def build_background(video: Path, frame_count: int) -> np.ndarray:
    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open video: {video}")
    frames = []
    for index in np.linspace(0, frame_count - 1, BACKGROUND_SAMPLES, dtype=int):
        frames.append(read_frame(capture, int(index)))
    capture.release()
    return np.median(np.stack(frames, axis=0), axis=0).astype(np.uint8)


def pool_mask(height: int, width: int) -> np.ndarray:
    mask = np.zeros((height, width), np.uint8)
    polygon = np.asarray(
        [
            (round(width * 0.02), round(height * 0.22)),
            (round(width * 0.94), round(height * 0.14)),
            (round(width * 0.997), round(height * 0.84)),
            (round(width * 0.02), round(height * 0.91)),
        ],
        dtype=np.int32,
    )
    cv2.fillConvexPoly(mask, polygon, 255)
    return mask


def colour_masks(frame: np.ndarray, background: np.ndarray, water: np.ndarray):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.int16)
    background_gray = cv2.cvtColor(background, cv2.COLOR_BGR2GRAY).astype(np.int16)
    background_drop = background_gray - gray

    red = (
        (((hue <= 14) | (hue >= 154)) & (saturation >= 70) & (value >= 40))
        & (water > 0)
    ).astype(np.uint8) * 255
    blue = (
        ((hue >= 86) & (hue <= 114))
        & (saturation >= 160)
        & (value <= 130)
        & (background_drop >= 8)
        & (water > 0)
    ).astype(np.uint8) * 255

    open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    red = cv2.morphologyEx(red, cv2.MORPH_OPEN, open_kernel)
    red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, close_kernel)
    blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, open_kernel)
    blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, close_kernel)
    return red, blue, value, background_drop


def _candidate_components(
    mask: np.ndarray,
    class_id: int,
    value: np.ndarray,
    background_drop: np.ndarray,
    previous: tuple[float, float] | None,
    missed_frames: int,
) -> list[Detection]:
    height, width = mask.shape
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    minimum_area = 7 if class_id == 0 else 10
    maximum_area = 1300 if class_id == 0 else 1600
    padding = 7 if class_id == 0 else 6
    candidates: list[Detection] = []

    for component in range(1, count):
        area = int(stats[component, cv2.CC_STAT_AREA])
        x = int(stats[component, cv2.CC_STAT_LEFT])
        y = int(stats[component, cv2.CC_STAT_TOP])
        component_width = int(stats[component, cv2.CC_STAT_WIDTH])
        component_height = int(stats[component, cv2.CC_STAT_HEIGHT])
        if not minimum_area <= area <= maximum_area:
            continue
        if min(component_width, component_height) < 2 or max(component_width, component_height) > 90:
            continue
        raw_aspect = max(component_width, component_height) / max(1, min(component_width, component_height))
        if raw_aspect > 8.0:
            continue

        centre = (float(centroids[component, 0]), float(centroids[component, 1]))
        distance = 0.0
        if previous is not None and missed_frames < 40:
            distance = math.dist(centre, previous)
            maximum_distance = min(140.0, 22.0 + 4.0 * missed_frames)
            if distance > maximum_distance:
                continue

        component_pixels = labels == component
        median_value = float(np.median(value[component_pixels]))
        median_drop = float(np.median(background_drop[component_pixels]))
        compactness = area / max(1.0, float(component_width * component_height))
        score = area + 30.0 * compactness + 1.5 * median_drop - 4.0 * distance
        if class_id == 0:
            score += 0.25 * median_value

        x1 = max(0, x - padding)
        y1 = max(0, y - padding)
        x2 = min(width, x + component_width + padding)
        y2 = min(height, y + component_height + padding)
        box_width, box_height = x2 - x1, y2 - y1
        if min(box_width, box_height) < 14 or max(box_width, box_height) > 110:
            continue
        if box_width * box_height < 250:
            continue
        candidates.append(
            Detection(
                class_id=class_id,
                bbox=(x1, y1, x2, y2),
                colour_pixels=area,
                median_value=median_value,
                median_background_drop=median_drop,
                centre=centre,
                score=score,
            )
        )
    return candidates


def refine_detection(
    frame: np.ndarray,
    colour_mask: np.ndarray,
    background_drop: np.ndarray,
    detection: Detection,
) -> Detection:
    """Grow a small colour seed into a complete foreground fish box."""
    height, width = colour_mask.shape
    centre_x, centre_y = (int(round(value)) for value in detection.centre)
    radius = 54
    x1, y1 = max(0, centre_x - radius), max(0, centre_y - radius)
    x2, y2 = min(width, centre_x + radius + 1), min(height, centre_y + radius + 1)
    crop = frame[y1:y2, x1:x2]
    if min(crop.shape[:2]) < 20:
        return detection

    local_colour = colour_mask[y1:y2, x1:x2]
    component_count, component_labels, _, component_centres = cv2.connectedComponentsWithStats(local_colour, 8)
    if component_count <= 1:
        return detection
    target = min(
        range(1, component_count),
        key=lambda index: math.dist(
            (float(component_centres[index, 0] + x1), float(component_centres[index, 1] + y1)),
            detection.centre,
        ),
    )
    seed = component_labels == target
    if int(seed.sum()) < 4:
        return detection

    grabcut_mask = np.full(seed.shape, cv2.GC_PR_BGD, np.uint8)
    probable_foreground = background_drop[y1:y2, x1:x2] >= 6
    grabcut_mask[probable_foreground] = cv2.GC_PR_FGD
    grabcut_mask[seed] = cv2.GC_FGD
    grabcut_mask[:3, :] = cv2.GC_BGD
    grabcut_mask[-3:, :] = cv2.GC_BGD
    grabcut_mask[:, :3] = cv2.GC_BGD
    grabcut_mask[:, -3:] = cv2.GC_BGD
    background_model = np.zeros((1, 65), np.float64)
    foreground_model = np.zeros((1, 65), np.float64)
    try:
        cv2.grabCut(
            crop,
            grabcut_mask,
            None,
            background_model,
            foreground_model,
            3,
            cv2.GC_INIT_WITH_MASK,
        )
    except cv2.error:
        return detection

    foreground = np.where(
        (grabcut_mask == cv2.GC_FGD) | (grabcut_mask == cv2.GC_PR_FGD), 255, 0
    ).astype(np.uint8)
    foreground = cv2.morphologyEx(
        foreground,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
    )
    found, found_labels, found_stats, _ = cv2.connectedComponentsWithStats(foreground, 8)
    best_component = None
    best_overlap = 0
    for component in range(1, found):
        overlap = int(np.count_nonzero((found_labels == component) & seed))
        area = int(found_stats[component, cv2.CC_STAT_AREA])
        if overlap > best_overlap and 30 <= area <= 3500:
            best_overlap = overlap
            best_component = component
    if best_component is None:
        return detection

    bx = int(found_stats[best_component, cv2.CC_STAT_LEFT]) + x1
    by = int(found_stats[best_component, cv2.CC_STAT_TOP]) + y1
    bw = int(found_stats[best_component, cv2.CC_STAT_WIDTH])
    bh = int(found_stats[best_component, cv2.CC_STAT_HEIGHT])
    padding = 3
    refined = (
        max(0, bx - padding),
        max(0, by - padding),
        min(width, bx + bw + padding),
        min(height, by + bh + padding),
    )
    refined_width = refined[2] - refined[0]
    refined_height = refined[3] - refined[1]
    if min(refined_width, refined_height) < 15 or max(refined_width, refined_height) > 110:
        return detection
    if not 0.55 <= (refined_width * refined_height) / max(1, (detection.bbox[2] - detection.bbox[0]) * (detection.bbox[3] - detection.bbox[1])) <= 8.0:
        return detection
    return replace(detection, bbox=refined)


def detect_colours(
    frame: np.ndarray,
    background: np.ndarray,
    water: np.ndarray,
    previous: dict[int, tuple[float, float] | None],
    missed: dict[int, int],
) -> dict[int, Detection | None]:
    red, blue, value, background_drop = colour_masks(frame, background, water)
    detected: dict[int, Detection | None] = {0: None, 1: None}

    red_candidates = _candidate_components(
        red, 0, value, background_drop, previous[0], missed[0]
    )
    red_selected = max(red_candidates, key=lambda item: item.score, default=None)
    detected[0] = red_selected
    if red_selected is None:
        missed[0] += 1
    else:
        previous[0] = red_selected.centre
        missed[0] = 0

    blue_candidates = _candidate_components(
        blue, 1, value, background_drop, previous[1], missed[1]
    )
    if red_selected is not None:
        blue_candidates = [
            item for item in blue_candidates
            if math.dist(item.centre, red_selected.centre) >= 34.0
            and box_iou(item.bbox, red_selected.bbox) <= 0.05
        ]
    blue_selected = max(blue_candidates, key=lambda item: item.score, default=None)
    detected[1] = blue_selected
    if blue_selected is None:
        missed[1] += 1
    else:
        previous[1] = blue_selected.centre
        missed[1] = 0
    return detected


def box_iou(first: tuple[int, int, int, int], second: tuple[int, int, int, int]) -> float:
    x1 = max(first[0], second[0])
    y1 = max(first[1], second[1])
    x2 = min(first[2], second[2])
    y2 = min(first[3], second[3])
    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    first_area = max(0, first[2] - first[0]) * max(0, first[3] - first[1])
    second_area = max(0, second[2] - second[0]) * max(0, second[3] - second[1])
    return intersection / max(1, first_area + second_area - intersection)


def yolo_line(detection: Detection, width: int, height: int) -> str:
    x1, y1, x2, y2 = detection.bbox
    return (
        f"{detection.class_id} {((x1 + x2) * 0.5 / width):.8f} "
        f"{((y1 + y2) * 0.5 / height):.8f} "
        f"{((x2 - x1) / width):.8f} {((y2 - y1) / height):.8f}"
    )


def draw_review(frame: np.ndarray, detections: dict[int, Detection], title: str) -> np.ndarray:
    preview = frame.copy()
    colours = {0: (0, 0, 255), 1: (0, 255, 80)}
    for class_id in sorted(detections):
        item = detections[class_id]
        x1, y1, x2, y2 = item.bbox
        colour = colours[class_id]
        cv2.rectangle(preview, (x1, y1), (x2, y2), colour, 2)
        cv2.putText(
            preview,
            f"{CLASS_NAMES[class_id]} c={item.colour_pixels}",
            (x1, max(18, y1 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            colour,
            1,
            cv2.LINE_AA,
        )
    cv2.putText(preview, title, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (0, 255, 255), 2, cv2.LINE_AA)
    return preview


def make_contact_sheets(items: list[tuple[Path, dict[int, Detection], str]], destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=False)
    thumb_width, thumb_height, columns = 320, 240, 6
    for page_start in range(0, len(items), CONTACT_SHEET_SIZE):
        page = items[page_start : page_start + CONTACT_SHEET_SIZE]
        rows = (len(page) + columns - 1) // columns
        sheet = np.zeros((rows * thumb_height, columns * thumb_width, 3), np.uint8)
        for offset, (image_path, detections, title) in enumerate(page):
            image = cv2.imread(str(image_path))
            review = draw_review(image, detections, title)
            thumb = cv2.resize(review, (thumb_width, thumb_height), interpolation=cv2.INTER_AREA)
            row, column = divmod(offset, columns)
            sheet[row * thumb_height : (row + 1) * thumb_height, column * thumb_width : (column + 1) * thumb_width] = thumb
        output = destination / f"labels_{page_start // CONTACT_SHEET_SIZE:02d}.jpg"
        if not cv2.imwrite(str(output), sheet, [int(cv2.IMWRITE_JPEG_QUALITY), 92]):
            raise RuntimeError(f"cannot save contact sheet: {output}")


def generate(video: Path, root: Path, stride: int, pending_name: str) -> int:
    pending = root / "review" / pending_name
    if pending.exists():
        raise SystemExit(f"review candidate already exists: {pending}")
    images_dir = pending / "images" / SCENE_NAME
    labels_dir = pending / "labels" / SCENE_NAME
    images_dir.mkdir(parents=True)
    labels_dir.mkdir(parents=True)

    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        raise SystemExit(f"cannot open video: {video}")
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if frame_count <= 0 or fps <= 0 or width <= 0 or height <= 0:
        raise SystemExit("invalid video metadata")
    capture.release()

    background = build_background(video, frame_count)
    background_path = pending / "background_median.jpg"
    if not cv2.imwrite(str(background_path), background, [int(cv2.IMWRITE_JPEG_QUALITY), 95]):
        raise RuntimeError(f"cannot save background: {background_path}")
    water = pool_mask(height, width)

    capture = cv2.VideoCapture(str(video))
    previous: dict[int, tuple[float, float] | None] = {0: None, 1: None}
    missed = {0: 0, 1: 0}
    rows: list[dict[str, object]] = []
    review_items: list[tuple[Path, dict[int, Detection], str]] = []
    rejected = {"split_gap": 0, "stride": 0, "missing_red": 0, "missing_green": 0}

    frame_index = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        detections = detect_colours(frame, background, water, previous, missed)
        seconds = frame_index / fps
        split = split_for_seconds(seconds)
        if frame_index % max(1, stride) != 0:
            rejected["stride"] += 1
        elif split is None:
            rejected["split_gap"] += 1
        elif detections[0] is None:
            rejected["missing_red"] += 1
        elif detections[1] is None:
            rejected["missing_green"] += 1
        else:
            red_mask, blue_mask, _, background_drop = colour_masks(frame, background, water)
            selected = {
                0: refine_detection(frame, red_mask, background_drop, detections[0]),
                1: refine_detection(frame, blue_mask, background_drop, detections[1]),
            }
            if box_iou(selected[0].bbox, selected[1].bbox) > 0.10:
                rejected["missing_green"] += 1
                frame_index += 1
                continue
            name = f"{SCENE_NAME}_f{frame_index:06d}.jpg"
            image_path = images_dir / name
            label_path = labels_dir / Path(name).with_suffix(".txt").name
            if not cv2.imwrite(str(image_path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95]):
                raise RuntimeError(f"cannot save image: {image_path}")
            label_path.write_text(
                "\n".join(yolo_line(selected[class_id], width, height) for class_id in sorted(selected)) + "\n",
                encoding="utf-8",
            )
            row: dict[str, object] = {
                "name": name,
                "frame": frame_index,
                "seconds": f"{seconds:.3f}",
                "split": split,
            }
            for class_id, prefix in ((0, "red"), (1, "green")):
                item = selected[class_id]
                row[f"{prefix}_bbox"] = ",".join(str(value) for value in item.bbox)
                row[f"{prefix}_pixels"] = item.colour_pixels
                row[f"{prefix}_value"] = f"{item.median_value:.2f}"
                row[f"{prefix}_background_drop"] = f"{item.median_background_drop:.2f}"
            rows.append(row)
            review_items.append((image_path, selected, f"f{frame_index} {seconds:.1f}s {split}"))
        frame_index += 1
    capture.release()

    if not rows:
        raise SystemExit("no complete two-fish frames accepted")
    metadata_path = pending / "metadata.csv"
    with metadata_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    make_contact_sheets(review_items, pending / "contact_sheets")

    labels_digest = hashlib.sha256()
    for path in sorted(labels_dir.glob("*.txt")):
        labels_digest.update(path.relative_to(pending).as_posix().encode("utf-8"))
        labels_digest.update(path.read_bytes())
    manifest = {
        "source": {
            "video": str(video.resolve()),
            "video_sha256": file_sha256(video),
            "frames": frame_count,
            "fps": fps,
            "width": width,
            "height": height,
            "duration_seconds": frame_count / fps,
        },
        "labelling": {
            "method": "HSV colour masks plus background-darkness and temporal continuity",
            "detector_model_used": False,
            "class_names": CLASS_NAMES,
            "stride": stride,
            "background_samples": BACKGROUND_SAMPLES,
            "split_windows_seconds": {
                "train": [[0.0, 50.0], [90.0, frame_count / fps]],
                "val": [[55.0, 67.0]],
                "test": [[72.0, 84.0]],
            },
        },
        "candidate": {
            "accepted_images": len(rows),
            "instances": len(rows) * 2,
            "split_counts": {name: sum(row["split"] == name for row in rows) for name in ("train", "val", "test")},
            "rejected": rejected,
            "labels_sha256": labels_digest.hexdigest(),
            "review_status": "pending_visual_review",
        },
    }
    (pending / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    return 0


def promote(root: Path, pending_name: str, dataset_name: str) -> int:
    pending = root / "review" / pending_name
    dataset = root / dataset_name
    if dataset.exists():
        raise SystemExit(f"formal dataset already exists: {dataset}")
    manifest_path = pending / "manifest.json"
    metadata_path = pending / "metadata.csv"
    if not manifest_path.is_file() or not metadata_path.is_file():
        raise SystemExit(f"incomplete review candidate: {pending}")

    images_destination = dataset / "images" / SCENE_NAME
    labels_destination = dataset / "labels" / SCENE_NAME
    splits_destination = dataset / "splits"
    images_destination.mkdir(parents=True)
    labels_destination.mkdir(parents=True)
    splits_destination.mkdir(parents=True)
    split_entries = {"train": [], "val": [], "test": []}

    with metadata_path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            name = row["name"]
            split = row["split"]
            source_image = pending / "images" / SCENE_NAME / name
            source_label = pending / "labels" / SCENE_NAME / Path(name).with_suffix(".txt").name
            shutil.copy2(source_image, images_destination / name)
            shutil.copy2(source_label, labels_destination / source_label.name)
            split_entries[split].append(f"./../images/{SCENE_NAME}/{name}")

    for split, entries in split_entries.items():
        (splits_destination / f"{split}.txt").write_text("\n".join(entries) + "\n", encoding="utf-8")
    (dataset / "dataset.yaml").write_text(
        "train: splits/train.txt\n"
        "val: splits/val.txt\n"
        "test: splits/test.txt\n\n"
        "names:\n"
        "  0: red_fish\n"
        "  1: green_fish\n",
        encoding="utf-8",
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["candidate"]["review_status"] = "visually_reviewed_and_promoted"
    manifest["formal_dataset"] = str(dataset.resolve())
    (dataset / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({name: len(entries) for name, entries in split_entries.items()}))
    return 0


def audit(root: Path, dataset_name: str) -> int:
    dataset = root / dataset_name
    problems: list[str] = []
    counts = {"train": 0, "val": 0, "test": 0}
    instances = {0: 0, 1: 0}
    box_widths: list[float] = []
    box_heights: list[float] = []
    exact_hashes: dict[str, str] = {}
    split_frames: dict[str, list[int]] = {"train": [], "val": [], "test": []}

    for split in counts:
        split_file = dataset / "splits" / f"{split}.txt"
        if not split_file.is_file():
            problems.append(f"missing split file: {split_file}")
            continue
        for relative in split_file.read_text(encoding="utf-8").splitlines():
            if not relative.strip():
                continue
            image_path = (split_file.parent / relative).resolve()
            label_path = dataset / "labels" / SCENE_NAME / (image_path.stem + ".txt")
            counts[split] += 1
            try:
                split_frames[split].append(int(image_path.stem.rsplit("_f", 1)[1]))
            except (IndexError, ValueError):
                problems.append(f"invalid frame name: {image_path.name}")
            image = cv2.imread(str(image_path))
            if image is None:
                problems.append(f"cannot decode image: {image_path}")
                continue
            if not label_path.is_file():
                problems.append(f"missing label: {label_path}")
                continue
            digest = file_sha256(image_path)
            if digest in exact_hashes:
                problems.append(f"exact duplicate: {image_path} == {exact_hashes[digest]}")
            exact_hashes[digest] = str(image_path)

            seen_classes = []
            for line_number, line in enumerate(label_path.read_text(encoding="utf-8").splitlines(), 1):
                parts = line.split()
                if len(parts) != 5:
                    problems.append(f"invalid field count: {label_path}:{line_number}")
                    continue
                try:
                    class_id = int(parts[0])
                    x, y, width, height = (float(item) for item in parts[1:])
                except ValueError:
                    problems.append(f"invalid number: {label_path}:{line_number}")
                    continue
                if class_id not in CLASS_NAMES:
                    problems.append(f"class out of range: {label_path}:{line_number}")
                    continue
                if not all(math.isfinite(item) for item in (x, y, width, height)):
                    problems.append(f"non-finite coordinate: {label_path}:{line_number}")
                if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0 and 0.0 < width <= 1.0 and 0.0 < height <= 1.0):
                    problems.append(f"coordinate out of range: {label_path}:{line_number}")
                if x - width / 2 < -1e-6 or x + width / 2 > 1.0 + 1e-6:
                    problems.append(f"horizontal box outside image: {label_path}:{line_number}")
                if y - height / 2 < -1e-6 or y + height / 2 > 1.0 + 1e-6:
                    problems.append(f"vertical box outside image: {label_path}:{line_number}")
                instances[class_id] += 1
                seen_classes.append(class_id)
                box_widths.append(width)
                box_heights.append(height)
            if sorted(seen_classes) != [0, 1]:
                problems.append(f"expected one box per class: {label_path}")

    memberships: dict[int, str] = {}
    for split, frames in split_frames.items():
        for frame in frames:
            if frame in memberships:
                problems.append(f"frame appears in two splits: {frame}")
            memberships[frame] = split
    cross_split_frame_gap = None
    for left_name, right_name in (("train", "val"), ("val", "test"), ("test", "train")):
        left, right = split_frames[left_name], split_frames[right_name]
        if left and right:
            gap = min(abs(a - b) for a in left for b in right)
            cross_split_frame_gap = gap if cross_split_frame_gap is None else min(cross_split_frame_gap, gap)

    report = {
        "dataset": str(dataset.resolve()),
        "images": sum(counts.values()),
        "split_counts": counts,
        "instances": {CLASS_NAMES[key]: value for key, value in instances.items()},
        "exact_duplicates": 0 if not any("exact duplicate" in item for item in problems) else None,
        "minimum_cross_split_frame_gap": cross_split_frame_gap,
        "box_width_normalized": {
            "min": min(box_widths, default=None),
            "median": float(np.median(box_widths)) if box_widths else None,
            "max": max(box_widths, default=None),
        },
        "box_height_normalized": {
            "min": min(box_heights, default=None),
            "median": float(np.median(box_heights)) if box_heights else None,
            "max": max(box_heights, default=None),
        },
        "problems": problems,
        "passed": not problems,
    }
    (dataset / "audit.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if not problems else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    generate_parser = subparsers.add_parser("generate")
    generate_parser.add_argument("--video", type=Path, required=True)
    generate_parser.add_argument("--root", type=Path, required=True)
    generate_parser.add_argument("--stride", type=int, default=8)
    generate_parser.add_argument("--pending-name", default="pending_color_v1")

    promote_parser = subparsers.add_parser("promote")
    promote_parser.add_argument("--root", type=Path, required=True)
    promote_parser.add_argument("--pending-name", default="pending_color_v1")
    promote_parser.add_argument("--dataset-name", default="dataset_v1")

    audit_parser = subparsers.add_parser("audit")
    audit_parser.add_argument("--root", type=Path, required=True)
    audit_parser.add_argument("--dataset-name", default="dataset_v1")

    args = parser.parse_args()
    if args.command == "generate":
        return generate(args.video, args.root, max(1, args.stride), args.pending_name)
    if args.command == "promote":
        return promote(args.root, args.pending_name, args.dataset_name)
    return audit(args.root, args.dataset_name)


if __name__ == "__main__":
    raise SystemExit(main())
