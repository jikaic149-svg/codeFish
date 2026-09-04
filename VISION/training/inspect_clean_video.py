"""Create a contact sheet from an unannotated RoboFish recording."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("video", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--samples", type=int, default=20)
    args = parser.parse_args()

    capture = cv2.VideoCapture(str(args.video))
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    if not capture.isOpened() or frame_count <= 0:
        raise SystemExit(f"cannot open video: {args.video}")

    indices = np.linspace(0, frame_count - 1, max(2, args.samples), dtype=int)
    previews: list[np.ndarray] = []
    for index in indices:
        capture.set(cv2.CAP_PROP_POS_FRAMES, int(index))
        ok, frame = capture.read()
        if not ok:
            continue
        preview = frame.copy()
        cv2.putText(
            preview,
            f"frame {index}  t={index / max(fps, 1e-6):.1f}s",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        previews.append(preview)
    capture.release()

    if not previews:
        raise SystemExit("no frames decoded")
    thumb_width = 480
    thumb_height = int(round(thumb_width * previews[0].shape[0] / previews[0].shape[1]))
    columns = 4
    rows = (len(previews) + columns - 1) // columns
    sheet = np.zeros((rows * thumb_height, columns * thumb_width, 3), np.uint8)
    for item, preview in enumerate(previews):
        thumb = cv2.resize(preview, (thumb_width, thumb_height), interpolation=cv2.INTER_AREA)
        row, column = divmod(item, columns)
        sheet[
            row * thumb_height : (row + 1) * thumb_height,
            column * thumb_width : (column + 1) * thumb_width,
        ] = thumb
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), sheet):
        raise SystemExit(f"cannot save {args.output}")
    print(args.output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
