"""Offline white nose-cap preview. Run with a pool video as the sole argument."""
import argparse
import csv
from collections import deque
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'vision'))
from white_head_point import WhiteHeadPoint
from yolo_tracker import YoloBodyTracker


GREEN, CYAN, RED, TEXT = (110, 255, 74), (236, 192, 40), (105, 104, 255), (225, 230, 235)


def label(image, text, x, y, colour=TEXT, size=.58):
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, colour, 1, cv2.LINE_AA)


def zoom_pair(frame, box, point):
    w, h = 320, 200
    if box is None:
        raw = np.zeros((h, w, 3), np.uint8)
        label(raw, 'No fish ROI', 15, 105)
        return raw, raw.copy()
    x, y, r, b = box
    scale = min(w / max(r-x, 1), h / max(b-y, 1)) * .80
    cx, cy = (x+r)/2, (y+b)/2
    matrix = np.float32([[scale, 0, w/2-cx*scale], [0, scale, h/2-cy*scale]])
    raw = cv2.warpAffine(frame, matrix, (w, h), borderMode=cv2.BORDER_REPLICATE)
    marked = raw.copy()
    if point is not None:
        u = tuple(np.rint(matrix @ [*point, 1]).astype(int))
        cv2.circle(marked, u, 7, GREEN, 1, cv2.LINE_AA)
    return raw, marked


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video', type=Path)
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts'/'white-head-preview')
    args = parser.parse_args()
    cap = cv2.VideoCapture(str(args.video))
    fps = cap.get(cv2.CAP_PROP_FPS)
    if not cap.isOpened() or not fps > 0:
        raise SystemExit(f'Cannot open video: {args.video}')
    body = YoloBodyTracker(ROOT/'vision'/'fish_body_yolo'/'best.pt')
    detector = WhiteHeadPoint()
    args.output.mkdir(exist_ok=True, parents=True)
    writer = cv2.VideoWriter(str(args.output/'preview.mp4'), cv2.VideoWriter_fourcc(*'mp4v'),
                             fps, (1152, 540))
    if not writer.isOpened():
        raise RuntimeError('Cannot open output video writer')
    count = hits = missing = longest = 0
    trace = deque(maxlen=max(2, round(.65*fps)))
    last_id = None
    samples = []
    sample_frames = {round(t*fps) for t in (4, 8, 20, 38, 55)}
    worst = None
    with (args.output/'head-points.csv').open('w', newline='') as out:
        rows = csv.writer(out)
        rows.writerow(('frame', 'video_time_s', 'body_source', 'white_head_valid', 'x_px', 'y_px'))
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            t = count/fps
            result = body.update(frame, t)
            box = result.metrics.get('fish_bbox') if result.valid else None
            track_id = result.metrics.get('track_id')
            if track_id != last_id:
                detector = WhiteHeadPoint()
                trace.clear()
                last_id = track_id
            point = detector.measure(frame, box, t)
            raw, marked = zoom_pair(frame, box, point)
            rows.writerow((count, f'{t:.4f}', result.metrics['tracking_mode'], int(point is not None),
                           *(('', '') if point is None else (f'{p:.2f}' for p in point))))
            status = 'LOST' if point is None else 'MEASURED'
            if count in sample_frames or (point is None and missing+1 > longest):
                card = np.full((235, 640, 3), (32, 28, 24), np.uint8)
                label(card, f'{t:05.2f} s    RAW  |  MARKED    {status}', 12, 24,
                      RED if point is None else TEXT)
                card[35:] = np.hstack((raw, marked))
                if count in sample_frames:
                    samples.append(card)
                if point is None:
                    worst = card
            if point is None:
                trace.clear()
                missing += 1
                longest = max(missing, longest)
            else:
                trace.append(point)
                hits += 1
                missing = 0
            canvas = np.full((540, 1152, 3), (32, 28, 24), np.uint8)
            scale = min(816/frame.shape[1], 453/frame.shape[0])
            full = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            if trace:
                line = np.rint(np.array(trace)*scale).astype('int32')
                cv2.polylines(full, [line], False, CYAN, 1, cv2.LINE_AA)
                cv2.circle(full, tuple(line[-1]), 5, GREEN, 2, cv2.LINE_AA)
            canvas[48:48+full.shape[0], 8:8+full.shape[1]] = full
            canvas[48:248, 824:1144] = raw
            canvas[302:502, 824:1144] = marked
            label(canvas, f'WHITE HEAD   /   {t:05.2f} s', 12, 31, size=.75)
            label(canvas, '01    RAW CROP', 824, 31)
            label(canvas, '02    '+status, 824, 285, RED if point is None else GREEN)
            label(canvas, 'Green ring: measured nose cap       Cyan: last 0.65 s       No fill across gaps',
                  12, 527, size=.52)
            writer.write(canvas)
            count += 1
    cap.release()
    writer.release()
    if not count:
        raise RuntimeError('No frames decoded')
    if worst is not None:
        samples.append(worst)
    if samples:
        if len(samples) % 2:
            samples.append(np.zeros_like(samples[0]))
        sheet = np.vstack([np.hstack(samples[i:i+2]) for i in range(0, len(samples), 2)])
        cv2.imencode('.jpg', sheet, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tofile(args.output/'frames.jpg')
    print(f'{hits}/{count} measured ({hits/count:.2%}); longest gap: {longest} frames ({longest/fps:.3f} s)')


if __name__ == '__main__':
    main()
