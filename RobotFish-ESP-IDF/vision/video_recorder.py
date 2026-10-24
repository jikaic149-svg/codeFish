"""Bounded video recording with explicit encoder and finalized-file status."""
import csv
import json
import math
from pathlib import Path
import queue
import threading
import time

import cv2

CSV_FIELDS = ['timestamp', 'pixel_u', 'pixel_v', 'physical_x', 'physical_y', 'yaw_deg', 'raw_yaw_deg', 'yaw_source', 'yaw_usable', 'detector_raw_yaw_deg', 'color_raw_yaw_deg', 'color_head_source', 'head_color_confidence', 'tail_color_confidence', 'head_u', 'head_v', 'tail_u', 'tail_v', 'pose_valid', 'pose_rejection_reason', 'detection_bbox', 'white_bbox', 'white_area', 'orange_bbox', 'orange_area', 'head_tail_distance_px', 'visual_yaw_deg', 'fused_yaw_deg', 'motion_prior_deg', 'motion_gate_applied', 'white_dynamic_value_min', 'white_tail_distance_gate_px', 'white_rejected_components', 'detector_head_in_white', 'detector_head_distance_to_white_px']

_latest = None


def recording_status():
    return _latest.status() if _latest else {'active': False, 'saving': False, 'saved': False, 'frames': 0, 'error': ''}


class AsyncVideoRecorder:
    def __init__(self, path, fourcc, fps, frame_size, csv_path):
        global _latest
        self.path = str(Path(path).resolve())
        self.csv_path = str(Path(csv_path).resolve())
        self.width, self.height = (int(value) // 2 * 2 for value in frame_size)
        self.fps = float(fps)
        self.guard = threading.Lock()
        self.queue = queue.Queue(maxsize=8)
        self.stopped = self.closed = False
        self.frames = self.dropped_frames = 0
        self.error = ''
        self.started = time.monotonic()
        self.finished = None
        self.video_out = self.csv_file = self.thread = None
        _latest = self
        try:
            if self.width < 2 or self.height < 2 or not math.isfinite(self.fps) or not 0 < self.fps <= 240:
                raise ValueError('Invalid recording frame size or FPS')
            Path(self.path).parent.mkdir(parents=True, exist_ok=True)
            self.video_out = cv2.VideoWriter(self.path, cv2.CAP_FFMPEG, fourcc, self.fps, (self.width, self.height))
            if not self.video_out.isOpened():
                raise RuntimeError('Video encoder could not open the output file')
            self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8-sig')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(CSV_FIELDS)
        except Exception as error:
            self.error = str(error)
            self.stopped = self.closed = True
            self.finished = time.monotonic()
            if self.video_out is not None: self.video_out.release()
            if self.csv_file is not None: self.csv_file.close()
            self._publish()
            raise RuntimeError(self.error) from error
        self.thread = threading.Thread(target=self._writer_loop, daemon=True)
        self.thread.start()
        self._publish()

    def status(self):
        with self.guard:
            return {'active': not self.stopped and not self.error, 'saving': self.stopped and not self.closed,
                'saved': self.closed and not self.error and self.frames > 0,
                'path': self.path, 'file': Path(self.path).name, 'csv_path': self.csv_path,
                'fps': self.fps, 'width': self.width, 'height': self.height,
                'frames': self.frames, 'dropped_frames': self.dropped_frames,
                'elapsed_s': round((self.finished or time.monotonic()) - self.started, 1), 'error': self.error}

    def _publish(self):
        print('ROBOFISH_VISION_RECORDING ' + json.dumps(self.status(), ensure_ascii=False), flush=True)

    def submit(self, frame, csv_row):
        if self.stopped: return False
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            with self.guard:
                self.error = 'Recording requires a BGR image'
                self.stopped = True
            return False
        if frame.shape[:2] != (self.height, self.width):
            frame = cv2.resize(frame, (self.width, self.height))
        try:
            self.queue.put_nowait((frame.copy(), tuple(csv_row) if csv_row is not None else None))
            return True
        except queue.Full:
            with self.guard: self.dropped_frames += 1
            return False

    def _writer_loop(self):
        try:
            while not (self.stopped and self.queue.empty()):
                try: frame, row = self.queue.get(timeout=.05)
                except queue.Empty: continue
                self.video_out.write(frame)
                if row is not None: self.csv_writer.writerow(row)
                with self.guard: self.frames += 1
        except Exception as error:
            with self.guard:
                self.error = str(error)
                self.stopped = True
        finally:
            self.video_out.release()
            self.csv_file.close()
            with self.guard:
                self.closed = True
                self.finished = time.monotonic()
                if not self.frames and not self.error: self.error = 'No video frames were recorded'
            self._publish()

    def close(self):
        self.stopped = True
        if self.thread and self.thread.is_alive(): self.thread.join(timeout=5)
        if self.thread and self.thread.is_alive():
            with self.guard: self.error = 'Timed out while saving the video'
            self._publish()
            raise RuntimeError(self.error)
