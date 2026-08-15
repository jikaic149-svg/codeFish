"""异步 YOLO 身份检测。

本模块刻意不在顶层导入 ultralytics/torch。模型和 CUDA 在后台线程中
初始化，使相机窗口可以先显示；帧输入采用单槽 latest-value 语义，避免
模型加载或推理期间积压旧画面。
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional


def _default_model_factory(model_path: str):
    # 这条重型导入必须留在检测线程内，不能移回模块顶层。
    from ultralytics import YOLO

    return YOLO(model_path)


class FishDetector:
    """后台加载并运行低频 YOLO 检测，只保留最新一帧。

    OpenCV 色块负责高频连续跟踪，因此这里使用纯 ``predict`` 做全局
    重捕。低频输入不再交给依赖连续帧的 BoT-SORT，避免跟踪 ID 门槛把
    本来已经检测到的低置信度鱼框再次过滤掉。
    """

    def __init__(
        self,
        model_path: str,
        conf: float = 0.5,
        imgsz: int = 640,
        device=0,
        model_factory: Optional[Callable[[str], object]] = None,
    ):
        self.model_path = model_path
        self.conf = conf
        self.imgsz = imgsz
        self.device = device
        self._model_factory = model_factory or _default_model_factory

        self._latest_frame = None
        self._latest_frame_lock = threading.Lock()
        self._result_lock = threading.Lock()
        self._result = {
            "pixel": None,
            "bbox": None,
            "confidence": 0.0,
            "track_id": None,
            "frame_time": 0.0,
            "infer_fps": 0.0,
        }
        self._status_lock = threading.Lock()
        self._status = {
            "loading": False,
            "ready": False,
            "error": None,
            "last_inference_error": None,
            "load_seconds": None,
        }

        self._fps_smooth = 0.0
        self._last_infer_t = time.perf_counter()
        self._stop_event = threading.Event()
        self.thread = None

    def start(self):
        if self.thread is not None and self.thread.is_alive():
            return self
        self._stop_event.clear()
        with self._status_lock:
            self._status.update(
                loading=True,
                ready=False,
                error=None,
                last_inference_error=None,
                load_seconds=None,
            )
        self.thread = threading.Thread(
            target=self._loop,
            name="FishDetector",
            daemon=True,
        )
        self.thread.start()
        print(
            "[YOLO] 正在后台加载，视觉窗口可先使用"
            f"（设备: {self.device}, 输入尺寸: {self.imgsz}）"
        )
        return self

    def submit_frame(self, frame, frame_time):
        if self._stop_event.is_set():
            return
        with self._latest_frame_lock:
            self._latest_frame = (frame, frame_time)

    def get_latest(self):
        with self._result_lock:
            return dict(self._result)

    def get_status(self):
        with self._status_lock:
            return dict(self._status)

    def close(self, timeout: float = 2.0):
        self._stop_event.set()
        with self._latest_frame_lock:
            self._latest_frame = None
        thread = self.thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=max(0.0, float(timeout)))
        with self._status_lock:
            self._status.update(loading=False, ready=False)
        return thread is None or not thread.is_alive()

    def _loop(self):
        load_started = time.perf_counter()
        try:
            model = self._model_factory(self.model_path)
        except Exception as error:
            load_seconds = time.perf_counter() - load_started
            with self._status_lock:
                self._status.update(
                    loading=False,
                    ready=False,
                    error=str(error),
                    load_seconds=load_seconds,
                )
            print(f"[YOLO] 后台加载失败: {error}")
            return

        load_seconds = time.perf_counter() - load_started
        if self._stop_event.is_set():
            return
        with self._status_lock:
            self._status.update(
                loading=False,
                ready=True,
                error=None,
                load_seconds=load_seconds,
            )
        print(f"[YOLO] 模型后台加载完成，用时 {load_seconds:.2f}s")

        while not self._stop_event.is_set():
            with self._latest_frame_lock:
                item = self._latest_frame
                self._latest_frame = None
            if item is None:
                self._stop_event.wait(0.002)
                continue

            frame, frame_time = item
            try:
                results = model.predict(
                    frame,
                    conf=self.conf,
                    imgsz=self.imgsz,
                    device=self.device,
                    iou=0.50,
                    max_det=3,
                    verbose=False,
                )
                parsed = self._parse_result(results, frame_time)
            except Exception as error:
                with self._status_lock:
                    self._status["last_inference_error"] = str(error)
                print(f"[YOLO] 推理出错: {error}")
                continue

            if self._stop_event.is_set():
                break
            now = time.perf_counter()
            dt = now - self._last_infer_t
            self._last_infer_t = now
            if dt > 0.0:
                self._fps_smooth = (
                    0.9 * self._fps_smooth + 0.1 * (1.0 / dt)
                )
            parsed["infer_fps"] = self._fps_smooth
            with self._result_lock:
                self._result = parsed
            with self._status_lock:
                self._status["last_inference_error"] = None

    @staticmethod
    def _parse_result(results, frame_time):
        parsed = {
            "pixel": None,
            "bbox": None,
            "confidence": 0.0,
            "track_id": None,
            "frame_time": frame_time,
            "infer_fps": 0.0,
        }
        if not results:
            return parsed
        boxes = getattr(results[0], "boxes", None)
        if boxes is None or len(boxes) == 0:
            return parsed

        best_idx = int(boxes.conf.argmax().item())
        x1, y1, x2, y2 = boxes.xyxy[best_idx].tolist()
        parsed["pixel"] = [(x1 + x2) / 2.0, (y1 + y2) / 2.0]
        parsed["bbox"] = [x1, y1, x2, y2]
        parsed["confidence"] = float(boxes.conf[best_idx].item())

        if boxes.id is not None:
            track_ids = boxes.id.cpu().numpy()
            if len(track_ids) > best_idx:
                parsed["track_id"] = int(track_ids[best_idx])
        return parsed
