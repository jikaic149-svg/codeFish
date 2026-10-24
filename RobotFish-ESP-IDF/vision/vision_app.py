import csv
import json
import math
import os
import queue
import socket
import sys
import threading
import time
import urllib.parse
import urllib.request
import cv2
import numpy as np
from flask import Flask, Response

from collections import deque
from yolo_tracker import YoloBodyTracker
from yaw_heading import CalibratedYaw
from control_config import CONTROL
from fixed_reference_tracker import ReferenceSource
from path_guidance import (
    PathGuidance,
    VelocityEstimator,
    compensate_camera_latency,
)
from turn_calibration import (
    TurnCalibrationError,
    TurnCalibrationSession,
    load_turn_calibrations,
    save_turn_calibration,
)
from vision_toolbar import (
    UiPalette,
    VisionHud,
    VisionToolbar,
    draw_unicode_text as _draw_unicode_text_impl,
    set_text_render_enabled,
)

# PIL-backed Chinese text is considerably more expensive than the OpenCV
# primitives used for tracking graphics.  The preview and MJPEG outputs are
# capped at 30 Hz, so suppress text rasterization on intermediate 60 Hz loop
# iterations.  Recording explicitly keeps text on every frame.
_UI_DRAW_STATE = {'allow_text': True}


def draw_unicode_text(*args, **kwargs):
    if _UI_DRAW_STATE['allow_text']:
        return _draw_unicode_text_impl(*args, **kwargs)
    return None

RUNTIME_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runtime")
if RUNTIME_DIR not in sys.path:
    sys.path.insert(0, RUNTIME_DIR)
from vision_protocol import pack_vision_udp_frame

# ============= 设置工作目录（带路径安全回退） =============
PREFERRED_DIR = os.path.dirname(os.path.abspath(__file__))
WORK_DIR = PREFERRED_DIR
if not os.path.exists(WORK_DIR):
    os.makedirs(WORK_DIR)
os.chdir(WORK_DIR)

# 核心提速秘籍：禁用 OpenCV MSMF 后端的硬件转换功能
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

# ==============================================================================
# [配置区] 相机采集帧率与分辨率
# ==============================================================================
TARGET_FPS = 30
TARGET_WIDTH = 1280
TARGET_HEIGHT = 720
CAMERA_INDEX = 0                 # 外接 Global Shutter Camera（索引 1 是笔记本内置相机）
EXPOSURE_MIN = -13                # Global Shutter Camera 的 DirectShow 有效下限
EXPOSURE_MAX = -4                 # 该驱动在 1280×720/30 FPS 下的有效范围
EXPOSURE_DEFAULT = -6
EXPOSURE_DIGITAL_STEP = 2.0 ** (1.0 / 3.0)
CAMERA_LATENCY_S = 0.30             # 三次屏幕秒表测试均值约 0.31 s
CAMERA_LATENCY_MAX_PREDICTION_M = 0.12

# 是否默认开启 CLAHE（直方图均衡化，水面有强烈反光或光照不均时开启）
ENABLE_CLAHE_DEFAULT = False
# Diagnostic view: the extra semantic OpenCV/colour endpoint provenance can be
# enabled independently.  The accepted reference marker and fading short
# trajectory remain visible in the normal view, matching the initial interface.
# The initial desktop view always showed the accepted fish reference marker
# and its short trajectory.  Keep that view as the default; the debug toggle
# still controls the additional endpoint provenance diagnostics.
SHOW_COLOR_AUX_OVERLAY_DEFAULT = True
# Show the actual filtered body-yaw arrow by default; the path controller's
# target-yaw arrow is drawn separately when a path is active.
SHOW_FILTERED_YAW_OVERLAY_DEFAULT = True
# Show the fish bbox in the normal desktop view, as in the initial interface.
# It is drawn in blue so it cannot be confused with the orange-tail contour.
SHOW_DETECTION_BBOX_OVERLAY_DEFAULT = True

# ==============================================================================
# [配置区] 场地真实物理尺寸与标靶 ID 映射
# ==============================================================================
# World-frame dimensions used by the homography below.  The camera image is
# calibrated in the order TL -> TR -> BR -> BL, so the top image edge is the
# long edge (x = 0 .. 4.2 m) and the left image edge is the short edge
# (y = 0 .. 2.1 m).
PHYSICAL_WIDTH = 4.2     # 画面上边：水池长边（米）
PHYSICAL_HEIGHT = 2.1    # 画面左边：水池短边（米）
POOL_SIZE_CONFIRMED = False  # No metric control until measured dimensions are supplied.


def configure_pool_size(width_m, height_m):
    global PHYSICAL_WIDTH, PHYSICAL_HEIGHT, POOL_SIZE_CONFIRMED
    if any(type(v) not in (int, float) or not math.isfinite(v) or not 0.05 <= v <= 50
           for v in (width_m, height_m)):
        raise ValueError('场地长宽须为 0.05～50 米的实测数值')
    PHYSICAL_WIDTH, PHYSICAL_HEIGHT = float(width_m), float(height_m)
    POOL_SIZE_CONFIRMED = True

MARKER_TL = 1
MARKER_TR = 2
MARKER_BR = 4
MARKER_BL = 3

# ==============================================================================
# [配置区] OpenCV 仿生鱼检测
# ==============================================================================
MARKER_PROFILE_PATH = os.path.join(WORK_DIR, "marker_profile.local.json")
MODEL_PATH = os.path.join(PREFERRED_DIR, 'fish_body_yolo', 'best.pt')
TURN_CALIBRATION_PATH = os.path.join(
    WORK_DIR, "turn_calibration.local.json"
)
POOL_CALIBRATION_PATH = os.path.join(
    WORK_DIR, "pool_calibration.local.json"
)

# ==============================================================================
# [配置区] 通信参数（平板 TCP & RoboFish ESP32 HTTP）
# ==============================================================================
TABLET_TCP_HOST = "0.0.0.0"
TABLET_TCP_PORT = 9998

# RoboFish 机器鱼 AP 网络默认配置
ROBOFISH_IP = "192.168.4.1"
ROBOFISH_PORT = 80
ROBOFISH_UDP_PORT = 9000
VISION_UDP_PERIOD_S = 0.05       # 20 Hz fixed-period latest-value frame
VISION_UDP_STALE_S = 0.25
UDP_SPEED_REFERENCE_MPS = 0.36   # 100% command corresponds to this target
# --- 坐标与速度计算平滑系数 ---
# Keep only light smoothing on the marker-derived position.  A low alpha
# makes the control point visibly lag behind quick fish motion.
POS_SMOOTHING_ALPHA = 0.65

# --- MJPEG 视频推流配置 ---
MJPEG_PORT = 8090
MJPEG_STREAM_WIDTH = 960      # 缩放至 960x540，降低带宽
MJPEG_STREAM_HEIGHT = 540
MJPEG_JPEG_QUALITY = 50       # 压缩质量设为 50，提升传输顺畅度
MJPEG_MAX_FPS = 30            # 限制推流上限 30 帧

# --- 本机预览窗口刷新帧率上限 ---
DISPLAY_MAX_FPS = 30
# Keep the measured fish trail in the normal view.  Points remain fully
# bright for a short window, then fade out and are removed after this age so a
# long-running session cannot leave a permanent yellow scribble on the image.
TRAJECTORY_FADE_START_S = 2.5
TRAJECTORY_MAX_AGE_S = 8.0
TRAJECTORY_MAX_POINTS = 240

# 辅助函数：绘制高对比度加粗黑边 OSD 文字（即使水面强光反光也能清晰可见）
def draw_outlined_text(img, text, pos, font, scale, color, thickness=2):
    x, y = pos
    cv2.putText(img, text, (x, y), font, scale, (0, 0, 0), thickness + 3, cv2.LINE_AA)
    cv2.putText(img, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)


def draw_calibration_overlay(img, points):
    """Draw the physical dimensions represented by the locked homography.

    ``points`` are image pixels in TL, TR, BR, BL order.  Keeping this overlay
    tied to the same four points used to build ``H`` makes an axis swap visible
    immediately on the live image instead of only in the numeric coordinates.
    """
    if points is None or len(points) != 4:
        return
    pts = np.asarray(points, dtype=np.float32).reshape((4, 2))
    if not np.isfinite(pts).all():
        return

    edges = (
        (0, 1, (0, 255, 0), f"TOP long edge = {PHYSICAL_WIDTH:.2f} m"),
        (0, 3, (255, 180, 0), f"LEFT short edge = {PHYSICAL_HEIGHT:.2f} m"),
    )
    for start, end, colour, label in edges:
        p0 = tuple(int(round(value)) for value in pts[start])
        p1 = tuple(int(round(value)) for value in pts[end])
        cv2.line(img, p0, p1, colour, 2, cv2.LINE_AA)
        midpoint = ((p0[0] + p1[0]) // 2, (p0[1] + p1[1]) // 2)
        # Put labels just inside the calibrated quadrilateral.  The small
        # fixed offset is intentional: it remains readable under perspective
        # without changing any coordinates used by the homography.
        if start == 0 and end == 1:
            text_pos = (midpoint[0] - 92, midpoint[1] + 24)
        else:
            text_pos = (midpoint[0] + 12, midpoint[1])
        draw_outlined_text(
            img, label, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.52, colour, 1
        )


def build_calibration_overlay(frame_shape, points):
    """Rasterize static calibration graphics once for the live UI.

    The pool is stationary, so its four corner labels and dimension lines do
    not need to invoke text/line drawing at camera rate.  A value of ``1`` is
    used as the untouched sentinel so even black outline pixels are retained
    by the mask.
    """
    if points is None or len(points) == 0:
        return None, None
    height, width = int(frame_shape[0]), int(frame_shape[1])
    overlay = np.ones((height, width, 3), dtype=np.uint8)
    for point_index, point in enumerate(points):
        px, py = int(round(point[0])), int(round(point[1]))
        cv2.circle(overlay, (px, py), 7, (0, 165, 255), -1)
        draw_outlined_text(
            overlay, str(point_index + 1), (px + 10, py - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2
        )
    if len(points) == 4:
        draw_calibration_overlay(overlay, points)
    mask = np.any(overlay != 1, axis=2).astype(np.uint8) * 255
    return overlay, mask


def apply_calibration_overlay(image, overlay, mask):
    if overlay is None or mask is None:
        return
    cv2.copyTo(overlay, mask, image)


def build_calibration_homography(points, frame_width, frame_height):
    """验证 TL→TR→BR→BL 四点并生成像素到物理坐标的单应矩阵。"""
    if not POOL_SIZE_CONFIRMED:
        return None, '请先在视觉参数中填写并应用实测场地长宽'
    if len(points) != 4:
        return None, "需要四个标定点"

    src = np.asarray(points, dtype=np.float32)
    if not np.isfinite(src).all():
        return None, "标定点包含无效数值"
    if not cv2.isContourConvex(src.reshape((-1, 1, 2))):
        return None, "四点顺序错误或区域不为凸四边形"

    top_mid = (src[0] + src[1]) * 0.5
    bottom_mid = (src[2] + src[3]) * 0.5
    left_mid = (src[0] + src[3]) * 0.5
    right_mid = (src[1] + src[2]) * 0.5
    if top_mid[1] >= bottom_mid[1] or left_mid[0] >= right_mid[0]:
        return None, "请严格按左上→右上→右下→左下点击"

    area = abs(float(cv2.contourArea(src.reshape((-1, 1, 2)))))
    if area < frame_width * frame_height * 0.05:
        return None, "标定区域过小"

    dst = np.float32([
        [0, 0], [PHYSICAL_WIDTH, 0],
        [PHYSICAL_WIDTH, PHYSICAL_HEIGHT], [0, PHYSICAL_HEIGHT]
    ])
    homography, _ = cv2.findHomography(src, dst, method=0)
    if homography is None or not np.isfinite(homography).all():
        return None, "无法计算有效标定矩阵"
    if abs(float(np.linalg.det(homography))) < 1e-12:
        return None, "标定矩阵退化"
    return homography, None


def save_pool_calibration(path, points, frame_width, frame_height):
    """Persist one stationary-pool calibration for reuse on later runs."""
    src = np.asarray(points, dtype=np.float32).reshape((4, 2))
    homography, error = build_calibration_homography(
        src, frame_width, frame_height
    )
    if error:
        raise ValueError(error)
    payload = {
        "version": 1,
        "measured_dimensions": True,
        "camera_source": str(CAMERA_INDEX),
        "frame_width": int(frame_width),
        "frame_height": int(frame_height),
        "physical_width_m": float(PHYSICAL_WIDTH),
        "physical_height_m": float(PHYSICAL_HEIGHT),
        "points_tl_tr_br_bl": src.tolist(),
        "homography": np.asarray(homography, dtype=np.float64).tolist(),
    }
    temporary_path = f"{path}.tmp"
    with open(temporary_path, "w", encoding="utf-8") as calibration_file:
        json.dump(payload, calibration_file, ensure_ascii=False, indent=2)
        calibration_file.write("\n")
    os.replace(temporary_path, path)
    return homography


def load_pool_calibration(path, frame_width, frame_height):
    """Load a saved calibration only when it matches this camera/config."""
    if not os.path.isfile(path):
        return None, None, None
    try:
        with open(path, "r", encoding="utf-8") as calibration_file:
            payload = json.load(calibration_file)
        if int(payload.get("version", 0)) != 1:
            return None, None, "unknown calibration version"
        if int(payload.get("frame_width", -1)) != int(frame_width) or \
                int(payload.get("frame_height", -1)) != int(frame_height):
            return None, None, "camera resolution changed"
        if payload.get('measured_dimensions') is not True:
            return None, None, '旧场地尺寸未经现场确认，请输入实测长宽后重新标定'
        if payload.get('camera_source') != str(CAMERA_INDEX):
            return None, None, '摄像头编号已改变或旧标定未绑定相机，请重新标定四角'
        configure_pool_size(payload.get('physical_width_m'), payload.get('physical_height_m'))
        points = np.asarray(
            payload.get("points_tl_tr_br_bl"), dtype=np.float32
        ).reshape((4, 2))
        homography, error = build_calibration_homography(
            points, frame_width, frame_height
        )
        if error:
            return None, None, error
        return homography, points.tolist(), None
    except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
        return None, None, str(error)

# ==============================================================================
# 相机多线程读取类（含曝光手动调节支持）
# ==============================================================================
class CameraStream:
    def __init__(self, src=0):
        self.src = src
        self.lock = threading.Lock()
        self.cap = cv2.VideoCapture(src, cv2.CAP_DSHOW if isinstance(src, int) else cv2.CAP_ANY)

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, TARGET_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)

        self.real_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.real_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.reported_fps = self.cap.get(cv2.CAP_PROP_FPS)

        # 曝光控制
        self.exposure_val = EXPOSURE_DEFAULT
        self.exposure_actual = None
        self.digital_exposure_gain = 1.0
        # Start in auto exposure: a fixed -6 made ordinary USB cameras nearly black.
        # The exposure buttons below already apply digital compensation.
        if isinstance(src, int):
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
        self.exposure_actual = self.cap.get(cv2.CAP_PROP_EXPOSURE)

        self.ret, self.frame = self.cap.read()
        # USB drivers may need a few reads after changing the capture format.
        for _ in range(9):
            if self.ret and self.frame is not None or not self.cap.isOpened():
                break
            time.sleep(0.05)
            self.ret, self.frame = self.cap.read()
        self.timestamp = time.time()
        self.stopped = False

        self.measured_fps = TARGET_FPS
        self._last_ts = self.timestamp

        self.thread = threading.Thread(target=self.update, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def adjust_exposure(self, delta):
        previous = self.exposure_val
        requested = int(np.clip(
            previous + int(delta), EXPOSURE_MIN, EXPOSURE_MAX
        ))
        if requested == previous:
            print(
                f"相机曝光已在边界 {previous}，"
                "不重复写入驱动"
            )
            return
        with self.lock:
            self.exposure_val = requested
            self.digital_exposure_gain = float(np.clip(
                EXPOSURE_DIGITAL_STEP ** (requested - EXPOSURE_DEFAULT),
                0.25,
                4.0,
            ))
        print(
            f"相机数字曝光补偿: {requested}，"
            f"增益 x{self.digital_exposure_gain:.2f}（硬件曝光保持启动值）"
        )

    def update(self):
        while not self.stopped:
            # DirectShow VideoCapture is not thread-safe when read() races a
            # property set.  Serialize both operations with the same lock so
            # exposure changes cannot be lost while a frame is being read.
            with self.lock:
                ret, frame = self.cap.read()
                digital_gain = self.digital_exposure_gain
            if ret:
                if abs(digital_gain - 1.0) > 1e-6:
                    frame = cv2.convertScaleAbs(
                        frame, alpha=digital_gain, beta=0.0
                    )
                now = time.time()
                dt = now - self._last_ts
                if dt > 0:
                    inst_fps = 1.0 / dt
                    self.measured_fps = 0.9 * self.measured_fps + 0.1 * inst_fps
                self._last_ts = now

                with self.lock:
                    self.ret = ret
                    self.frame = frame
                    self.timestamp = now
            else:
                time.sleep(0.001)

    def read(self):
        with self.lock:
            return self.ret, self.frame.copy() if self.frame is not None else None, self.timestamp

    def release(self):
        self.stopped = True
        if self.thread.is_alive():
            self.thread.join()
        self.cap.release()

# ==============================================================================
# MJPEG 视频流服务器
# ==============================================================================
class MJPEGServer:
    def __init__(self, host="0.0.0.0", port=MJPEG_PORT, max_viewers=1):
        self.host = host
        self.port = port
        self.frame = None
        self.lock = threading.Lock()
        self.max_viewers = max_viewers
        self._viewer_count = 0
        self._viewer_lock = threading.Lock()
        self.last_update_t = 0.0
        self.app = Flask(__name__)

        @self.app.route('/video.mjpg')
        def video():
            with self._viewer_lock:
                if self._viewer_count >= self.max_viewers:
                    print(f"视频流已有 {self._viewer_count} 个客户端在看，拒绝新连接")
                    return Response("视频流已被占用，请先关闭其他正在观看的窗口/设备", status=503)
            return Response(
                self.generate(),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )

    def update(self, frame):
        if frame is None:
            return

        now = time.time()
        if now - self.last_update_t < (1.0 / MJPEG_MAX_FPS):
            return
        self.last_update_t = now

        small_frame = cv2.resize(frame, (MJPEG_STREAM_WIDTH, MJPEG_STREAM_HEIGHT))
        with self.lock:
            self.frame = small_frame

    def generate(self):
        with self._viewer_lock:
            self._viewer_count += 1
            print(f"视频流新增1个观看者，当前共 {self._viewer_count} 个")

        last_send_t = 0.0
        min_interval = 1.0 / MJPEG_MAX_FPS

        try:
            while True:
                now = time.time()
                wait = min_interval - (now - last_send_t)
                if wait > 0:
                    time.sleep(wait)

                with self.lock:
                    frame = None if self.frame is None else self.frame.copy()

                if frame is None:
                    time.sleep(0.01)
                    continue

                ok, jpg = cv2.imencode(
                    '.jpg', frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), MJPEG_JPEG_QUALITY]
                )
                if not ok:
                    continue

                last_send_t = time.time()
                yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n'
                    + jpg.tobytes() +
                    b'\r\n'
                )
        except GeneratorExit:
            pass
        finally:
            with self._viewer_lock:
                self._viewer_count = max(0, self._viewer_count - 1)
                print(f"视频流断开1个观看者，当前共 {self._viewer_count} 个")

    def start(self):
        threading.Thread(
            target=lambda: self.app.run(
                host=self.host,
                port=self.port,
                threaded=True,
                debug=False,
                use_reloader=False
            ),
            daemon=True
        ).start()
        print(f"MJPEG视频流已启动: http://0.0.0.0:{self.port}/video.mjpg")

# ==============================================================================
# 平板 TCP 服务器（含实时 TX/RX 传输频率计算）
# ==============================================================================
class TabletTCPServer:
    def __init__(self, host=TABLET_TCP_HOST, port=TABLET_TCP_PORT):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.client_addr = None
        self._lock = threading.Lock()
        self._stop = False

        self._accept_thread = None
        self._sender_thread = None
        self._receiver_thread = None

        self._pending_data = None
        self._pending_lock = threading.Lock()
        self._has_pending = threading.Event()

        self._latest_trajectory = None
        self._latest_tracking_err = None
        self._command_queue = queue.Queue()
        self._rx_lock = threading.Lock()

        # 频率计算参数
        self.tx_fps = 0.0
        self.rx_fps = 0.0
        self._last_tx_t = time.time()
        self._last_rx_t = time.time()

        self._init_server()

    def _init_server(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            self.server_socket.settimeout(1.0)
            print(f"平板 TCP 有线双向服务器已启动: {self.host}:{self.port}")

            self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
            self._accept_thread.start()
            self._sender_thread = threading.Thread(target=self._sender_loop, daemon=True)
            self._sender_thread.start()
            self._receiver_thread = threading.Thread(target=self._receiver_loop, daemon=True)
            self._receiver_thread.start()
        except Exception as e:
            print(f"平板 TCP 服务器启动失败: {e}")

    def _accept_loop(self):
        while not self._stop:
            try:
                conn, addr = self.server_socket.accept()
                conn.settimeout(0.5)
                with self._lock:
                    if self.client_socket:
                        try:
                            self.client_socket.close()
                        except:
                            pass
                    self.client_socket = conn
                    self.client_addr = addr
                print(f"平板 UI 已建立连接: {addr}")
            except socket.timeout:
                continue
            except Exception:
                continue

    def _sender_loop(self):
        while not self._stop:
            if not self._has_pending.wait(timeout=0.5):
                continue
            with self._pending_lock:
                data_dict = self._pending_data
                self._has_pending.clear()

            with self._lock:
                sock = self.client_socket
            if sock is None or data_dict is None:
                continue

            try:
                json_str = json.dumps(data_dict, ensure_ascii=False, default=float) + "\n"
                sock.sendall(json_str.encode('utf-8'))

                # 计算发送频率
                now = time.time()
                dt = now - self._last_tx_t
                if dt > 0:
                    self.tx_fps = 0.9 * self.tx_fps + 0.1 * (1.0 / dt)
                self._last_tx_t = now

            except (socket.error, BrokenPipeError):
                with self._lock:
                    if self.client_socket:
                        print(f"平板 UI 连接断开: {self.client_addr}")
                        try:
                            self.client_socket.close()
                        except:
                            pass
                        self.client_socket = None

    def _receiver_loop(self):
        buffer = ""
        while not self._stop:
            with self._lock:
                sock = self.client_socket

            if sock is None:
                time.sleep(0.1)
                continue

            try:
                data = sock.recv(2048).decode('utf-8')
                if not data:
                    time.sleep(0.01)
                    continue

                buffer += data
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if line:
                        # 计算接收频率
                        now = time.time()
                        dt = now - self._last_rx_t
                        if dt > 0:
                            self.rx_fps = 0.9 * self.rx_fps + 0.1 * (1.0 / dt)
                        self._last_rx_t = now

                        try:
                            parsed = json.loads(line)
                            msg_type = parsed.get("type")

                            if msg_type == "trajectory":
                                with self._rx_lock:
                                    self._latest_trajectory = parsed
                                num_pts = parsed.get('total_points', len(parsed.get('points', [])))
                                mode = parsed.get('motion_mode', 'bionic')
                                rev = parsed.get('reverse', False)
                                loop_flag = parsed.get('loop', False)
                                print(f" [UI 消息] 收到轨迹规划: 点数={num_pts} | 模式={mode} | 反向={rev} | 循环={loop_flag}")

                            elif msg_type == "tracking_error":
                                with self._rx_lock:
                                    self._latest_tracking_err = parsed

                            elif msg_type == "command":
                                cmd = parsed.get('cmd')
                                self._command_queue.put(parsed)
                                print(f" [UI 消息] 收到控制指令: 【{cmd.upper()}】")

                        except json.JSONDecodeError:
                            pass

            except socket.timeout:
                continue
            except (socket.error, ConnectionResetError):
                with self._lock:
                    if self.client_socket:
                        print(f"平板 UI 接收通道已断开")
                        try:
                            self.client_socket.close()
                        except:
                            pass
                        self.client_socket = None
            except Exception:
                time.sleep(0.05)

    def send(self, data_dict):
        with self._pending_lock:
            self._pending_data = data_dict
        self._has_pending.set()

    def get_comm_fps(self):
        now = time.time()
        if now - self._last_tx_t > 1.5:
            self.tx_fps *= 0.5
        if now - self._last_rx_t > 1.5:
            self.rx_fps *= 0.5
        return round(self.tx_fps, 1), round(self.rx_fps, 1)

    def get_latest_tracking_error(self):
        with self._rx_lock:
            return self._latest_tracking_err

    def get_latest_trajectory(self):
        with self._rx_lock:
            return self._latest_trajectory

    def get_next_command(self):
        try:
            return self._command_queue.get_nowait()
        except queue.Empty:
            return None

    def close(self):
        self._stop = True
        with self._lock:
            if self.client_socket:
                try:
                    self.client_socket.close()
                except:
                    pass
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass

# ==============================================================================
# RoboFish ESP32S3 异步 HTTP 通信类 (含 10Hz 限频防冲垮 + 频率统计)
# ==============================================================================
class RoboFishComm:
    """HTTP management plus a fixed-period, latest-value UDP data plane."""

    def __init__(self, ip=ROBOFISH_IP, port=ROBOFISH_PORT,
                 udp_port=ROBOFISH_UDP_PORT,
                 udp_period_s=VISION_UDP_PERIOD_S):
        self.ip = ip
        self.port = port
        self.base_url = f"http://{ip}:{port}"
        self.udp_port = int(udp_port)
        self.udp_period_s = max(0.01, float(udp_period_s))
        self.cmd_queue = queue.Queue(maxsize=1)  # 队列深度设为 1，只留最新指令
        self.stopped = False

        # 单片机通信频率统计与限频控制
        self.mcu_hz = 0.0
        self._last_send_t = time.time()
        self._last_cmd_t = 0.0                  # 发包间隔时间戳
        self._last_params = None                # 指令去重缓存
        self.vision_seq = 0                     # MCU 视觉 PID 样本序号
        self._http_lock = threading.Lock()

        self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_target = (self.ip, self.udp_port)
        self._udp_lock = threading.Lock()
        self._udp_latest = None
        self._udp_latest_t = 0.0
        self._udp_stop = threading.Event()
        self.udp_tx_hz = 0.0
        self.udp_last_send_t = 0.0

        self.thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.thread.start()
        self._udp_thread = threading.Thread(
            target=self._udp_loop, name="VisionUdpSender", daemon=True
        )
        self._udp_thread.start()
        print(
            f"RoboFish 管理 HTTP + 循迹 UDP 已就绪 -> HTTP {self.base_url}, "
            f"UDP {self.ip}:{self.udp_port} @ {1.0 / self.udp_period_s:.1f}Hz"
        )

    def _request_control(self, variable, value=1, timeout=0.3):
        query_string = urllib.parse.urlencode({"var": variable, "val": value})
        url = f"{self.base_url}/control?{query_string}"
        req = urllib.request.Request(
            url, headers={
                'User-Agent': 'RoboFish-Visual-Tracker',
                'Connection': 'close',
            }
        )
        with urllib.request.urlopen(req, timeout=timeout) as response:
            return response.read().decode('utf-8', errors='replace').strip()

    def _udp_loop(self):
        next_t = time.monotonic()
        while not self._udp_stop.is_set():
            next_t += self.udp_period_s
            wait_s = next_t - time.monotonic()
            if wait_s > 0:
                self._udp_stop.wait(wait_s)
            if self._udp_stop.is_set():
                break
            # If a send or scheduler pause overruns one or more periods,
            # discard the missed slots instead of emitting a burst of frames.
            now = time.monotonic()
            if now - next_t > self.udp_period_s:
                next_t = now
            with self._udp_lock:
                latest = self._udp_latest
                latest_t = self._udp_latest_t
            if latest is None or time.monotonic() - latest_t > VISION_UDP_STALE_S:
                continue
            try:
                self._udp_socket.sendto(
                    pack_vision_udp_frame(**latest), self._udp_target
                )
                now = time.monotonic()
                if self.udp_last_send_t > 0.0:
                    dt = now - self.udp_last_send_t
                    if dt > 0.0:
                        self.udp_tx_hz = 0.9 * self.udp_tx_hz + 0.1 / dt
                self.udp_last_send_t = now
            except OSError as error:
                # UDP is deliberately fire-and-forget; HTTP remains available
                # for management and explicit STOP if the datagram path fails.
                print(f"[UDP 发送失败] {error}")

    def set_tracking_command(self, steering_demand, speed_target_mps,
                             brake_request=False):
        """Publish one latest tracking command; a thread sends it at 20 Hz.

        ``steering_demand`` is positive for a left correction, matching
        :class:`PathGuidance`; the same polarity is carried on the UDP wire.
        """
        self.vision_seq = (self.vision_seq + 1) & 0xFFFFFFFF
        power = int(round(np.clip(
            float(speed_target_mps) / max(1e-6, UDP_SPEED_REFERENCE_MPS) * 100.0,
            0.0, 100.0
        )))
        steering = int(round(np.clip(float(steering_demand) * 100.0, -100.0, 100.0)))
        command = {
            "sequence": self.vision_seq,
            "timestamp_ms": int(time.monotonic_ns() // 1_000_000),
            "power_percent": power,
            "steering_percent": steering,
            "brake": bool(brake_request),
        }
        with self._udp_lock:
            self._udp_latest = command
            self._udp_latest_t = time.monotonic()
        return command

    def stop_tracking(self):
        with self._udp_lock:
            self._udp_latest = None
            self._udp_latest_t = 0.0

    def _request_vision_pid(
        self, sequence, x_error_m, y_error_m, speed_mps,
        curve_severity, brake_request, timeout=0.3
    ):
        query_string = urllib.parse.urlencode({
            "seq": int(sequence),
            "x_error": f"{float(x_error_m):.4f}",
            "y_error": f"{float(y_error_m):.4f}",
            "speed": f"{float(speed_mps):.4f}",
            "curve": f"{float(curve_severity):.3f}",
            "brake": 1 if brake_request else 0,
        })
        url = f"{self.base_url}/vision_pid?{query_string}"
        req = urllib.request.Request(
            url, headers={
                'User-Agent': 'RoboFish-Visual-Tracker',
                'Connection': 'close',
            }
        )
        with urllib.request.urlopen(req, timeout=timeout) as response:
            payload = json.loads(response.read().decode('utf-8'))
        if not payload.get("ok") or int(payload.get("seq", -1)) != int(sequence):
            raise RuntimeError(f"invalid vision PID ACK: {payload}")
        return payload

    def _worker_loop(self):
        while not self.stopped:
            try:
                params = self.cmd_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                if params.get("kind") == "vision_pid":
                    request_started = time.perf_counter()
                    with self._http_lock:
                        payload = self._request_vision_pid(
                            params["seq"], params["x_error"], params["y_error"],
                            params["speed"], params["curve"], params["brake"],
                            timeout=0.3
                        )
                    latency_ms = (time.perf_counter() - request_started) * 1000.0
                    print(
                        f"[VISION ACK] seq={payload['seq']} "
                        f"applied={payload.get('applied_seq', '-')} "
                        f"esc={payload.get('esc', '-')}° "
                        f"RTT={latency_ms:.1f}ms"
                    )
                else:
                    action_map = {
                        "forward": "servo_forward",
                        "backward": "servo_backward",
                        "left": "servo_left",
                        "right": "servo_right",
                        "stop": "servo_stop",
                        "mode_bionic": "mode",
                        "mode_prop": "mode",
                        "mode_hybrid": "mode",
                    }
                    with self._http_lock:
                        self._request_control(action_map[params["action"]], timeout=0.2)

                # 统计单片机交互实际成功 Hz
                now = time.time()
                dt = now - self._last_send_t
                if dt > 0:
                    self.mcu_hz = 0.9 * self.mcu_hz + 0.1 * (1.0 / dt)
                self._last_send_t = now

            except Exception as e:
                if params.get("kind") == "vision_pid":
                    print(
                        f"[VISION ACK 失败] seq={params.get('seq', '-')} "
                        f"x={float(params.get('x_error', 0.0)):+.4f}m "
                        f"y={float(params.get('y_error', 0.0)):.4f}m "
                        f"错误={e}"
                    )
                else:
                    print(f"[HTTP 发送失败] 无法连通机器鱼 ({self.base_url}): {e}")

    def get_mcu_hz(self):
        if self.udp_last_send_t <= 0.0 or time.monotonic() - self.udp_last_send_t > 1.5:
            self.udp_tx_hz *= 0.5
        return round(self.udp_tx_hz, 1)

    def _send_async(self, params):
        now = time.time()
        is_stop = params.get("action") == "stop"

        # 1. 限频控制：单片机限制最高下发频率为 10Hz (间隔>=0.1s)
        if not is_stop and (now - self._last_cmd_t < 0.1):
            return False

        self._last_cmd_t = now
        self._last_params = params

        # 2. 清空队列旧指令；相同动作仍需刷新固件的 2 秒推进租约。
        while not self.cmd_queue.empty():
            try:
                self.cmd_queue.get_nowait()
            except queue.Empty:
                break

        try:
            self.cmd_queue.put_nowait(params)
            return True
        except queue.Full:
            return False


    def ensure_bionic_mode(self):
        """The authenticated workstation adapter must confirm mode 1 first."""
        self.stop_tracking()
        print('请通过上位机启动纯仿生视觉循迹，禁止使用旧混合模式入口。')
        return False

    def send_command(self, cmd_str):
        if cmd_str in ["stop", "emergency_stop"]:
            print(f"[HTTP 下发] 机器鱼急停/停止推进")
            self.stop_tracking()
            self._send_async({"action": "stop"})

    def stop_now(self):
        """同步停止，用于退出、清轨迹和失去视觉目标。"""
        self.stop_tracking()
        self._last_params = None
        while not self.cmd_queue.empty():
            try:
                self.cmd_queue.get_nowait()
            except queue.Empty:
                break
        try:
            with self._http_lock:
                self._request_control("servo_stop", timeout=0.5)
            print("[HTTP 反馈] 推进已停止")
            return True
        except Exception as error:
            print(f"[HTTP 停止失败] {error}")
            return False

    def process_tracking_error(
        self, cross_m, along_m, dist_m, speed_mps=0.0,
        curve_severity=0.0, brake_request=False,
        cross_track_m=0.0, heading_error_deg=0.0,
        path_curvature_per_m=0.0, steering_demand=0.0,
        speed_target_mps=None, observed_at=None, feedback_valid=False,
    ):
        """Publish final PC control demand over the fixed-period UDP plane."""
        target_speed = (
            float(speed_target_mps)
            if speed_target_mps is not None and np.isfinite(float(speed_target_mps))
            else float(speed_mps)
        )
        command = self.set_tracking_command(
            steering_demand=float(steering_demand),
            speed_target_mps=target_speed,
            brake_request=bool(brake_request),
        )
        print(
            f"[VISION目标] seq={command['sequence']} "
            f"power={command['power_percent']}% "
            f"steering={command['steering_percent']:+d}% "
            f"v={float(speed_mps):.2f}/{target_speed:.2f}m/s 离={cross_track_m:.2f}m "
            f"角={heading_error_deg:+.0f}° 曲率={path_curvature_per_m:+.2f}/m "
            f"制动={'是' if brake_request else '否'}"
        )

    def close(self):
        self.stopped = True
        self.stop_tracking()
        self._udp_stop.set()
        if self._udp_thread.is_alive():
            self._udp_thread.join(timeout=1.0)
        try:
            self._udp_socket.close()
        except OSError:
            pass
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)

# ==============================================================================
# 异步录像写入类
# ==============================================================================
from video_recorder import AsyncVideoRecorder


# ==============================================================================
# 主函数
# ==============================================================================
def main():
    print("\n" + "=" * 60)
    print("YOLO 鱼体检测 + IMU yaw + 串级路径控制正在启动...")
    print(f"工作目录: {os.path.abspath(WORK_DIR)}")
    print(f"目标捕获: {TARGET_WIDTH}x{TARGET_HEIGHT} @ {TARGET_FPS}FPS")
    print(
        f"相机延迟补偿: {CAMERA_LATENCY_S * 1000:.0f} ms "
        f"(最大预测 {CAMERA_LATENCY_MAX_PREDICTION_M:.2f} m)"
    )
    print(
        "场地尺寸须经实测确认；等待已确认的标定或网页输入。"
    )
    print("画面与操作通过本机工作站转接；控制目标交给工作站统一仲裁。")
    print(f"检测模型：{MODEL_PATH}；鱼头标定建立 IMU yaw 零位")
    print("=" * 60 + "\n")

    cam = CameraStream(src=CAMERA_INDEX).start()
    if not cam.ret:
        print("无法打开相机！请检查 USB 接口是否松动，或尝试修改索引。")
        cam.release()
        return

    print(f"相机流已就绪：{cam.real_width} x {cam.real_height}")
    if (cam.real_width, cam.real_height) != (TARGET_WIDTH, TARGET_HEIGHT):
        print(
            f"[WARN] camera driver selected {cam.real_width}x{cam.real_height}; "
            f"requested {TARGET_WIDTH}x{TARGET_HEIGHT}"
        )

    # 1. 启动平板 TCP 服务器
    tablet_server = TabletTCPServer(host=TABLET_TCP_HOST, port=TABLET_TCP_PORT)

    # 2. 启动 MJPEG 视频推流服务器
    mjpeg_server = MJPEGServer()
    mjpeg_server.start()

    # 3. 启动 RoboFish ESP32 HTTP 通信与 PID 控制服务
    fish_comm = RoboFishComm(ip=ROBOFISH_IP, port=ROBOFISH_PORT)

    # Detection owns fish identity; optical flow measures brief detection gaps.
    reference_tracker = YoloBodyTracker(MODEL_PATH)
    detector_meter = {'time':None,'count':0,'hz':0.}
    white_head_tracker = reference_tracker  # Existing white-threshold/preview controls.
    print("[VISION MODE] YOLO_PT | position=fish centre | yaw=calibrated body IMU")

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    use_clahe = ENABLE_CLAHE_DEFAULT

    # Entries are ``(pixel_x, pixel_y, wall_time)``.  Keeping the timestamp
    # with each sample lets the renderer fade old motion instead of retaining
    # an opaque trail for the whole lifetime of the process.
    trajectory = deque(maxlen=TRAJECTORY_MAX_POINTS)

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    recorder = None
    is_recording = False
    last_recorded_frame_time = None
    current_status_text = "READY"
    is_tracing_active = False
    drawn_path_state = {
        'pixels': [],
        'drawing': False,
        'active': False,
        'segment': 0,
        'last_control_t': 0.0,
        'lost_since': None,
    }
    try:
        turn_calibration_results = load_turn_calibrations(
            TURN_CALIBRATION_PATH
        )
    except (OSError, ValueError, TypeError) as error:
        turn_calibration_results = {}
        print(f"转圈标定文件无法使用，将回退 0.20m 估计半径：{error}")
    left_turn_fit = turn_calibration_results.get("LEFT")
    right_turn_fit = turn_calibration_results.get("RIGHT")
    path_guidance = PathGuidance(
        spacing_m=0.03,
        lookahead_m=CONTROL['guidance']['lookahead_m'],
        min_lookahead_m=CONTROL['guidance']['min_lookahead_m'],
        k_f=CONTROL['guidance']['cross_track_gain'],
        cruise_speed_mps=CONTROL['guidance']['cruise_speed_mps'],
        min_speed_target_mps=CONTROL['guidance']['min_speed_mps'],
        full_attraction_error_m=0.18,
        curve_preview_m=0.32,
        estimated_min_turn_radius_m=0.20,
        left_turn_radius_m=(
            left_turn_fit.radius_m if left_turn_fit is not None else None
        ),
        right_turn_radius_m=(
            right_turn_fit.radius_m if right_turn_fit is not None else None
        ),
        max_x_error_m=(1.0 / 3.0),
        arrival_radius_m=0.20,
        brake_radius_m=0.40,
    )
    turn_calibration_session = TurnCalibrationSession()
    for direction, fit in turn_calibration_results.items():
        direction_name = "左转" if direction == "LEFT" else "右转"
        print(
            f"已加载{direction_name}实测半径 {fit.radius_m:.3f}m "
            f"（直径 {fit.diameter_m:.3f}m，残差 {fit.rms_residual_m:.3f}m）"
        )
    missing_turns = [
        "左转" if direction == "LEFT" else "右转"
        for direction in ("LEFT", "RIGHT")
        if direction not in turn_calibration_results
    ]
    if missing_turns:
        print(
            f"{'、'.join(missing_turns)}尚无实测转圈半径，"
            "曲率前馈暂用 0.20m 估计值。"
        )

    calib_state = {
        'H': None,
        # The pool is stationary.  Keep the inverse beside H and update it
        # only when calibration changes; np.linalg.inv() in the 60 Hz loop
        # needlessly costs CPU and creates temporary arrays.
        'inv_H': None,
        'is_calibrating': False,
        'manual_locked': False,
        'auto_locked': False,
        'auto_stable_count': 0,
        'auto_prev_points': None,
        'pts_raw': [],
        'pts_disp': [],
        'overlay': None,
        'overlay_mask': None,
        'overlay_dirty': True,
    }
    saved_homography, saved_points, calibration_error = load_pool_calibration(
        POOL_CALIBRATION_PATH, cam.real_width, cam.real_height
    )
    if saved_homography is not None:
        calib_state['H'] = saved_homography
        try:
            calib_state['inv_H'] = np.linalg.inv(saved_homography)
        except (TypeError, ValueError, np.linalg.LinAlgError):
            calib_state['H'] = None
            calib_state['inv_H'] = None
            print("保存的场地标定矩阵不可逆，将重新等待标定。")
        if calib_state['H'] is None:
            saved_homography = None
        else:
            calib_state['manual_locked'] = True
            calib_state['pts_raw'] = saved_points
            calib_state['pts_disp'] = [
                (int(round(point[0])), int(round(point[1])))
                for point in saved_points
            ]
            calib_state['overlay_dirty'] = True
            print(
                f"已加载固定水池标定：{POOL_CALIBRATION_PATH}，"
                f"上边 {PHYSICAL_WIDTH:.2f}m、左边 {PHYSICAL_HEIGHT:.2f}m"
            )
    elif calibration_error:
        print(f"固定水池标定未加载，将等待 ArUco/手动标定：{calibration_error}")

    toolbar = VisionToolbar(enabled=False)
    toolbar.binary_threshold = white_head_tracker.val_min
    hud = VisionHud()
    pending_ui_actions = deque()
    frame_state = {
        'latest': None,
        'reference_position': None,
        'reference_source': ReferenceSource.INVALID,
        'tail_marker_position': None,
        # Bounding box of the accepted orange contour.  Kept separate from
        # the reference/body box so the diagnostic view can prove that the
        # yaw tail really came from an orange pixel component.
        'tail_marker_bbox': None,
        'tail_marker_quality': 0.0,
    }
    marker_roi_state = {
        'selecting': False,
        'dragging': False,
        'start': None,
        'end': None,
        'confirmed_bbox': None,
        'confirmed_center': None,
        'confirmed_until': 0.0,
    }
    heading_state = {
        'selecting': False,
        'tail_point': None,
        'head_point': None,
        'unit_vector': None,
        'world_unit_vector': None,
        'control_heading': None,
        'control_heading_source': None,
        'angle_deg': None,
        'confirmed_until': 0.0,
    }
    pointer_state = {
        'pressed_toolbar_action': None,
    }

    # 像素平滑与平滑轨迹控制参数
    pixel_state = {'smooth_x': None, 'smooth_y': None}
    PIXEL_SMOOTH_ALPHA = 0.35      # 像素一阶平滑滤波系数
    MIN_TRAJ_DIST_SQ = 3.0 ** 2    # 移动阈值死区

    pos_state = {'smooth_x': None, 'smooth_y': None, 'frame_time': None}
    velocity_estimator = VelocityEstimator(
        history_seconds=0.40,
        min_span_seconds=0.12,
        blend=0.45,
        max_speed_mps=0.80,
    )
    imu_yaw = CalibratedYaw()
    body_yaw_state = {
        'yaw_deg': None,
        'visual_yaw_deg': None,
        'fused_yaw_deg': None,
        'motion_prior_deg': None,
        'motion_gate_applied': False,
        'yaw_rate_rps': None,
        'unit_vector': None,
        'source': 'INVALID',
        'prediction_age_s': None,
        'usable': False,
        'last_frame_time': None,
        'last_update_wall_t': None,
        'raw_yaw_deg': None,
        'measurement_rejected': False,
        'polarity_corrected': False,
        'reacquire_pending': False,
        'reacquire_samples': 0,
        'pose_rejection_reason': None,
        'detector_raw_yaw_deg': None,
        'color_raw_yaw_deg': None,
        'yaw_measurement_source': 'NONE',
        'color_tail_pixel': None,
        'color_head_pixel': None,
        'color_head_source': 'UNAVAILABLE',
        'head_color_confidence': 0.0,
        'white_dynamic_value_min': None,
        'detector_head_in_white': None,
        'detector_head_distance_to_white_px': None,
        'tail_color_confidence': 0.0,
        # A manual head-direction click establishes the 0/180-degree
        # polarity.  OpenCV measurements may still be displayed before this,
        # but they are not allowed to start vehicle tracking by themselves.
        'polarity_calibrated': False,
        'tail_pixel': None,
        'head_pixel': None,
    }
    motion_prior_state = {
        'last_t': None,
        'last_r_cmd_rps': 0.0,
        'last_heading_rad': None,
        'last_fused_yaw_deg': None,
        'gate_count': 0,
    }
    # Rendering and control consume the same calibrated, fresh IMU heading.
    yaw_control_state = {
        'heading': None,
        'last_valid_t': None,
        'source': 'IMU_WAIT',
        'reported_source': None,
    }

    def reset_yaw_control_state():
        yaw_control_state.update({
            'heading': None,
            'last_valid_t': None,
            'source': 'IMU_WAIT',
            'reported_source': None,
        })

    def select_yaw_for_control(now):
        # Position never supplies an alternative heading when IMU is unavailable.
        if body_yaw_state['usable'] and body_yaw_state['unit_vector'] is not None:
            return body_yaw_state['unit_vector'], body_yaw_state['yaw_rate_rps'], body_yaw_state['source']
        return None, 0.0, 'IMU_WAIT'

    velocity_state = {'vx_smooth': 0.0, 'vy_smooth': 0.0}
    fps_state = {'last_loop_t': time.time(), 'loop_fps': 0.0}
    perf_state = {
        'last_report_t': 0.0,
        'last_endpoint_report_t': 0.0,
    }
    display_state = {
        'last_t': 0.0,
        'last_yaw_draw_t': 0.0,
        'window_width': cam.real_width,
        'window_height': cam.real_height,
    }
    show_debug_overlay = False
    # Keep the diagnostic view in full-frame binary mode until the operator
    # toggles it off with V.  The threshold slider remains visible in the
    # toolbar so the effect of each adjustment is immediately observable.
    show_binary_preview = False
    show_color_aux_overlay = SHOW_COLOR_AUX_OVERLAY_DEFAULT
    show_filtered_yaw_overlay = SHOW_FILTERED_YAW_OVERLAY_DEFAULT
    show_detection_bbox_overlay = SHOW_DETECTION_BBOX_OVERLAY_DEFAULT

    last_motion_mode = None  # 记录上一次发出的运动模式

    window_name = "AUV OpenCV Tracker (RoboFish PID)"
    display_width = 960
    display_height = int(display_width * cam.real_height / cam.real_width)
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, display_width, display_height)
    display_state['window_width'] = display_width
    display_state['window_height'] = display_height

    def mouse_callback(event, x, y, flags, param):
        # Git 原版：HighGUI 直接给出原图坐标，不再二次换算。
        rx = int(np.clip(x, 0, cam.real_width - 1))
        ry = int(np.clip(y, 0, cam.real_height - 1))

        # Handle the live binary-threshold slider before buttons/path input.
        slider_hit = toolbar.slider_hit_test(
            rx, ry, cam.real_width, cam.real_height
        )
        if toolbar.slider_dragging:
            if event in (cv2.EVENT_MOUSEMOVE, cv2.EVENT_LBUTTONUP):
                white_head_tracker.set_binary_threshold(
                    toolbar.update_slider(rx, cam.real_width, cam.real_height)
                )
            if event == cv2.EVENT_LBUTTONUP:
                toolbar.end_slider_drag()
            drawn_path_state['drawing'] = False
            marker_roi_state['dragging'] = False
            return
        if event == cv2.EVENT_LBUTTONDOWN and slider_hit:
            white_head_tracker.set_binary_threshold(
                toolbar.begin_slider_drag(rx, cam.real_width, cam.real_height)
            )
            drawn_path_state['drawing'] = False
            marker_roi_state['dragging'] = False
            return

        if event in (cv2.EVENT_LBUTTONDOWN, cv2.EVENT_LBUTTONUP):
            toolbar_action = toolbar.hit_test(
                rx, ry, cam.real_width, cam.real_height
            )
            if toolbar_action is not None:
                if event == cv2.EVENT_LBUTTONDOWN:
                    pending_ui_actions.append(toolbar_action)
                    pointer_state['pressed_toolbar_action'] = toolbar_action
                elif pointer_state['pressed_toolbar_action'] != toolbar_action:
                    pending_ui_actions.append(toolbar_action)
                pointer_state['pressed_toolbar_action'] = (
                    toolbar_action if event == cv2.EVENT_LBUTTONDOWN else None
                )
                drawn_path_state['drawing'] = False
                marker_roi_state['dragging'] = False
                return
            if toolbar.contains(rx, ry, cam.real_height):
                return

        if marker_roi_state['selecting']:
            canvas_bottom = max(0, cam.real_height - toolbar.height - 1)
            marker_y = int(np.clip(ry, 0, canvas_bottom))
            if event == cv2.EVENT_LBUTTONDOWN:
                marker_roi_state['start'] = (rx, marker_y)
                marker_roi_state['end'] = (rx, marker_y)
                marker_roi_state['dragging'] = True
            elif event == cv2.EVENT_MOUSEMOVE and marker_roi_state['dragging']:
                marker_roi_state['end'] = (rx, marker_y)
            elif event == cv2.EVENT_LBUTTONUP and marker_roi_state['dragging']:
                marker_roi_state['end'] = (rx, marker_y)
                marker_roi_state['dragging'] = False
                start = marker_roi_state['start']
                end = marker_roi_state['end']
                if start is not None and end is not None:
                    pending_ui_actions.append((
                        'APPLY_MARKER_ROI',
                        (start[0], start[1], end[0], end[1])
                    ))
            elif event == cv2.EVENT_RBUTTONDOWN:
                marker_roi_state['selecting'] = False
                marker_roi_state['dragging'] = False
                marker_roi_state['start'] = None
                marker_roi_state['end'] = None
                print("已取消整鱼框选。")
            return

        if heading_state['selecting']:
            if event == cv2.EVENT_LBUTTONDOWN:
                pending_ui_actions.append(('APPLY_HEAD_DIRECTION', (rx, ry)))
            elif event == cv2.EVENT_RBUTTONDOWN:
                heading_state['selecting'] = False
                print("已取消鱼头方向标定，原有初始方向保持不变。")
            return

        if event == cv2.EVENT_LBUTTONDOWN and calib_state['is_calibrating']:
            calib_state['pts_raw'].append([rx, ry])
            calib_state['pts_disp'].append((rx, ry))
            calib_state['overlay_dirty'] = True
            if len(calib_state['pts_raw']) == 4:
                homography, error = build_calibration_homography(
                    calib_state['pts_raw'], cam.real_width, cam.real_height
                )
                if error:
                    print(f"手动标定失败：{error}，请重新按左上→右上→右下→左下点击。")
                    calib_state['pts_raw'].clear()
                    calib_state['pts_disp'].clear()
                    calib_state['overlay_dirty'] = True
                else:
                    calib_state['H'] = homography
                    calib_state['inv_H'] = np.linalg.inv(homography)
                    calib_state['overlay_dirty'] = True
                    calib_state['manual_locked'] = True
                    calib_state['is_calibrating'] = False
                    calib_state['pts_raw'] = [
                        [float(value) for value in point]
                        for point in calib_state['pts_raw']
                    ]
                    try:
                        save_pool_calibration(
                            POOL_CALIBRATION_PATH,
                            calib_state['pts_raw'],
                            cam.real_width,
                            cam.real_height,
                        )
                    except (OSError, ValueError) as save_error:
                        print(f"场地标定已生效，但保存失败：{save_error}")
                    else:
                        print(f"场地标定已保存：{POOL_CALIBRATION_PATH}")
                    print("手动标定完成并锁定！")

        elif not calib_state['is_calibrating']:
            if event == cv2.EVENT_LBUTTONDOWN:
                drawn_path_state['pixels'] = [(rx, ry)]
                drawn_path_state['drawing'] = True
                drawn_path_state['active'] = False
                drawn_path_state['segment'] = 0
                path_guidance.clear()
            elif event == cv2.EVENT_MOUSEMOVE and drawn_path_state['drawing']:
                if toolbar.contains(rx, ry, cam.real_height):
                    return
                points = drawn_path_state['pixels']
                if not points or (rx - points[-1][0]) ** 2 + (ry - points[-1][1]) ** 2 >= 5 ** 2:
                    points.append((rx, ry))
            elif event == cv2.EVENT_LBUTTONUP and drawn_path_state['drawing']:
                drawn_path_state['drawing'] = False
                if len(drawn_path_state['pixels']) < 2:
                    drawn_path_state['pixels'].clear()
            elif event == cv2.EVENT_RBUTTONDOWN:
                drawn_path_state['pixels'].clear()
                drawn_path_state['drawing'] = False
                drawn_path_state['active'] = False
                drawn_path_state['segment'] = 0
                path_guidance.clear()

    cv2.setMouseCallback(window_name, mouse_callback)
    print("\n鼠标与窗口尺寸已还原 Git 原版。")
    print(
        "操作统一使用上位机按钮；完整画面可框选和绘制轨迹，右键清除。"
        "按 d 可切换调试叠加层（默认关闭）。\n"
    )

    # 先呈现相机画面，再让后台线程导入 Torch/Ultralytics 和预热 CUDA。
    startup_ret, startup_frame, _ = cam.read()
    if startup_ret and startup_frame is not None:
        cv2.imshow(window_name, startup_frame)
        cv2.waitKey(1)

    try:
        while True:
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                print("窗口已关闭。")
                break
            ret, frame, frame_time = cam.read()
            if ret and frame is not None:
                web_bridge.raw(frame, frame_time)
            if not ret or frame is None:
                continue

            # 始终处理采集线程提供的最新画面，不再因时间戳暂时未变化而暂停主循环。
            # ``CameraStream.read`` already returns an owned snapshot.  Keep
            # the reference for a possible calibration action instead of
            # copying the 1280x720 image a second time on every frame.
            frame_state['latest'] = frame

            # 计算主循环帧率
            now = time.time()
            render_clock = time.perf_counter()
            render_due = (
                is_recording
                or render_clock - display_state['last_t']
                >= 1.0 / DISPLAY_MAX_FPS
            )
            _UI_DRAW_STATE['allow_text'] = bool(render_due)
            set_text_render_enabled(render_due)
            loop_dt = now - fps_state['last_loop_t']
            fps_state['last_loop_t'] = now
            if loop_dt > 0:
                inst_loop_fps = 1.0 / loop_dt
                fps_state['loop_fps'] = 0.9 * fps_state['loop_fps'] + 0.1 * inst_loop_fps

            img_show = frame.copy()
            # Grayscale is only needed by CLAHE or while ArUco is still being
            # searched.  Once the stationary pool H is locked, skip this full
            # frame conversion on every iteration.
            img_gray = None

            if marker_roi_state['selecting'] and marker_roi_state['start'] is not None:
                marker_end = marker_roi_state['end'] or marker_roi_state['start']
                cv2.rectangle(
                    img_show,
                    marker_roi_state['start'],
                    marker_end,
                    (0, 165, 255),
                    2,
                )

            if (
                marker_roi_state['confirmed_bbox'] is not None
                and time.time() <= marker_roi_state['confirmed_until']
            ):
                confirm_x1, confirm_y1, confirm_x2, confirm_y2 = marker_roi_state['confirmed_bbox']
                confirm_cx, confirm_cy = marker_roi_state['confirmed_center']
                cv2.rectangle(
                    img_show, (confirm_x1, confirm_y1), (confirm_x2, confirm_y2),
                    UiPalette.SUCCESS, 2, cv2.LINE_AA
                )
                cv2.drawMarker(
                    img_show, (int(round(confirm_cx)), int(round(confirm_cy))),
                    UiPalette.SUCCESS, cv2.MARKER_CROSS, 14, 2, cv2.LINE_AA
                )
                draw_unicode_text(
                    img_show, "已自动找到黑色鱼体中心",
                    (confirm_x1, max(3, confirm_y1 - 22)),
                    14, UiPalette.SUCCESS, bold=True, stroke_width=1
                )

            if len(drawn_path_state['pixels']) > 1:
                path_pixels = np.asarray(
                    drawn_path_state['pixels'], dtype=np.int32
                ).reshape((-1, 1, 2))
                cv2.polylines(
                    img_show, [path_pixels], False, (255, 0, 255), 3,
                    cv2.LINE_AA
                )
                cv2.circle(
                    img_show, tuple(drawn_path_state['pixels'][-1]), 7,
                    (0, 255, 0), -1
                )

            if calib_state['overlay_dirty']:
                (
                    calib_state['overlay'],
                    calib_state['overlay_mask'],
                ) = build_calibration_overlay(
                    frame.shape, calib_state['pts_raw']
                )
                calib_state['overlay_dirty'] = False
            apply_calibration_overlay(
                img_show,
                calib_state['overlay'],
                calib_state['overlay_mask'],
            )

            # 动态 CLAHE 直方图均衡化控制（应对水面强反光）
            needs_gray = use_clahe or (
                calib_state['H'] is None and not calib_state['is_calibrating']
            )
            if needs_gray:
                img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if use_clahe:
                img_gray = clahe.apply(img_gray)

            corner_pixels = {}
            # Manual four-point calibration and automatic ArUco locking are
            # mutually exclusive.  Running both while the user is clicking
            # makes the UI appear to accept points while H is replaced by the
            # background detector.
            if calib_state['H'] is None and not calib_state['is_calibrating']:
                corners, ids, _ = detector.detectMarkers(img_gray)

                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(img_show, corners, ids)
                    ids_flat = ids.flatten()
                    for i in range(len(ids_flat)):
                        marker_id = int(ids_flat[i])
                        cx, cy = np.mean(corners[i][0][:, 0]), np.mean(corners[i][0][:, 1])

                        if marker_id in [MARKER_TL, MARKER_TR, MARKER_BL, MARKER_BR]:
                            corner_pixels[marker_id] = [cx, cy]

            # YOLO confirms identity; measured flow bridges brief detection gaps.
            reference_observation = reference_tracker.update(frame, frame_time)
            detection_bbox = reference_observation.metrics.get('fish_bbox')
            detection_confidence = reference_observation.confidence
            track_id = reference_observation.metrics.get('track_id')
            auv_id_detected = reference_observation.valid
            detector_count=reference_observation.metrics.get('detection_count',0)
            if detector_meter['time'] is None or detector_count<detector_meter['count']:
                detector_meter.update(time=frame_time,count=detector_count,hz=0.)
            elif frame_time-detector_meter['time']>=1.0:
                detector_meter['hz']=(detector_count-detector_meter['count'])/(frame_time-detector_meter['time'])
                detector_meter.update(time=frame_time,count=detector_count)
            detection_fps = detector_meter['hz']
            detection_status = {'algorithm':'YOLO_PT','ready':True,'loading':False,'error':None,'load_seconds':reference_tracker.load_seconds}
            detection_result = {'frame_time':frame_time,'pose_valid':False,'bbox':detection_bbox}
            detection_pose_valid = False
            pose_tail = pose_head = None
            auv_pixel = (
                list(reference_observation.position)
                if reference_observation.position is not None
                else None
            )
            frame_state['reference_position'] = (
                tuple(reference_observation.position)
                if reference_observation.position is not None else None
            )
            frame_state['reference_source'] = reference_observation.source
            frame_state['tail_marker_position'] = (
                reference_observation.metrics.get('tail_marker_position')
                if reference_observation.metrics else None
            )
            frame_state['tail_marker_bbox'] = (
                reference_observation.metrics.get('tail_marker_bbox',
                                                   reference_observation.metrics.get('bbox'))
                if reference_observation.metrics else None
            )
            frame_state['tail_marker_quality'] = float(
                reference_observation.metrics.get('tail_marker_quality', 0.0)
                if reference_observation.metrics else 0.0
            )
            body_yaw_state['tail_color_confidence'] = frame_state[
                'tail_marker_quality'
            ]

            # Keep the optional binary preview separate from detection and yaw.
            head_color_result = reference_tracker.head_result
            binary_mask_preview = (
                white_head_tracker.binary_preview(frame)
                if show_binary_preview else None
            )
            color_head_confidence = float(head_color_result.get('confidence', 0.0))
            color_head_pixel = head_color_result.get('position')
            body_yaw_state['color_head_source'] = head_color_result.get(
                'source', 'UNAVAILABLE'
            )
            # Store the current observation (including None) in diagnostics;
            # the tracker keeps its own internal state for the next ROI hint.
            body_yaw_state['color_head_pixel'] = color_head_pixel
            body_yaw_state['head_color_confidence'] = color_head_confidence
            body_yaw_state['detector_head_in_white'] = head_color_result.get(
                'head_hint_inside'
            )
            body_yaw_state['detector_head_distance_to_white_px'] = head_color_result.get(
                'head_hint_distance_px'
            )
            tail_color_pixel = frame_state.get('tail_marker_position')
            yaw_tail_pixel, yaw_head_pixel = tail_color_pixel, color_head_pixel
            selected_measurement_ready = reference_observation.valid
            color_raw_yaw_deg = raw_yaw_deg = None
            yaw_measurement_source = 'IMU_YAW'
            if reference_observation.valid:
                previous_id=body_yaw_state.get('track_id')
                if previous_id is not None and previous_id!=track_id:
                    imu_yaw.reset()
                    heading_state['world_unit_vector']=None
                    if drawn_path_state['active']:
                        drawn_path_state['active']=False;is_tracing_active=False
                        fish_comm.stop_now();current_status_text='TARGET LOST'
                body_yaw_state['track_id']=track_id
            bridge=globals().get('web_bridge')
            yaw_sample=bridge.get_yaw_sample() if bridge is not None else {'valid':False}
            yaw_result=imu_yaw.update(yaw_sample,time.time())
            body_yaw_state.update({
                'yaw_deg':yaw_result['yaw_deg'],'visual_yaw_deg':None,
                'fused_yaw_deg':yaw_result['yaw_deg'],'raw_yaw_deg':yaw_result['raw_yaw_deg'],
                'yaw_rate_rps':yaw_result['yaw_rate_rps'],'source':yaw_result['source'],
                'prediction_age_s':yaw_result['age_s'],'measurement_valid':yaw_result['measurement_valid'],
                'coordinate_space':'world','unit_vector':yaw_result['unit_vector'],
                'usable':bool(yaw_result['usable'] and calib_state['H'] is not None),
                'yaw_measurement_source':'IMU_YAW','tail_pixel':tail_color_pixel,
                'initialized':yaw_result['initialized'],'head_pixel':color_head_pixel,
                'world_yaw_rad':yaw_result['world_yaw_rad'],'sample_time':yaw_result['sample_time'],
                'last_frame_time':frame_time,'last_update_wall_t':time.monotonic(),
            })
            if yaw_result['usable']:
                heading_state['control_heading']=yaw_result['unit_vector']
                heading_state['control_heading_source']='IMU_YAW'
            # Draw the actual detector/flow box without background expansion.
            if show_detection_bbox_overlay and detection_bbox is not None:
                box_x1, box_y1, box_x2, box_y2 = [int(value) for value in detection_bbox]
                cv2.rectangle(
                    img_show, (box_x1, box_y1), (box_x2, box_y2),
                    (255, 180, 0), 1
                )
                detector_label = f"机器鱼 #{track_id if track_id is not None else '-'}　{detection_confidence * 100:.0f}%"
                draw_unicode_text(
                    img_show, detector_label,
                    (box_x1, max(3, box_y1 - 20)), 13,
                    (255, 180, 0), bold=True, stroke_width=1
                )

            if show_debug_overlay and reference_observation.valid:
                for label, point, color in [('BODY',tail_color_pixel,(0,180,255)),('WHITE',color_head_pixel,(255,255,255))]:
                    if point is not None:
                        pixel=tuple(int(round(v)) for v in point)
                        cv2.circle(img_show,pixel,5,color,2,cv2.LINE_AA)
                        draw_unicode_text(img_show,label,(pixel[0]+8,pixel[1]),11,color)

            # The green arrow is the calibrated IMU yaw, even when the fish is stationary.
            if (show_filtered_yaw_overlay and reference_observation.valid
                    and body_yaw_state['usable'] and body_yaw_state['unit_vector'] is not None
                    and calib_state['H'] is not None and calib_state['inv_H'] is not None):
                centre_world=cv2.perspectiveTransform(
                    np.float32([[reference_observation.position]]),calib_state['H'])[0,0]
                tip_world=centre_world+.22*np.asarray(body_yaw_state['unit_vector'],dtype=np.float32)
                arrow=cv2.perspectiveTransform(np.float32([[centre_world,tip_world]]),calib_state['inv_H'])[0]
                origin=tuple(int(round(v)) for v in arrow[0]);tip=tuple(int(round(v)) for v in arrow[1])
                cv2.arrowedLine(img_show,origin,tip,UiPalette.SUCCESS,2,cv2.LINE_AA,tipLength=.24)
                draw_unicode_text(img_show,f"IMU yaw {body_yaw_state['yaw_deg']:+.1f}°",
                                  (tip[0]+8,max(3,tip[1]-8)),12,UiPalette.SUCCESS)

            # ------------------------------------------------------------------
            # 目标平滑绘制与抗锯齿轨迹管理
            # ------------------------------------------------------------------
            if auv_pixel is not None:
                raw_cx, raw_cy = auv_pixel

                if pixel_state['smooth_x'] is None or pixel_state['smooth_y'] is None:
                    pixel_state['smooth_x'] = raw_cx
                    pixel_state['smooth_y'] = raw_cy
                else:
                    pixel_state['smooth_x'] = PIXEL_SMOOTH_ALPHA * raw_cx + (1 - PIXEL_SMOOTH_ALPHA) * pixel_state['smooth_x']
                    pixel_state['smooth_y'] = PIXEL_SMOOTH_ALPHA * raw_cy + (1 - PIXEL_SMOOTH_ALPHA) * pixel_state['smooth_y']

                cx = int(pixel_state['smooth_x'])
                cy = int(pixel_state['smooth_y'])

                source_colours = {
                    ReferenceSource.RIGID_BODY: UiPalette.SUCCESS,
                    ReferenceSource.YOLO_BODY: UiPalette.SUCCESS,
                    ReferenceSource.MARKER: UiPalette.SUCCESS,
                    ReferenceSource.PREDICTED: UiPalette.WARNING,
                    ReferenceSource.ESTIMATED_FALLBACK: UiPalette.WARNING,
                }
                reference_colour = source_colours.get(
                    reference_observation.source, UiPalette.DANGER
                )
                if show_color_aux_overlay:
                    cv2.rectangle(img_show, (cx - 15, cy - 15), (cx + 15, cy + 15), reference_colour, 2)
                    cv2.circle(img_show, (cx, cy), 4, reference_colour, -1)
                    id_text = (
                        f"{hud.SOURCE_NAMES.get(reference_observation.source, reference_observation.source)}　"
                        f"质量 {reference_observation.quality * 100:.0f}%"
                    )
                    draw_unicode_text(
                        img_show, id_text, (cx + 20, max(3, cy - 24)),
                        13, reference_colour, bold=True, stroke_width=1
                    )

                if len(trajectory) == 0:
                    trajectory.append((cx, cy, now))
                else:
                    last_pt = trajectory[-1]
                    dist_sq = (cx - last_pt[0]) ** 2 + (cy - last_pt[1]) ** 2
                    if dist_sq >= MIN_TRAJ_DIST_SQ:
                        trajectory.append((cx, cy, now))
            else:
                pixel_state['smooth_x'] = None
                pixel_state['smooth_y'] = None

            # Draw the swimming trail continuously, including frames where the
            # marker detector has a brief miss.  Each segment is composited
            # with an age-dependent alpha: recent motion is bright yellow,
            # then the trail smoothly fades and is discarded after the timeout.
            while trajectory:
                try:
                    oldest_age = now - float(trajectory[0][2])
                except (IndexError, TypeError, ValueError):
                    trajectory.popleft()
                    continue
                if oldest_age <= TRAJECTORY_MAX_AGE_S:
                    break
                trajectory.popleft()
            if len(trajectory) > 1:
                trail_layer = np.zeros_like(img_show)
                trail_alpha = np.zeros(img_show.shape[:2], dtype=np.uint8)
                fade_span = max(
                    1e-6, TRAJECTORY_MAX_AGE_S - TRAJECTORY_FADE_START_S
                )
                trajectory_points = tuple(trajectory)
                for previous, current in zip(
                    trajectory_points, trajectory_points[1:]
                ):
                    try:
                        age = max(0.0, now - float(current[2]))
                        if age <= TRAJECTORY_FADE_START_S:
                            segment_alpha = 1.0
                        else:
                            segment_alpha = np.clip(
                                (TRAJECTORY_MAX_AGE_S - age) / fade_span,
                                0.0, 1.0,
                            )
                        if segment_alpha <= 0.0:
                            continue
                        start = (int(previous[0]), int(previous[1]))
                        end = (int(current[0]), int(current[1]))
                        cv2.line(
                            trail_layer, start, end, (0, 255, 255), 2,
                            cv2.LINE_AA,
                        )
                        cv2.line(
                            trail_alpha, start, end,
                            int(round(255.0 * float(segment_alpha))), 2,
                            cv2.LINE_AA,
                        )
                    except (IndexError, TypeError, ValueError, OverflowError):
                        continue
                if np.any(trail_alpha):
                    alpha = trail_alpha.astype(np.float32) / 255.0
                    alpha = alpha[..., None]
                    img_show[:] = np.clip(
                        img_show.astype(np.float32) * (1.0 - alpha)
                        + trail_layer.astype(np.float32) * alpha,
                        0.0, 255.0,
                    ).astype(np.uint8)

            if heading_state['selecting'] and frame_state['tail_marker_position'] is not None:
                tail_x, tail_y = [
                    int(round(value))
                    for value in frame_state['tail_marker_position']
                ]
                cv2.drawMarker(
                    img_show, (tail_x, tail_y), UiPalette.PRIMARY,
                    cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA
                )
                draw_unicode_text(
                    img_show, "黑体中心已定位，请点击白色鱼头中心",
                    (min(frame.shape[1] - 235, tail_x + 12), max(3, tail_y - 24)),
                    13, UiPalette.PRIMARY, bold=True, stroke_width=1
                )

            # Endpoint provenance overlay.  These boxes are deliberately
            # drawn independently: white is the largest white component
            # selected *inside* the OpenCV fish box; orange is the accepted
            # orange contour used as the only valid tail source.  Seeing all
            # three boxes on the same frame makes cross-target associations
            # and near-coincident endpoints immediately observable.
            if show_debug_overlay:
                white_bbox = head_color_result.get('bbox')
                if white_bbox is not None and len(white_bbox) == 4:
                    wx1, wy1, wx2, wy2 = [int(round(value)) for value in white_bbox]
                    cv2.rectangle(
                        img_show, (wx1, wy1), (wx2, wy2),
                        (255, 255, 255), 2, cv2.LINE_AA,
                    )
                    draw_unicode_text(
                        img_show,
                        f"WHITE max area={float(head_color_result.get('area', 0.0)):.0f}px2",
                        (wx1, max(3, wy1 - 5)), 10,
                        (255, 255, 255), bold=True, stroke_width=1,
                    )
                orange_bbox = frame_state.get('tail_marker_bbox')
                if orange_bbox is not None and len(orange_bbox) == 4:
                    ox1, oy1, ox2, oy2 = [int(round(value)) for value in orange_bbox]
                    cv2.rectangle(
                        img_show, (ox1, oy1), (ox2, oy2),
                        (0, 165, 255), 2, cv2.LINE_AA,
                    )
                    draw_unicode_text(
                        img_show,
                        f"BLACK body area={float((reference_observation.metrics or {}).get(
                            'tail_marker_area',
                            (reference_observation.metrics or {}).get('area', 0.0),
                        ) or 0.0):.0f}px2",
                        (ox1, min(frame.shape[0] - 3, oy2 + 14)), 10,
                        (0, 165, 255), bold=True, stroke_width=1,
                    )

            if (
                heading_state['tail_point'] is not None
                and heading_state['head_point'] is not None
                and time.time() <= heading_state['confirmed_until']
            ):
                tail_point = tuple(int(round(value)) for value in heading_state['tail_point'])
                head_point = tuple(int(round(value)) for value in heading_state['head_point'])
                cv2.arrowedLine(
                    img_show, tail_point, head_point, UiPalette.PRIMARY,
                    3, cv2.LINE_AA, tipLength=0.18
                )
                draw_unicode_text(
                    img_show,
                    f"初始鱼头方向 {heading_state['angle_deg']:.1f}°",
                    (min(frame.shape[1] - 160, head_point[0] + 8), max(3, head_point[1] - 22)),
                    13, UiPalette.PRIMARY, bold=True, stroke_width=1
                )

            # 自动标定四个角点
            if POOL_SIZE_CONFIRMED and len(corner_pixels) == 4 and not calib_state['manual_locked'] and not calib_state['auto_locked']:
                src_pts = np.float32([
                    corner_pixels[MARKER_TL], corner_pixels[MARKER_TR],
                    corner_pixels[MARKER_BR], corner_pixels[MARKER_BL]
                ])
                previous = calib_state['auto_prev_points']
                if previous is not None and float(np.max(np.linalg.norm(src_pts - previous, axis=1))) <= 2.0:
                    calib_state['auto_stable_count'] += 1
                else:
                    calib_state['auto_stable_count'] = 1
                calib_state['auto_prev_points'] = src_pts.copy()

                if calib_state['auto_stable_count'] >= 15:
                    homography, error = build_calibration_homography(
                        src_pts, cam.real_width, cam.real_height
                    )
                    if error:
                        print(f"自动标定未锁定：{error}")
                        calib_state['auto_stable_count'] = 0
                    else:
                        calib_state['H'] = homography
                        calib_state['inv_H'] = np.linalg.inv(homography)
                        calib_state['overlay_dirty'] = True
                        calib_state['auto_locked'] = True
                        # Keep the image points used for H so the dimension
                        # overlay remains visible after auto-calibration locks.
                        calib_state['pts_raw'] = src_pts.tolist()
                        calib_state['pts_disp'] = [
                            (int(round(point[0])), int(round(point[1])))
                            for point in src_pts
                        ]
                        try:
                            save_pool_calibration(
                                POOL_CALIBRATION_PATH,
                                src_pts,
                                cam.real_width,
                                cam.real_height,
                            )
                        except (OSError, ValueError) as save_error:
                            print(f"自动场地标定已生效，但保存失败：{save_error}")
                        else:
                            print(f"自动场地标定已保存：{POOL_CALIBRATION_PATH}")
                        print("自动标定成功！四个场地角点已稳定并锁定。")
            elif len(corner_pixels) < 4:
                calib_state['auto_stable_count'] = 0
                calib_state['auto_prev_points'] = None

            # 解算物理坐标与平滑滤波
            current_x, current_y = None, None
            direct_marker_world_position = None
            control_reference_available = (
                reference_observation.source in (ReferenceSource.RIGID_BODY, ReferenceSource.YOLO_BODY)
                or reference_observation.source == ReferenceSource.PREDICTED
            )
            if (
                calib_state['H'] is not None
                and auv_pixel is not None
                and control_reference_available
            ):
                pt_real = cv2.perspectiveTransform(
                    np.float32([[[auv_pixel[0], auv_pixel[1]]]]), calib_state['H']
                )
                raw_x, raw_y = float(pt_real[0][0][0]), float(pt_real[0][0][1])
                if reference_observation.source in (ReferenceSource.RIGID_BODY, ReferenceSource.YOLO_BODY):
                    direct_marker_world_position = (raw_x, raw_y)

                if pos_state['smooth_x'] is None or pos_state['smooth_y'] is None:
                    pos_state['smooth_x'] = raw_x
                    pos_state['smooth_y'] = raw_y
                elif pos_state['frame_time'] != frame_time:
                    a_pos = POS_SMOOTHING_ALPHA
                    pos_state['smooth_x'] = a_pos * raw_x + (1 - a_pos) * pos_state['smooth_x']
                    pos_state['smooth_y'] = a_pos * raw_y + (1 - a_pos) * pos_state['smooth_y']
                pos_state['frame_time'] = frame_time

                current_x = float(pos_state['smooth_x'])
                current_y = float(pos_state['smooth_y'])
            else:
                pos_state['smooth_x'] = None
                pos_state['smooth_y'] = None
                pos_state['frame_time'] = None

            # 转圈测量只接收黑色鱼体的直接观测；PREDICTED/OpenCV 降级位置
            # 不进入圆拟合，避免“看起来连续”的假轨迹污染半径。
            if (
                turn_calibration_session.active
                and direct_marker_world_position is not None
            ):
                turn_calibration_session.add(
                    direct_marker_world_position, frame_time
                )

            # 短窗口轨迹拟合速度：比逐帧差分更能抑制鱼尾摆动和单帧尖峰。
            speed = None
            direction_deg = None
            velocity = None
            if (current_x is not None and current_y is not None and
                    reference_observation.usable_for_pwm_speed and
                    reference_observation.source in (ReferenceSource.RIGID_BODY, ReferenceSource.YOLO_BODY)):
                velocity = velocity_estimator.update(
                    (current_x, current_y), frame_time
                )
                if velocity is not None:
                    velocity_state['vx_smooth'] = float(velocity[0])
                    velocity_state['vy_smooth'] = float(velocity[1])
                    speed = float(np.linalg.norm(velocity))
                    direction_deg=body_yaw_state.get('yaw_deg')
            else:
                velocity_estimator.reset()
                velocity_state['vx_smooth'] = 0.0
                velocity_state['vy_smooth'] = 0.0

            control_x, control_y = current_x, current_y
            if current_x is not None and current_y is not None and velocity is not None:
                compensated_position = compensate_camera_latency(
                    (current_x, current_y),
                    velocity,
                    CAMERA_LATENCY_S,
                    CAMERA_LATENCY_MAX_PREDICTION_M,
                )
                control_x = float(compensated_position[0])
                control_y = float(compensated_position[1])

            # ------------------------------------------------------------------
            # 1. 处理 UI 传来的控制指令 (急停/停止)
            # ------------------------------------------------------------------
            cmd_msg = tablet_server.get_next_command()
            if cmd_msg:
                cmd = cmd_msg.get("cmd", "")
                if cmd in ["stop", "emergency_stop"]:
                    current_status_text = "EMERGENCY STOP" if cmd == "emergency_stop" else "STOPPED"
                    is_tracing_active = False
                    motion_prior_state['last_r_cmd_rps'] = 0.0
                    fish_comm.send_command(cmd)

            # ------------------------------------------------------------------
            # 2. 处理 UI 传来的轨迹规划，仅在模式改变时下发一次模式切换
            # ------------------------------------------------------------------
            traj_msg = tablet_server.get_latest_trajectory()
            if traj_msg:
                plan_pts = traj_msg.get("points", [])
                if len(plan_pts) > 1 and calib_state['H'] is not None:
                    try:
                        inv_H = calib_state['inv_H']
                        pts_phys = np.float32([[pt] for pt in plan_pts])
                        pts_pix = cv2.perspectiveTransform(pts_phys, inv_H)

                        for k in range(1, len(pts_pix)):
                            pt1 = (int(pts_pix[k-1][0][0]), int(pts_pix[k-1][0][1]))
                            pt2 = (int(pts_pix[k][0][0]), int(pts_pix[k][0][1]))
                            cv2.line(img_show, pt1, pt2, (255, 0, 255), 2, cv2.LINE_AA)
                            cv2.circle(img_show, pt1, 3, (0, 255, 255), -1)
                    except Exception:
                        pass

            # 平板误差仅保留显示；本机鼠标轨迹独占推进控制。
            err_msg = tablet_server.get_latest_tracking_error()

            # 本机绘制轨迹：START 时已完成米制等距采样；这里更新单向
            # 路径进度、离轨自适应前视点和鱼体坐标左右误差。
            local_path_error = None
            if drawn_path_state['active']:
                if calib_state['H'] is None or len(drawn_path_state['pixels']) < 2:
                    drawn_path_state['active'] = False
                    is_tracing_active = False
                    path_guidance.clear()
                    current_status_text = "UNCALIBRATED"
                elif current_x is None or current_y is None:
                    if drawn_path_state['lost_since'] is None:
                        drawn_path_state['lost_since'] = time.monotonic()
                    # Keep the route through a brief miss, but send zero thrust:
                    # missing pixels are never fabricated speed measurements.
                    if now-drawn_path_state['last_control_t'] >= VISION_UDP_PERIOD_S:
                        fish_comm.process_tracking_error(0.,0.,0.,speed_mps=0.,
                            brake_request=True,steering_demand=0.,speed_target_mps=0.,
                            observed_at=float(frame_time),feedback_valid=False)
                        drawn_path_state['last_control_t'] = now
                    current_status_text = 'TARGET WAIT'
                    if time.monotonic()-drawn_path_state['lost_since'] >= 0.6:
                        drawn_path_state['active'] = False
                        is_tracing_active = False
                        fish_comm.stop_now()
                        current_status_text = 'TARGET LOST'
                elif not path_guidance.prepared:
                    drawn_path_state['active'] = False
                    is_tracing_active = False
                    current_status_text = "PATH INVALID"
                    print("轨迹引导器尚未准备，请重新点击【启动】。")
                elif not body_yaw_state['usable']:
                    fish_comm.process_tracking_error(0.,0.,0.,brake_request=True,
                        speed_target_mps=0.,observed_at=float(frame_time),feedback_valid=False)
                    current_status_text='IMU YAW WAIT'
                else:
                    drawn_path_state['lost_since'] = None
                    body_heading, body_yaw_rate, body_heading_source = (
                        select_yaw_for_control(now)
                    )
                    if globals().get('web_bridge') is not None:
                        path_guidance.cruise_speed_mps = web_bridge.cruise_speed_mps
                        path_guidance.min_speed_target_mps = min(CONTROL['guidance']['min_speed_mps'], web_bridge.cruise_speed_mps)
                    local_path_error = path_guidance.update(
                        (control_x, control_y),
                        frame_time,
                        # 初版循迹：人工尾→头方向作为启动航向，黑体中心
                        # 稳定移动后由短窗位移 course 平滑跟随真实转弯。
                        # 白色区域/OpenCV 仍负责端点验证和显示，不会因一帧
                        # yaw 丢失而把已经启动的路径追踪强制停掉。
                        speed_mps=(speed if speed is not None else 0.0),
                        velocity=velocity,
                        body_heading=body_heading,
                        body_yaw_rate_rps=body_yaw_rate,
                    )
                    # PathGuidance reports VISUAL_YAW for both a fresh visual
                    # heading and the short hold window.  Preserve the actual
                    # source in the runtime record for diagnostics.
                    local_path_error['control_heading_source'] = (
                        body_heading_source
                    )
                    if body_heading_source != yaw_control_state.get(
                        'reported_source'
                    ):
                        yaw_value = body_yaw_state.get('yaw_deg')
                        age_value = body_yaw_state.get('prediction_age_s')
                        print(
                            f"[HEADING] control={body_heading_source} "
                            f"yaw={yaw_value if yaw_value is not None else '-'} "
                            f"age={age_value if age_value is not None else '-'}"
                        )
                        yaw_control_state['reported_source'] = (
                            body_heading_source
                        )
                    drawn_path_state['segment'] = local_path_error['seg_index']
                    motion_prior_state.update({
                        'last_t': float(frame_time),
                        'last_r_cmd_rps': float(local_path_error.get('r_cmd_rps', 0.0)),
                        'last_heading_rad': math.radians(
                            float(body_yaw_state['yaw_deg'])
                        ) if body_yaw_state.get('yaw_deg') is not None else None,
                        'last_fused_yaw_deg': body_yaw_state.get('yaw_deg'),
                    })

                    # 引导几何属于算法模块；像素绘制仍由主 UI 编排。
                    try:
                        inv_h = calib_state['inv_H']
                        psi_ref_rad = float(local_path_error['psi_ref_rad'])
                        target_heading_world = np.asarray(
                            (math.cos(psi_ref_rad), -math.sin(psi_ref_rad)),
                            dtype=np.float32,
                        )
                        target_heading_tip = (
                            np.asarray((control_x, control_y), dtype=np.float32)
                            + target_heading_world * 0.22
                        )
                        guide_world = np.float32([[
                            local_path_error['lookahead_point'],
                            local_path_error['closest_point'],
                            target_heading_tip,
                        ]])
                        guide_pixels = cv2.perspectiveTransform(
                            guide_world, inv_h
                        )[0]
                        lookahead_pixel = tuple(
                            int(round(value)) for value in guide_pixels[0]
                        )
                        closest_pixel = tuple(
                            int(round(value)) for value in guide_pixels[1]
                        )
                        target_heading_pixel = tuple(
                            int(round(value)) for value in guide_pixels[2]
                        )
                        tail_pixel = tuple(
                            int(round(value)) for value in reference_observation.position
                        )
                        cv2.line(
                            img_show, tail_pixel, lookahead_pixel,
                            (255, 255, 0), 2, cv2.LINE_AA
                        )
                        cv2.circle(img_show, lookahead_pixel, 7, (255, 255, 0), 2)
                        cv2.circle(img_show, closest_pixel, 4, (255, 0, 255), -1)
                        # Target yaw is the path reference (tangent plus
                        # cross-track correction).  The actual body yaw is
                        # drawn separately in green by the endpoint overlay.
                        cv2.arrowedLine(
                            img_show, tail_pixel, target_heading_pixel,
                            UiPalette.PRIMARY, 3, cv2.LINE_AA, tipLength=0.20
                        )
                        draw_unicode_text(
                            img_show,
                            f"TARGET yaw={imu_yaw.relative_target_deg(psi_ref_rad):+.1f}deg",
                            (target_heading_pixel[0] + 8,
                             max(3, target_heading_pixel[1] - 8)),
                            11, UiPalette.PRIMARY, bold=True, stroke_width=1,
                        )
                    except (ValueError, np.linalg.LinAlgError, cv2.error):
                        pass

                    if local_path_error['settled']:
                        fish_comm.stop_now()
                        drawn_path_state['active'] = False
                        is_tracing_active = False
                        reset_yaw_control_state()
                        current_status_text = "ARRIVED"
                        print(
                            "已进入终点 0.20 m 停止圈，"
                            "已发送渐变 STOP。"
                        )
                    elif now - drawn_path_state['last_control_t'] >= VISION_UDP_PERIOD_S:
                        fish_comm.process_tracking_error(
                            local_path_error['x_error_m'],
                            local_path_error['along_m'],
                            local_path_error['drive_distance_m'],
                            speed_mps=(speed if speed is not None else 0.0),
                            curve_severity=local_path_error['curve_severity'],
                            brake_request=local_path_error['brake_request'],
                            cross_track_m=local_path_error['cross_track_m'],
                            heading_error_deg=local_path_error['heading_error_deg'],
                            path_curvature_per_m=(
                                local_path_error['path_curvature_per_m']
                            ),
                            steering_demand=local_path_error['steering_demand'],
                            target_yaw_rad=local_path_error['psi_ref_rad'],
                            body_yaw_rad=local_path_error['psi_body_rad'],
                            yaw_observed_at=body_yaw_state.get('sample_time'),
                            curve_feedforward=local_path_error['curve_feedforward'],
                            speed_target_mps=local_path_error['speed_target_mps'],
                            observed_at=float(frame_time),
                            feedback_valid=(calib_state['H'] is not None and speed is not None and
                                reference_observation.usable_for_pwm_speed and
                                reference_observation.source in (ReferenceSource.RIGID_BODY, ReferenceSource.YOLO_BODY)),
                        )
                        drawn_path_state['last_control_t'] = now
                        is_tracing_active = True
                        current_status_text = (
                            "BIONIC BRAKING"
                            if local_path_error['brake_request']
                            else "BIONIC TRACKING"
                        )

            # Wi-Fi authorization may be revoked by the sender between frames.
            # Do not show an active path after rejection or manual takeover.
            if drawn_path_state['active'] and not getattr(fish_comm, 'authorized', True):
                drawn_path_state['active'] = False
                is_tracing_active = False
                reset_yaw_control_state()
                current_status_text = "CONTROL LOST"

            # 获取当前实时通信频率
            tab_tx_hz, tab_rx_hz = tablet_server.get_comm_fps()
            mcu_hz = fish_comm.get_mcu_hz()

            # ==============================================================================
            # 统一中文 HUD：紧凑半透明卡片，不再用超大英文覆盖画面。
            display_error = local_path_error if local_path_error is not None else err_msg
            hud_prompt = None
            if turn_calibration_session.active:
                hud_prompt = (
                    "转圈测量：请在网页保持 BIONIC 满左或满右，"
                    f"至少半圈；当前 {turn_calibration_session.sample_count} 点，"
                    "再点【转圈测量】完成"
                )
            elif calib_state['is_calibrating']:
                hud_prompt = (
                    f"场地标定 {len(calib_state['pts_raw'])}/4："
                    "依次点击 左上 → 右上 → 右下 → 左下"
                )
            elif marker_roi_state['selecting']:
                hud_prompt = "框选机器鱼：框住完整机器鱼，程序自动寻找白色鱼头与黑色鱼体；右键取消"
            elif heading_state['selecting']:
                hud_prompt = "鱼头方向：黑体中心自动取点，请点击鱼头中心；右键取消"
            elif (
                calib_state['H'] is not None
                and heading_state['world_unit_vector'] is None
            ):
                hud_prompt = "请连接机器鱼 IMU，再点击【鱼头方向】建立 0 yaw 基准"
            elif detection_status['loading']:
                hud_prompt = "OpenCV / CUDA 正在后台加载，窗口与标定功能可先使用"
            elif detection_status['error']:
                hud_prompt = f"OpenCV 加载失败：{detection_status['error']}"

            if show_debug_overlay and (
                reference_observation.search_roi is not None
                and reference_observation.source == ReferenceSource.INVALID
            ):
                sx1, sy1, sx2, sy2 = reference_observation.search_roi
                cv2.rectangle(
                    img_show, (sx1, sy1), (sx2, sy2),
                    UiPalette.WARNING, 1, cv2.LINE_AA
                )

            hud.draw(
                img_show,
                source=reference_observation.source,
                status=current_status_text,
                quality=reference_observation.quality,
                pwm_ready=reference_observation.usable_for_pwm_speed,
                position=(current_x, current_y) if current_x is not None and current_y is not None else None,
                speed=speed,
                direction_deg=direction_deg,
                gate_reasons=reference_observation.gate_reasons,
                loop_fps=fps_state['loop_fps'],
                camera_fps=cam.measured_fps,
                detector_fps=detection_fps,
                exposure=cam.exposure_val,
                tablet_tx_hz=tab_tx_hz,
                tablet_rx_hz=tab_rx_hz,
                mcu_hz=mcu_hz,
                tracking_error=display_error,
                prompt=hud_prompt,
            )

            if is_recording:
                cv2.circle(img_show, (frame.shape[1] - 22, 107), 5, UiPalette.DANGER, -1)
                draw_unicode_text(
                    img_show, "录像中", (frame.shape[1] - 78, 98),
                    13, UiPalette.DANGER, bold=True
                )

            # ==============================================================================
            # 构建 JSON 数据并推送/保存完整的画面
            # ==============================================================================
            guidance_json = None
            if local_path_error is not None:
                guidance_json = {
                    "body_yaw_deg": body_yaw_state.get('yaw_deg'),
                    "target_yaw_deg": imu_yaw.relative_target_deg(float(local_path_error['psi_ref_rad'])),
                    "actual_steering_percent": getattr(fish_comm,'yaw_status',{}).get('steering_percent'),
                    "x_error_m": round(float(local_path_error['x_error_m']), 4),
                    "body_left_m": round(float(local_path_error['body_left_m']), 4),
                    "heading_error_deg": round(
                        float(local_path_error['heading_error_deg']), 2
                    ),
                    "steering_error_deg": round(
                        float(local_path_error['steering_error_deg']), 2
                    ),
                    "curve_severity": round(
                        float(local_path_error['curve_severity']), 3
                    ),
                    "path_curve_severity": round(
                        float(local_path_error['path_curve_severity']), 3
                    ),
                    "path_curve_angle_deg": round(
                        float(local_path_error['path_curve_angle_deg']), 2
                    ),
                    "path_curvature_per_m": round(
                        float(local_path_error['path_curvature_per_m']), 3
                    ),
                    "curve_feedforward": round(
                        float(local_path_error['curve_feedforward']), 3
                    ),
                    "heading_feedback": round(
                        float(local_path_error['heading_feedback']), 3
                    ),
                    "cross_track_feedback": round(
                        float(local_path_error['cross_track_feedback']), 3
                    ),
                    "steering_demand": round(
                        float(local_path_error['steering_demand']), 3
                    ),
                    "psi_path_deg": round(
                        math.degrees(float(local_path_error['psi_path_rad'])), 2
                    ),
                    "psi_body_deg": round(
                        math.degrees(float(local_path_error['psi_body_rad'])), 2
                    ),
                    "psi_ref_deg": round(
                        math.degrees(float(local_path_error['psi_ref_rad'])), 2
                    ),
                    "r_cmd_rps": round(float(local_path_error['r_cmd_rps']), 3),
                    "cross_track_rate_mps": round(
                        float(local_path_error['cross_track_rate_mps']), 4
                    ),
                    "speed_target_mps": round(
                        float(local_path_error['speed_target_mps']), 4
                    ),
                    "turn_radius_m": round(
                        float(local_path_error['turn_radius_m']), 4
                    ),
                    "turn_radius_source": local_path_error[
                        'turn_radius_source'
                    ],
                    "cross_track_m": round(
                        float(local_path_error['cross_track_m']), 4
                    ),
                    "signed_cross_track_error_m": round(
                        float(local_path_error['signed_cross_track_error_m']), 4
                    ),
                    "off_path_ratio": round(
                        float(local_path_error['off_path_ratio']), 3
                    ),
                    "control_severity": round(
                        float(local_path_error['control_severity']), 3
                    ),
                    "curve_direction": local_path_error['curve_direction'],
                    "lookahead_m": round(
                        float(local_path_error['lookahead_m']), 3
                    ),
                    "brake_request": bool(local_path_error['brake_request']),
                    "heading_source": local_path_error['heading_source'],
                    "control_heading_source": local_path_error.get(
                        'control_heading_source',
                        local_path_error['heading_source'],
                    ),
                    "heading": [
                        round(float(value), 5)
                        for value in local_path_error['heading']
                    ],
                    "lookahead": [
                        round(float(value), 4)
                        for value in local_path_error['lookahead_point']
                    ],
                    "progress_m": round(float(local_path_error['along_m']), 4),
                    "remaining_m": round(float(local_path_error['dist_m']), 4),
                    "drive_distance_m": round(
                        float(local_path_error['drive_distance_m']), 4
                    ),
                    "end_distance_m": round(
                        float(local_path_error['end_distance_m']), 4
                    ),
                    "segment": int(local_path_error['seg_index']),
                }
            json_data = {
                "timestamp": frame_time,
                "pool": {"dimensions_confirmed": POOL_SIZE_CONFIRMED,
                         "width_m": PHYSICAL_WIDTH if POOL_SIZE_CONFIRMED else None,
                         "height_m": PHYSICAL_HEIGHT if POOL_SIZE_CONFIRMED else None},
                "tracking": {
                    "active": bool(drawn_path_state['active']),
                    "status": current_status_text,
                    "control_error": getattr(fish_comm, 'last_error', ''),
                },
                "speed_loop": dict(getattr(fish_comm, 'speed_status', {})),
                "yaw_loop": dict(getattr(fish_comm, 'yaw_status', {})),
                "track_id": track_id,
                "auv_detected": auv_id_detected,
                "pixel": {
                    "u": float(auv_pixel[0]) if auv_pixel else None,
                    "v": float(auv_pixel[1]) if auv_pixel else None
                },
                "reference": reference_observation.to_dict(),
                "initial_heading": {
                    "calibrated": heading_state['angle_deg'] is not None,
                    "tail_pixel": heading_state['tail_point'],
                    "head_pixel": heading_state['head_point'],
                    "angle_deg": heading_state['angle_deg'],
                    "world_unit": heading_state['world_unit_vector'],
                },
                "body_yaw": {
                    "yaw_deg": body_yaw_state['yaw_deg'],
                    "world_yaw_rad": body_yaw_state.get('world_yaw_rad'),
                    "visual_yaw_deg": body_yaw_state['visual_yaw_deg'],
                    "fused_yaw_deg": body_yaw_state['fused_yaw_deg'],
                    "motion_prior_deg": body_yaw_state['motion_prior_deg'],
                    "motion_gate_applied": bool(
                        body_yaw_state['motion_gate_applied']
                    ),
                    "motion_gate_count": int(motion_prior_state['gate_count']),
                    "raw_yaw_deg": body_yaw_state['raw_yaw_deg'],
                    "yaw_rate_rps": body_yaw_state['yaw_rate_rps'],
                    "source": body_yaw_state['source'],
                    "prediction_age_s": body_yaw_state['prediction_age_s'],
                    "usable": bool(body_yaw_state['usable']),
                    "measurement_valid": bool(body_yaw_state.get('measurement_valid',False)),
                    "initialized": bool(body_yaw_state.get('initialized',False)),
                    "turn_limited": body_yaw_state.get('turn_limited',False),
                    "coordinate_space": body_yaw_state.get('coordinate_space'),
                    "measurement_rejected": bool(
                        body_yaw_state['measurement_rejected']
                    ),
                    "polarity_corrected": bool(
                        body_yaw_state['polarity_corrected']
                    ),
                    "reacquire_pending": bool(
                        body_yaw_state['reacquire_pending']
                    ),
                    "reacquire_samples": int(
                        body_yaw_state['reacquire_samples']
                    ),
                    "detector_raw_yaw_deg": body_yaw_state['detector_raw_yaw_deg'],
                    "color_raw_yaw_deg": body_yaw_state['color_raw_yaw_deg'],
                    "yaw_measurement_source": body_yaw_state[
                        'yaw_measurement_source'
                    ],
                    "color_tail_pixel": body_yaw_state['color_tail_pixel'],
                    "color_head_pixel": body_yaw_state['color_head_pixel'],
                    "color_head_source": body_yaw_state['color_head_source'],
                    "detector_head_in_white": body_yaw_state['detector_head_in_white'],
                    "detector_head_distance_to_white_px": body_yaw_state[
                        'detector_head_distance_to_white_px'
                    ],
                    "filter_frame_time": body_yaw_state['last_frame_time'],
                    "head_color_confidence": round(
                        float(body_yaw_state['head_color_confidence']), 4
                    ),
                    "white_dynamic_value_min": body_yaw_state[
                        'white_dynamic_value_min'
                    ],
                    "tail_color_confidence": round(
                        float(body_yaw_state['tail_color_confidence']), 4
                    ),
                    "polarity_calibrated": bool(
                        body_yaw_state['polarity_calibrated']
                    ),
                    "tail_pixel": body_yaw_state['tail_pixel'],
                    "head_pixel": body_yaw_state['head_pixel'],
                    "pose_rejection_reason": body_yaw_state['pose_rejection_reason'],
                },
                "turn_calibration": {
                    "active": turn_calibration_session.active,
                    "samples": turn_calibration_session.sample_count,
                    "saved": {
                        direction: fit.to_dict()
                        for direction, fit in turn_calibration_results.items()
                    },
                },
                "guidance": guidance_json,
                "detection_bbox": [round(float(value), 2) for value in detection_bbox] if detection_bbox else None,
                "physical": {
                    "x": round(current_x, 4) if current_x is not None else None,
                    "y": round(current_y, 4) if current_y is not None else None,
                    "control_x": round(control_x, 4) if control_x is not None else None,
                    "control_y": round(control_y, 4) if control_y is not None else None,
                    "camera_latency_s": CAMERA_LATENCY_S,
                },
                "velocity": {
                    "vx": float(round(velocity_state['vx_smooth'], 4)) if speed is not None else None,
                    "vy": float(round(velocity_state['vy_smooth'], 4)) if speed is not None else None,
                    "speed": float(round(speed, 4)) if speed is not None else None,
                    "direction_deg": float(round(direction_deg, 2)) if direction_deg is not None else None
                },
                "calibrated": calib_state['H'] is not None,
                "system": {
                    "loop_fps": round(fps_state['loop_fps'], 1),
                    "camera_fps": round(cam.measured_fps, 1),
                    "detector_fps": round(detection_fps, 1),
                    "detection_confidence": round(detection_confidence, 3),
                    "detector_ready": bool(detection_status['ready']),
                    "detector_loading": bool(detection_status['loading']),
                    "detector_load_seconds": detection_status['load_seconds'],
                    "clahe_enabled": use_clahe,
                    "exposure": cam.exposure_val,
                    "tablet_tx_hz": tab_tx_hz,
                    "tablet_rx_hz": tab_rx_hz,
                    "mcu_hz": mcu_hz,
                    "udp_tx_hz": mcu_hz,
                    "udp_period_s": VISION_UDP_PERIOD_S,
                }
            }

            if show_binary_preview and binary_mask_preview is not None:
                # Replace the camera view with the full-frame binary mask. The
                # toolbar is drawn afterwards, so threshold adjustment remains
                # available while this diagnostic mode is active.
                img_show = cv2.cvtColor(binary_mask_preview, cv2.COLOR_GRAY2BGR)
                draw_unicode_text(
                    img_show,
                    f"全画面二值化 V={toolbar.binary_threshold}",
                    (12, 120),
                    14,
                    UiPalette.PRIMARY,
                    bold=True,
                )

            toolbar.draw(img_show, active={
                'START': drawn_path_state['active'],
                'STOP': current_status_text in ('STOPPED', 'EMERGENCY STOP'),
                'MARKER_ROI': marker_roi_state['selecting'],
                'HEAD_DIRECTION': heading_state['selecting'],
                'POOL_CALIB': calib_state['is_calibrating'],
                'TURN_CALIB': turn_calibration_session.active,
                'RECORD': is_recording,
                'CLAHE': use_clahe,
                'DEBUG': show_debug_overlay,
            })

            json_data['detector'] = detection_status
            tablet_server.send(json_data)
            if render_due:
                mjpeg_server.update(img_show)

            if is_recording and recorder is not None and frame_time != last_recorded_frame_time:
                last_recorded_frame_time = frame_time
                endpoint_distance = (math.hypot(color_head_pixel[0] - tail_color_pixel[0],
                                               color_head_pixel[1] - tail_color_pixel[1])
                                     if color_head_pixel is not None and tail_color_pixel is not None else None)
                csv_row = [
                    f"{frame_time:.4f}",
                    f"{auv_pixel[0]:.2f}" if auv_pixel else "",
                    f"{auv_pixel[1]:.2f}" if auv_pixel else "",
                    f"{current_x:.4f}" if current_x is not None else "",
                    f"{current_y:.4f}" if current_y is not None else "",
                    f"{body_yaw_state['yaw_deg']:.3f}"
                    if body_yaw_state['yaw_deg'] is not None else "",
                    f"{body_yaw_state['raw_yaw_deg']:.3f}"
                    if body_yaw_state['raw_yaw_deg'] is not None else "",
                    body_yaw_state['source'],
                    int(bool(body_yaw_state['usable'])),
                    f"{body_yaw_state['detector_raw_yaw_deg']:.3f}"
                    if body_yaw_state['detector_raw_yaw_deg'] is not None else "",
                    f"{body_yaw_state['color_raw_yaw_deg']:.3f}"
                    if body_yaw_state['color_raw_yaw_deg'] is not None else "",
                    body_yaw_state['color_head_source'],
                    f"{body_yaw_state['head_color_confidence']:.3f}",
                    f"{body_yaw_state['tail_color_confidence']:.3f}",
                    f"{color_head_pixel[0]:.2f}" if color_head_pixel else "",
                    f"{color_head_pixel[1]:.2f}" if color_head_pixel else "",
                    f"{tail_color_pixel[0]:.2f}" if tail_color_pixel else "",
                    f"{tail_color_pixel[1]:.2f}" if tail_color_pixel else "",
                    int(bool(detection_pose_valid)),
                    detection_result.get('pose_rejection_reason') or '',
                    ';'.join(
                        f"{float(value):.1f}" for value in detection_bbox
                    ) if detection_bbox is not None and len(detection_bbox) == 4 else '',
                    ';'.join(
                        f"{float(value):.1f}" for value in head_color_result.get('bbox', ())
                    ) if head_color_result.get('bbox') is not None else '',
                    f"{float(head_color_result.get('area', 0.0) or 0.0):.1f}",
                    ';'.join(
                        f"{float(value):.1f}" for value in frame_state.get('tail_marker_bbox', ())
                    ) if frame_state.get('tail_marker_bbox') is not None else '',
                    f"{float((reference_observation.metrics or {}).get(
                        'tail_marker_area',
                        (reference_observation.metrics or {}).get('area', 0.0),
                    ) or 0.0):.1f}",
                    f"{float(endpoint_distance):.1f}" if endpoint_distance is not None else '',
                    f"{body_yaw_state['visual_yaw_deg']:.3f}"
                    if body_yaw_state['visual_yaw_deg'] is not None else "",
                    f"{body_yaw_state['fused_yaw_deg']:.3f}"
                    if body_yaw_state['fused_yaw_deg'] is not None else "",
                    f"{body_yaw_state['motion_prior_deg']:.3f}"
                    if body_yaw_state['motion_prior_deg'] is not None else "",
                    int(bool(body_yaw_state['motion_gate_applied'])),
                    body_yaw_state['white_dynamic_value_min']
                    if body_yaw_state['white_dynamic_value_min'] is not None else "",
                    head_color_result.get('tail_distance_gate_px')
                    if head_color_result.get('tail_distance_gate_px') is not None else "",
                    int(head_color_result.get('rejected_components', 0) or 0),
                    (int(bool(body_yaw_state['detector_head_in_white']))
                     if body_yaw_state['detector_head_in_white'] is not None else ""),
                    body_yaw_state['detector_head_distance_to_white_px']
                    if body_yaw_state['detector_head_distance_to_white_px'] is not None else "",
                ]
                if not recorder.submit(img_show, csv_row) and recorder.error:
                    print('[RECORD ERROR] '+recorder.error, flush=True)
                    is_recording = False
                    recorder.close()
                    recorder = None

            _t3 = time.perf_counter()
            if render_due:
                cv2.imshow(window_name, img_show)
                display_state['last_t'] = _t3

            web_bridge.dispatch(mouse_callback, pending_ui_actions)
            # 键盘按键响应
            key = cv2.waitKey(1) & 0xFF
            keyboard_actions = {
                ord('q'): 'EXIT',
                ord(' '): 'STOP',
                ord('['): 'EXP_DOWN',
                ord(']'): 'EXP_UP',
                ord('a'): 'RECORD',
                ord('b'): 'SNAPSHOT',
                ord('t'): 'POOL_CALIB',
                ord('c'): 'CLAHE',
                ord('m'): 'MARKER_ROI',
                ord('d'): 'DEBUG',
                ord('v'): 'BINARY',
            }
            if key in (10, 13):
                pending_ui_actions.append('START')
            elif key in keyboard_actions:
                pending_ui_actions.append(keyboard_actions[key])

            exit_requested = False
            while pending_ui_actions:
                action_item = pending_ui_actions.popleft()
                if isinstance(action_item, tuple):
                    action, action_payload = action_item
                else:
                    action, action_payload = action_item, None

                if action == 'EXIT':
                    print("\n已点击退出。")
                    exit_requested = True
                    break

                if action == 'STOP':
                    drawn_path_state['active'] = False
                    is_tracing_active = False
                    current_status_text = "STOPPED"
                    reset_yaw_control_state()
                    fish_comm.stop_now()

                elif action == 'CLEAR_PATH':
                    if drawn_path_state['active']:fish_comm.stop_now()
                    drawn_path_state['pixels'].clear()
                    drawn_path_state['drawing'] = False
                    drawn_path_state['active'] = False
                    drawn_path_state['segment'] = 0
                    path_guidance.clear()
                    is_tracing_active = False
                    current_status_text = "PATH CLEARED"
                    reset_yaw_control_state()

                elif action == 'START':
                    if globals().get('REPLAY_ONLY',False):
                        current_status_text='REPLAY PREVIEW ONLY'
                        print('录像位置不是真实反馈；循迹请切换实时摄像头。')
                    elif turn_calibration_session.active:
                        current_status_text = "TURN CALIBRATING"
                        print("无法启动循迹：请先再次点击【转圈测量】完成本次测量。")
                    elif not POOL_SIZE_CONFIRMED:
                        current_status_text = "POOL SIZE REQUIRED"
                        print('无法启动：请先输入实测场地长宽，并完成四角标定。')
                    elif calib_state['H'] is None:
                        current_status_text = "UNCALIBRATED"
                        print("无法启动：请先点击【场地标定】完成四角标定。")
                    elif len(drawn_path_state['pixels']) < 2:
                        current_status_text = "NO PATH"
                        print("无法启动：请先用鼠标左键拖动画出轨迹。")
                    elif current_x is None or current_y is None:
                        current_status_text = "TARGET LOST"
                        reasons = ','.join(reference_observation.gate_reasons) or '无'
                        print(
                            "无法启动：黑色鱼体未锁定。"
                            f"来源={reference_observation.source}，门控={reasons}。"
                            "请先重新框选【框选机器鱼】。"
                        )
                    elif heading_state['world_unit_vector'] is None or not body_yaw_state['usable']:
                        current_status_text = "NO HEADING"
                        print("无法启动：请确认 IMU 遥测有效，并完成鱼头 0 yaw 标定。")
                    else:
                        if reference_observation.source in (ReferenceSource.RIGID_BODY, ReferenceSource.YOLO_BODY):
                            print("当前使用黑色鱼体中心追踪。")
                        if reference_observation.source == ReferenceSource.ESTIMATED_FALLBACK:
                            print("当前是无色标降级位置，只允许测试，数据不能用于 PWM—速度精标定。")
                        path_input = np.float32([[
                            [point[0], point[1]]
                            for point in drawn_path_state['pixels']
                        ]])
                        path_world = cv2.perspectiveTransform(
                            path_input,
                            calib_state['H']
                        )[0]
                        # Start from the current IMU heading in the marked world frame.
                        reset_yaw_control_state()
                        startup_visual_heading, _, startup_visual_source = (
                            select_yaw_for_control(now)
                        )
                        startup_heading=startup_visual_heading
                        startup_heading_source='IMU_YAW'
                        try:
                            initial_guidance = path_guidance.start(
                                path_world,
                                (control_x, control_y),
                                frame_time,
                                startup_heading,
                            )
                        except (ValueError, RuntimeError) as error:
                            current_status_text = "PATH INVALID"
                            path_guidance.clear()
                            print(f"无法准备轨迹：{error}")
                        else:
                            start_distance = float(np.linalg.norm(
                                np.asarray((control_x, control_y))
                                - path_guidance.path[0]
                            ))
                            if start_distance > 0.40:
                                current_status_text = "PATH TOO FAR"
                                path_guidance.clear()
                                print(
                                    f"无法启动：轨迹起点距离鱼 {start_distance:.2f}m，"
                                    "请从鱼附近开始画。"
                                )
                            elif fish_comm.ensure_bionic_mode():
                                fish_comm.vision_seq = 0
                                drawn_path_state['lost_since'] = None
                                drawn_path_state['active'] = True
                                drawn_path_state['segment'] = initial_guidance['seg_index']
                                drawn_path_state['last_control_t'] = 0.0
                                is_tracing_active = True
                                current_status_text = "BIONIC TRACKING"
                                print(
                                    "纯仿生前视轨迹引导已启动，双桨关闭："
                                    f"{len(path_guidance.path)} 个等距点，"
                                    "LOS 视觉外环 + 速度/yaw 并联 PID 内环，"
                                    "前视 0.10~0.24 m，"
                                    "UDP 固定 20 Hz。"
                                )
                                first_error = float(
                                    initial_guidance['steering_error_deg']
                                )
                                first_turn = (
                                    '左'
                                    if first_error > 3.0
                                    else '右'
                                    if first_error < -3.0
                                    else '直行'
                                )
                                source_text = '已标定的 IMU yaw'
                                print(
                                    f"起步使用{source_text}，"
                                    f"首个引导={first_turn} {abs(first_error):.1f}°。"
                                )
                            else:
                                path_guidance.clear()
                                current_status_text = "CONTROL LOST"
                                print("无法确认机器鱼进入纯仿生模式，未启动追踪。")

                elif action == 'TURN_CALIB':
                    if turn_calibration_session.active:
                        try:
                            turn_fit = turn_calibration_session.finish()
                        except TurnCalibrationError as error:
                            current_status_text = "READY"
                            print(f"转圈测量未通过：{error}")
                            print("本次结果未保存，请重新点击【转圈测量】再测。")
                        else:
                            try:
                                save_turn_calibration(
                                    TURN_CALIBRATION_PATH, turn_fit
                                )
                            except OSError as error:
                                current_status_text = "READY"
                                print(f"转圈结果保存失败：{error}")
                            else:
                                turn_calibration_results[
                                    turn_fit.direction
                                ] = turn_fit
                                path_guidance.set_turn_radius(
                                    turn_fit.direction, turn_fit.radius_m
                                )
                                current_status_text = "TURN CALIBRATED"
                                direction_name = (
                                    "左转"
                                    if turn_fit.direction == "LEFT"
                                    else "右转"
                                )
                                print(
                                    f"{direction_name}满转测量完成："
                                    f"直径={turn_fit.diameter_m:.3f}m，"
                                    f"半径={turn_fit.radius_m:.3f}m，"
                                    f"曲率={turn_fit.curvature_per_m:.2f}/m"
                                )
                                print(
                                    f"质量：有效点={turn_fit.inlier_count}/"
                                    f"{turn_fit.sample_count}，"
                                    f"圆弧={turn_fit.arc_coverage_deg:.1f}°，"
                                    f"RMS残差={turn_fit.rms_residual_m:.3f}m。"
                                )
                                print(
                                    "该方向的曲率前馈已改用实测半径；"
                                    "另一方向仍保持原实测值或 0.20m 估计值。"
                                )
                    elif calib_state['H'] is None:
                        print("无法测量：请先点击【场地标定】完成四角标定。")
                    elif (
                        direct_marker_world_position is None
                    ):
                        print(
                            "无法测量：当前必须直接锁定黑色鱼体；"
                            "PREDICTED 和 OpenCV 降级位置不会用于拟合。"
                        )
                    else:
                        if drawn_path_state['active'] or is_tracing_active:
                            drawn_path_state['active'] = False
                            is_tracing_active = False
                            fish_comm.stop_now()
                            print("已结束视觉循迹，准备被动测量转圈轨迹。")
                        calib_state['is_calibrating'] = False
                        marker_roi_state['selecting'] = False
                        heading_state['selecting'] = False
                        turn_calibration_session.start(
                            direct_marker_world_position, frame_time
                        )
                        current_status_text = "TURN CALIBRATING"
                        print(
                            "转圈测量已开始：请在网页端选择 BIONIC，"
                            "持续满左或满右转至少半圈。"
                        )
                        print(
                            "程序只记录黑色鱼体直接观测，不会自动控制鱼；"
                            "完成后再次点击【转圈测量】。"
                        )

                elif action == 'MARKER_ROI':
                    if drawn_path_state['active']:fish_comm.stop_now()
                    if turn_calibration_session.active:
                        turn_calibration_session.cancel()
                        print("已取消未完成的转圈测量。")
                    drawn_path_state['active'] = False
                    is_tracing_active = False
                    calib_state['is_calibrating'] = False
                    heading_state['selecting'] = False
                    marker_roi_state['selecting'] = not marker_roi_state['selecting']
                    marker_roi_state['dragging'] = False
                    marker_roi_state['start'] = None
                    marker_roi_state['end'] = None
                    if marker_roi_state['selecting']:
                        current_status_text = "SELECT MARKER ROI"
                        print("请框住完整机器鱼区域，程序会自动寻找白色鱼头与黑色鱼体；右键或再次点击【框选机器鱼】取消。")
                    else:
                        current_status_text = "READY"
                        print("已取消整鱼框选，原有识别保持不变。")

                elif action == 'APPLY_MARKER_ROI':
                    latest_frame = frame_state['latest']
                    try:
                        selected = reference_tracker.select(latest_frame, action_payload, frame_time)
                        imu_yaw.reset()
                        body_yaw_state.update(yaw_deg=None,unit_vector=None,usable=False,
                                              measurement_valid=False,last_frame_time=None)
                        heading_state['world_unit_vector'] = None
                        heading_state['control_heading'] = None
                        marker_roi_state['selecting'] = False
                        marker_roi_state['dragging'] = False
                        marker_roi_state['start'] = None
                        marker_roi_state['end'] = None
                        marker_roi_state['confirmed_bbox'] = selected.metrics['fish_bbox']
                        marker_roi_state['confirmed_center'] = selected.position
                        marker_roi_state['confirmed_until'] = time.time() + 3.0
                        current_status_text = "MARKER READY"
                        print("已锁定白色鱼头与黑色鱼体；位置使用黑色主体中心，无需黑色鱼体。")
                    except Exception as error:
                        marker_roi_state['selecting'] = True
                        marker_roi_state['dragging'] = False
                        marker_roi_state['start'] = None
                        marker_roi_state['end'] = None
                        current_status_text = "MARKER ROI RETRY"
                        print(f"整鱼框选失败：{error}。请调整范围后重新框选。")

                elif action == 'HEAD_DIRECTION':
                    if drawn_path_state['active']:fish_comm.stop_now()
                    if turn_calibration_session.active:
                        turn_calibration_session.cancel()
                        print("已取消未完成的转圈测量。")
                    drawn_path_state['active'] = False
                    is_tracing_active = False
                    calib_state['is_calibrating'] = False
                    marker_roi_state['selecting'] = False
                    heading_state['selecting'] = not heading_state['selecting']
                    if heading_state['selecting']:
                        current_status_text = "SELECT HEAD DIRECTION"
                        print("黑体中心将自动获取，请直接点击鱼头中心；右键或再次点击【鱼头方向】取消。")
                    else:
                        current_status_text = "READY"
                        print("已取消鱼头方向标定，原有初始方向保持不变。")

                elif action == 'APPLY_HEAD_DIRECTION':
                    tail_point = frame_state['tail_marker_position']
                    if calib_state['H'] is None:
                        current_status_text = "SELECT HEAD DIRECTION"
                        print("请先完成场地标定，再标定鱼头方向。")
                    elif not imu_yaw.valid_sample(yaw_sample,time.time()):
                        current_status_text='IMU YAW REQUIRED'
                        print('鱼头标定需要新鲜 IMU yaw，请先连接机器鱼并确认遥测有效。')
                    elif (
                        tail_point is None
                    ):
                        current_status_text = "SELECT HEAD DIRECTION"
                        print("当前没有可靠的黑色鱼体中心，请先完成框选机器鱼并让白头和黑体保持可见。")
                    else:
                        head_point = tuple(float(value) for value in action_payload)
                        tail_point = tuple(float(value) for value in tail_point)
                        print("已采用手动点击的鱼头位置标定初始朝向。")
                        dx = head_point[0] - tail_point[0]
                        dy = head_point[1] - tail_point[1]
                        length = float(np.hypot(dx, dy))
                        if length < 8:
                            current_status_text = "SELECT HEAD DIRECTION"
                            print("鱼头点离黑体中心太近，请准确点击鱼头中心。")
                        else:
                            world_points = cv2.perspectiveTransform(
                                np.float32([[tail_point, head_point]]),
                                calib_state['H'],
                            )[0]
                            world_delta = world_points[1] - world_points[0]
                            world_length = float(np.linalg.norm(world_delta))
                            if world_length < .02:
                                current_status_text = "SELECT HEAD DIRECTION"
                                print("鱼头方向映射失败，请重新点击。")
                            else:
                                world_unit = world_delta / world_length
                                heading_state['tail_point'] = tail_point
                                heading_state['head_point'] = head_point
                                heading_state['unit_vector'] = (dx / length, dy / length)
                                heading_state['world_unit_vector'] = (
                                    float(world_unit[0]), float(world_unit[1])
                                )
                                heading_state['control_heading'] = (
                                    float(world_unit[0]), float(world_unit[1])
                                )
                                heading_state['control_heading_source'] = 'CALIBRATED'
                                heading_state['angle_deg'] = float(
                                    (
                                        np.degrees(np.arctan2(-world_unit[1], world_unit[0]))
                                        + 360.0
                                    ) % 360.0
                                )
                                zero=imu_yaw.calibrate(math.radians(heading_state['angle_deg']),yaw_sample,time.time())
                                body_yaw_state.update(yaw_deg=0.,yaw_rate_rps=0.,
                                    unit_vector=zero['unit_vector'],world_yaw_rad=zero['world_yaw_rad'],
                                    sample_time=zero['sample_time'],source='IMU_YAW',prediction_age_s=zero['age_s'],
                                    usable=True,measurement_valid=True,initialized=True,polarity_calibrated=True,
                                    last_frame_time=float(frame_time),last_update_wall_t=time.monotonic(),
                                    raw_yaw_deg=zero['raw_yaw_deg'])
                                heading_state['confirmed_until'] = time.time() + 5.0
                                heading_state['selecting'] = False
                                current_status_text = "HEAD DIRECTION READY"
                                print(
                                    "初始鱼头方向标定完成："
                                    f"尾点=({tail_point[0]:.1f},{tail_point[1]:.1f})，"
                                    f"头点=({head_point[0]:.1f},{head_point[1]:.1f})，"
                                    f"世界方向={heading_state['angle_deg']:.1f}°，当前 IMU 相对 yaw=0°；左转为正。"
                                )

                elif action == 'POOL_SIZE':
                    configure_pool_size(*action_payload)
                    if drawn_path_state['active'] or is_tracing_active:
                        fish_comm.stop_now()
                    calib_state['is_calibrating'] = False
                    pending_ui_actions.appendleft('POOL_CALIB')

                elif action == 'POOL_CALIB':
                    if not POOL_SIZE_CONFIRMED:
                        current_status_text = "POOL SIZE REQUIRED"
                        print('请先在视觉参数中填写并应用实测场地长宽；未确认时仅识别预览。')
                        continue
                    if turn_calibration_session.active:
                        turn_calibration_session.cancel()
                        print("已取消未完成的转圈测量。")
                    calib_state['is_calibrating'] = not calib_state['is_calibrating']
                    reset_yaw_control_state()
                    marker_roi_state['selecting'] = False
                    heading_state['selecting'] = False
                    if calib_state['is_calibrating']:
                        white_head_tracker.reset_motion()
                        drawn_path_state['active'] = False
                        drawn_path_state['pixels'].clear()
                        path_guidance.clear()
                        heading_state['tail_point'] = None
                        heading_state['head_point'] = None
                        heading_state['unit_vector'] = None
                        heading_state['world_unit_vector'] = None
                        heading_state['angle_deg'] = None
                        imu_yaw.reset()
                        motion_prior_state.update({
                            'last_t': None,
                            'last_r_cmd_rps': 0.0,
                            'last_heading_rad': None,
                            'last_fused_yaw_deg': None,
                            'gate_count': 0,
                        })
                        body_yaw_state.update({
                            'yaw_deg': None,
                            'visual_yaw_deg': None,
                            'fused_yaw_deg': None,
                            'motion_prior_deg': None,
                            'motion_gate_applied': False,
                            'yaw_rate_rps': None,
                            'unit_vector': None,
                            'source': 'INVALID',
                            'prediction_age_s': None,
                            'usable': False,
                            'measurement_rejected': False,
                            'polarity_corrected': False,
                            'reacquire_pending': False,
                            'reacquire_samples': 0,
                            'pose_rejection_reason': None,
                            'polarity_calibrated': False,
                            'last_frame_time': None,
                            'last_update_wall_t': None,
                            'raw_yaw_deg': None,
                            'detector_raw_yaw_deg': None,
                            'color_raw_yaw_deg': None,
                            'color_tail_pixel': None,
                            'color_head_pixel': None,
                            'color_head_source': 'UNAVAILABLE',
                            'head_color_confidence': 0.0,
                            'white_dynamic_value_min': None,
                            'tail_color_confidence': 0.0,
                            'yaw_measurement_source': 'NONE',
                        })
                        is_tracing_active = False
                        current_status_text = "CALIBRATING"
                        calib_state['H'] = None
                        calib_state['inv_H'] = None
                        calib_state['overlay_dirty'] = True
                        calib_state['manual_locked'] = False
                        calib_state['auto_locked'] = False
                        calib_state['auto_stable_count'] = 0
                        calib_state['auto_prev_points'] = None
                        calib_state['pts_raw'].clear()
                        calib_state['pts_disp'].clear()
                        calib_state['overlay_dirty'] = True
                        pos_state['smooth_x'] = None
                        pos_state['smooth_y'] = None
                        print("请按左上→右上→右下→左下依次点击四个场地角点。")
                    else:
                        calib_state['pts_raw'].clear()
                        calib_state['pts_disp'].clear()
                        calib_state['overlay_dirty'] = True
                        current_status_text = "UNCALIBRATED"
                        print("已取消场地标定；当前没有有效标定矩阵。")

                elif action == 'RECORD':
                    if is_recording:
                        is_recording = False
                        if recorder:
                            recorder.close()
                            recorder = None
                        print("Recording stopped and file finalized.", flush=True)
                    else:
                        stamp = time.strftime("%Y%m%d_%H%M%S") + f"_{time.time_ns()%1000000000:09d}"
                        last_recorded_frame_time = None
                        try:
                            recorder = AsyncVideoRecorder(
                                path=f'{RECORD_DIR}/auv_record_{stamp}.mp4',
                                fourcc=fourcc,
                                fps=max(1.0, min(120.0, round(cam.reported_fps if not isinstance(cam.src,int) else cam.measured_fps, 1))),
                                frame_size=(img_show.shape[1], img_show.shape[0]),
                                csv_path=f'{RECORD_DIR}/auv_data_{stamp}.csv')
                            is_recording = True
                        except (RuntimeError, OSError, ValueError) as error:
                            is_recording = False
                            recorder = None
                            print('[RECORD ERROR] '+str(error), flush=True)

                elif action == 'SNAPSHOT':
                    ms = int((time.time() * 1000) % 1000)
                    img_filename = f"{RECORD_DIR}/auv_frame_{time.strftime('%Y%m%d_%H%M%S')}_{ms:03d}.jpg"
                    cv2.imwrite(img_filename, img_show)
                    print(f"抓拍成功！已保存为: {img_filename}")

                elif action == 'CLAHE':
                    use_clahe = not use_clahe
                    print(f"CLAHE 状态切换: {'【开启】' if use_clahe else '【关闭】'}")

                elif action == 'DEBUG':
                    show_debug_overlay = not show_debug_overlay
                    current_status_text = (
                        "DEBUG OVERLAY" if show_debug_overlay else "READY"
                    )
                    print(
                        f"调试层{'开启' if show_debug_overlay else '关闭'}："
                        "仅影响画面叠加，不影响检测、滤波或控制"
                    )

                elif action == 'BINARY':
                    show_binary_preview = not show_binary_preview
                    print(
                        f"全画面二值化{'开启' if show_binary_preview else '关闭'}"
                    )

                elif action == 'EXP_DOWN':
                    cam.adjust_exposure(-1)

                elif action == 'EXP_UP':
                    cam.adjust_exposure(1)

            web_bridge.complete(current_status_text, drawn_path_state['active'])
            if exit_requested:
                break

    except Exception as e:
        print(f"运行过程中发生未捕获异常: {e}")
        import traceback
        traceback.print_exc()

    finally:
        print("\n正在清理系统资源并关闭网络服务...")
        fish_comm.close()
        tablet_server.close()
        cam.release()
        if recorder:
            recorder.close()
        cv2.destroyAllWindows()
        print("所有通信接口与相机资源已安全释放，程序优雅退出。")

if __name__ == "__main__":
    main()
