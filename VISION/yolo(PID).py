import os
import cv2
import numpy as np
import time
import csv
import threading
import json
import socket
import queue
import urllib.request
import urllib.parse
from flask import Flask, Response

os.environ["YOLO_CONFIG_DIR"] = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".ultralytics"
)
os.environ["MPLCONFIGDIR"] = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".matplotlib"
)

from ultralytics import YOLO
from collections import deque

# ============= 设置工作目录（带路径安全回退） =============
PREFERRED_DIR = os.path.dirname(os.path.abspath(__file__))
WORK_DIR = PREFERRED_DIR
if not os.path.exists(WORK_DIR):
    os.makedirs(WORK_DIR)
os.chdir(WORK_DIR)

# 🔴 核心提速秘籍：禁用 OpenCV MSMF 后端的硬件转换功能
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

# ==============================================================================
# 🔴 [配置区] 相机采集帧率与分辨率
# ==============================================================================
TARGET_FPS = 60
TARGET_WIDTH = 640
TARGET_HEIGHT = 480

# 是否默认开启 CLAHE（直方图均衡化，水面有强烈反光或光照不均时开启）
ENABLE_CLAHE_DEFAULT = False

# ==============================================================================
# 🔴 [配置区] 场地真实物理尺寸与标靶 ID 映射
# ==============================================================================
PHYSICAL_WIDTH = 3.14   # 水池长度（米）
PHYSICAL_HEIGHT = 1.6   # 水池宽度（米）

MARKER_TL = 1
MARKER_TR = 2
MARKER_BR = 4
MARKER_BL = 3

# ==============================================================================
# 🐟 [配置区] YOLO 仿生鱼检测
# ==============================================================================
YOLO_MODEL_PATH = os.path.join(WORK_DIR, "best.pt")
YOLO_CONF_THRESHOLD = 0.5
YOLO_IMG_SIZE = 640
YOLO_DEVICE = 0             # 0 = GPU，无显卡改为 "cpu"

# ==============================================================================
# 📡 [配置区] 通信参数（平板 TCP & RoboFish ESP32 HTTP）
# ==============================================================================
TABLET_TCP_HOST = "0.0.0.0"
TABLET_TCP_PORT = 9998

# 🔴 RoboFish 机器鱼 AP 网络默认配置
ROBOFISH_IP = "192.168.4.1"
ROBOFISH_PORT = 80

# --- 坐标与速度计算平滑系数 ---
POS_SMOOTHING_ALPHA = 0.4      # 坐标平滑系数（抗 YOLO 坐标抖动）
VELOCITY_SMOOTHING_ALPHA = 0.3 # 速度平滑系数

# --- MJPEG 视频推流配置 ---
MJPEG_PORT = 8090
MJPEG_STREAM_WIDTH = 960      # 缩放至 960x540，降低带宽
MJPEG_STREAM_HEIGHT = 540
MJPEG_JPEG_QUALITY = 50       # 压缩质量设为 50，提升传输顺畅度
MJPEG_MAX_FPS = 30            # 限制推流上限 30 帧

# --- 本机预览窗口刷新帧率上限 ---
DISPLAY_MAX_FPS = 30

# 辅助函数：绘制高对比度加粗黑边 OSD 文字（即使水面强光反光也能清晰可见）
def draw_outlined_text(img, text, pos, font, scale, color, thickness=2):
    x, y = pos
    cv2.putText(img, text, (x, y), font, scale, (0, 0, 0), thickness + 3, cv2.LINE_AA)
    cv2.putText(img, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)


def build_calibration_homography(points, frame_width, frame_height):
    """验证 TL→TR→BR→BL 四点并生成像素到物理坐标的单应矩阵。"""
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

# ==============================================================================
# 🎛️ 标准位置式 PID 控制器类 (带抗积分饱和限幅 & 微分平滑)
# ==============================================================================
class PIDController:
    def __init__(self, Kp=150.0, Ki=8.0, Kd=35.0, output_min=10, output_max=70):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_min = output_min
        self.output_max = output_max

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def update(self, error):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0.0:
            dt = 0.01

        # 1. P 项 (比例)
        p_term = self.Kp * error

        # 2. I 项 (积分) + 抗饱和限幅 (Anti-Windup)
        self.integral += error * dt
        self.integral = np.clip(self.integral, -0.5, 0.5)
        i_term = self.Ki * self.integral

        # 3. D 项 (微分 - 抑制水下惯性引发的蛇形走位)
        derivative = (error - self.prev_error) / dt
        d_term = self.Kd * derivative

        self.prev_error = error
        self.last_time = now

        # 总 PID 输出计算
        output = p_term + i_term + d_term
        return output

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()


def compute_drawn_path_error(position, path_points, start_segment=0):
    """计算鱼相对绘制轨迹的有符号横向误差和剩余距离，单位为米。"""
    if position is None or path_points is None or len(path_points) < 2:
        return None

    pos = np.asarray(position, dtype=np.float32)
    path = np.asarray(path_points, dtype=np.float32)
    first = int(np.clip(start_segment - 2, 0, len(path) - 2))
    last = min(len(path) - 2, first + 30)
    best = None

    for seg_index in range(first, last + 1):
        p0 = path[seg_index]
        segment = path[seg_index + 1] - p0
        length_sq = float(np.dot(segment, segment))
        if length_sq <= 1e-8:
            continue

        projection = float(np.clip(np.dot(pos - p0, segment) / length_sq, 0.0, 1.0))
        closest = p0 + projection * segment
        offset = pos - closest
        distance_sq = float(np.dot(offset, offset))
        if best is None or distance_sq < best['distance_sq']:
            segment_length = float(np.sqrt(length_sq))
            cross_m = float((segment[0] * offset[1] - segment[1] * offset[0]) / segment_length)
            best = {
                'distance_sq': distance_sq,
                'cross_m': cross_m,
                'projection': projection,
                'segment': seg_index,
                'segment_length': segment_length,
            }

    if best is None:
        return None

    remaining_m = (1.0 - best['projection']) * best['segment_length']
    for seg_index in range(best['segment'] + 1, len(path) - 1):
        remaining_m += float(np.linalg.norm(path[seg_index + 1] - path[seg_index]))

    return {
        'cross_m': best['cross_m'],
        'along_m': best['projection'] * best['segment_length'],
        'dist_m': remaining_m,
        'seg_index': best['segment'],
    }

# ==============================================================================
# 📷 相机多线程读取类（含曝光手动调节支持）
# ==============================================================================
class CameraStream:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src, cv2.CAP_DSHOW)
        self.lock = threading.Lock()

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, TARGET_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)

        self.real_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.real_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.reported_fps = self.cap.get(cv2.CAP_PROP_FPS)

        # 曝光控制
        self.exposure_val = -5
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure_val)
        self.cap.set(cv2.CAP_PROP_GAIN, 100)

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
        with self.lock:
            self.exposure_val += delta
            self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure_val)
            print(f"📷 相机手动曝光值调整为: {self.exposure_val}")

    def update(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            if ret:
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
# 📺 MJPEG 视频流服务器
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
                    print(f"⚠️ 视频流已有 {self._viewer_count} 个客户端在看，拒绝新连接")
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
            print(f"📺 视频流新增1个观看者，当前共 {self._viewer_count} 个")

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
                print(f"📺 视频流断开1个观看者，当前共 {self._viewer_count} 个")

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
        print(f"📺 MJPEG视频流已启动: http://0.0.0.0:{self.port}/video.mjpg")

# ==============================================================================
# 📱 平板 TCP 服务器（含实时 TX/RX 传输频率计算）
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

        # 🔴 频率计算参数
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
            print(f"✅ 平板 TCP 有线双向服务器已启动: {self.host}:{self.port}")

            self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
            self._accept_thread.start()
            self._sender_thread = threading.Thread(target=self._sender_loop, daemon=True)
            self._sender_thread.start()
            self._receiver_thread = threading.Thread(target=self._receiver_loop, daemon=True)
            self._receiver_thread.start()
        except Exception as e:
            print(f"⚠️ 平板 TCP 服务器启动失败: {e}")

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
                print(f"✅ 平板 UI 已建立连接: {addr}")
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
                        print(f"🔌 平板 UI 连接断开: {self.client_addr}")
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
                                print(f"🗺️  [UI 消息] 收到轨迹规划: 点数={num_pts} | 模式={mode} | 反向={rev} | 循环={loop_flag}")

                            elif msg_type == "tracking_error":
                                with self._rx_lock:
                                    self._latest_tracking_err = parsed

                            elif msg_type == "command":
                                cmd = parsed.get('cmd')
                                self._command_queue.put(parsed)
                                print(f"⚠️  [UI 消息] 收到控制指令: 【{cmd.upper()}】")

                        except json.JSONDecodeError:
                            pass

            except socket.timeout:
                continue
            except (socket.error, ConnectionResetError):
                with self._lock:
                    if self.client_socket:
                        print(f"🔌 平板 UI 接收通道已断开")
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
# 🤖 RoboFish ESP32S3 异步 HTTP 通信类 (含 10Hz 限频防冲垮 + 频率统计)
# ==============================================================================
class RoboFishComm:
    def __init__(self, ip=ROBOFISH_IP, port=ROBOFISH_PORT):
        self.ip = ip
        self.port = port
        self.base_url = f"http://{ip}:{port}"
        self.cmd_queue = queue.Queue(maxsize=1)  # 🔴 队列深度设为 1，只留最新指令
        self.stopped = False

        # 🔴 单片机通信频率统计与限频控制
        self.mcu_hz = 0.0
        self._last_send_t = time.time()
        self._last_cmd_t = 0.0                  # 发包间隔时间戳
        self._last_params = None                # 指令去重缓存
        self.vision_seq = 0                     # MCU 视觉 PID 样本序号

        self.thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.thread.start()
        print(f"✅ RoboFish 机器鱼 HTTP PID 控制接口准备就绪 -> 目标地址: {self.base_url}")

    def _request_control(self, variable, value=1, timeout=0.3):
        query_string = urllib.parse.urlencode({"var": variable, "val": value})
        url = f"{self.base_url}/control?{query_string}"
        req = urllib.request.Request(
            url, headers={'User-Agent': 'RoboFish-Visual-Tracker'}
        )
        with urllib.request.urlopen(req, timeout=timeout) as response:
            return response.read().decode('utf-8', errors='replace').strip()

    def _request_vision_pid(self, sequence, x_error_m, y_error_m, timeout=0.3):
        query_string = urllib.parse.urlencode({
            "seq": int(sequence),
            "x_error": f"{float(x_error_m):.4f}",
            "y_error": f"{float(y_error_m):.4f}",
        })
        url = f"{self.base_url}/vision_pid?{query_string}"
        req = urllib.request.Request(
            url, headers={'User-Agent': 'RoboFish-Visual-Tracker'}
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
                    self._request_vision_pid(
                        params["seq"], params["x_error"], params["y_error"],
                        timeout=0.3
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
                    self._request_control(action_map[params["action"]], timeout=0.2)

                # 统计单片机交互实际成功 Hz
                now = time.time()
                dt = now - self._last_send_t
                if dt > 0:
                    self.mcu_hz = 0.9 * self.mcu_hz + 0.1 * (1.0 / dt)
                self._last_send_t = now

            except Exception as e:
                # 🔴 增加错误打印，防止网络报错被静默忽略
                print(f"⚠️ [HTTP 发送失败] 无法连通机器鱼 ({self.base_url}): {e}")

    def get_mcu_hz(self):
        if time.time() - self._last_send_t > 1.5:
            self.mcu_hz *= 0.5
        return round(self.mcu_hz, 1)

    def _send_async(self, params):
        now = time.time()
        is_stop = params.get("action") == "stop"

        # 1. 限频控制：单片机限制最高下发频率为 10Hz (间隔>=0.1s)
        if not is_stop and (now - self._last_cmd_t < 0.1):
            return

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
        except queue.Full:
            pass

    def set_mode(self, motion_mode):
        mode_map = {
            "bionic": "mode_bionic",
            "propeller": "mode_prop",
            "prop": "mode_prop",
            "hybrid": "mode_hybrid"
        }
        target_mode = mode_map.get(str(motion_mode).lower(), "mode_bionic")
        print(f"⚙️ [HTTP 下发] 切换机器鱼模式为: {target_mode}")
        self._send_async({"action": target_mode})

    def ensure_prop_mode(self):
        """按固件的循环模式接口切换，最多三次确保进入 PROP。"""
        for _ in range(3):
            try:
                mode = self._request_control("mode", timeout=0.5).upper()
                print(f"⚙️ [HTTP 模式反馈] {mode or 'EMPTY'}")
                if mode == "PROP":
                    return True
            except Exception as error:
                print(f"⚠️ [HTTP 模式切换失败] {error}")
                return False
        return False

    def send_command(self, cmd_str):
        if cmd_str in ["stop", "emergency_stop"]:
            print(f"🚀 [HTTP 下发] 机器鱼急停/停止推进")
            self._send_async({"action": "stop"})

    def stop_now(self):
        """同步停止，用于退出、清轨迹和失去视觉目标。"""
        self._last_params = None
        while not self.cmd_queue.empty():
            try:
                self.cmd_queue.get_nowait()
            except queue.Empty:
                break
        try:
            self._request_control("servo_stop", timeout=0.5)
            print("🛑 [HTTP 反馈] 推进已停止")
            return True
        except Exception as error:
            print(f"⚠️ [HTTP 停止失败] {error}")
            return False

    def process_tracking_error(self, cross_m, along_m, dist_m):
        """发送标定后的误差；X/Y PID 均由 ESP32 的 VisionControl task 计算。"""
        self.vision_seq = (self.vision_seq + 1) & 0x7fffffff
        print(
            f"🚀 [MCU XY PID] seq={self.vision_seq} "
            f"x={cross_m:+.3f}m y={dist_m:.3f}m"
        )
        self._send_async({
            "kind": "vision_pid",
            "seq": self.vision_seq,
            "x_error": float(np.clip(cross_m, -5.0, 5.0)),
            "y_error": float(np.clip(dist_m, 0.0, 5.0)),
        })

    def close(self):
        self.stopped = True
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)

# ==============================================================================
# 🐟 YOLO 仿生鱼检测器（独立线程 + BoT-SORT 轻量追踪）
# ==============================================================================
class FishDetector:
    def __init__(self, model_path, conf=0.5, imgsz=640, device=0):
        print(f"🐟 正在加载YOLO模型: {model_path} ...")
        self.model = YOLO(model_path)
        self.conf = conf
        self.imgsz = imgsz
        self.device = device

        self._latest_frame = None
        self._latest_frame_lock = threading.Lock()

        self._result_lock = threading.Lock()
        self._result = {
            'pixel': None,
            'confidence': 0.0,
            'track_id': None,
            'frame_time': 0.0
        }

        self._fps_smooth = 0.0
        self._last_infer_t = time.time()

        self._stop = False
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self.thread.start()
        print(f"✅ YOLO检测线程已启动（设备: {self.device}, 输入尺寸: {self.imgsz}）")
        return self

    def submit_frame(self, frame, frame_time):
        with self._latest_frame_lock:
            self._latest_frame = (frame, frame_time)

    def _loop(self):
        while not self._stop:
            with self._latest_frame_lock:
                item = self._latest_frame
                self._latest_frame = None
            if item is None:
                time.sleep(0.001)
                continue

            frame, frame_time = item

            try:
                results = self.model.track(
                    frame,
                    conf=self.conf,
                    imgsz=self.imgsz,
                    device=self.device,
                    tracker="botsort.yaml",
                    persist=True,
                    verbose=False
                )
            except Exception as e:
                print(f"⚠️ YOLO推理出错: {e}")
                continue

            pixel = None
            confidence = 0.0
            track_id = None

            if len(results) > 0 and results[0].boxes is not None and len(results[0].boxes) > 0:
                boxes = results[0].boxes
                best_idx = int(boxes.conf.argmax().item())

                x1, y1, x2, y2 = boxes.xyxy[best_idx].tolist()
                confidence = float(boxes.conf[best_idx].item())
                pixel = [(x1 + x2) / 2.0, (y1 + y2) / 2.0]

                if boxes.id is not None:
                    track_ids = boxes.id.cpu().numpy()
                    if len(track_ids) > best_idx:
                        track_id = int(track_ids[best_idx])

            now = time.time()
            dt = now - self._last_infer_t
            self._last_infer_t = now
            if dt > 0:
                self._fps_smooth = 0.9 * self._fps_smooth + 0.1 * (1.0 / dt)

            with self._result_lock:
                self._result = {
                    'pixel': pixel,
                    'confidence': confidence,
                    'track_id': track_id,
                    'frame_time': frame_time,
                    'infer_fps': self._fps_smooth
                }

    def get_latest(self):
        with self._result_lock:
            return dict(self._result)

    def close(self):
        self._stop = True
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)

# ==============================================================================
# 🎬 异步录像写入类
# ==============================================================================
class AsyncVideoRecorder:
    def __init__(self, path, fourcc, fps, frame_size, csv_path):
        self.video_out = cv2.VideoWriter(path, fourcc, fps, frame_size)
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'pixel_u', 'pixel_v', 'physical_x', 'physical_y'])

        self.queue = queue.Queue(maxsize=300)
        self.stopped = False
        self.dropped_frames = 0
        self.thread = threading.Thread(target=self._writer_loop, daemon=True)
        self.thread.start()

    def submit(self, frame, csv_row):
        try:
            self.queue.put_nowait((frame, csv_row))
        except queue.Full:
            self.dropped_frames += 1

    def _writer_loop(self):
        while not (self.stopped and self.queue.empty()):
            try:
                frame, csv_row = self.queue.get(timeout=0.1)
            except queue.Empty:
                continue
            self.video_out.write(frame)
            if csv_row is not None:
                self.csv_writer.writerow(csv_row)

    def close(self):
        self.stopped = True
        if self.thread.is_alive():
            self.thread.join()
        self.video_out.release()
        self.csv_file.close()
        if self.dropped_frames > 0:
            print(f"⚠️ 录制过程中磁盘写入跟不上，丢弃了 {self.dropped_frames} 帧")


# ==============================================================================
# 主函数
# ==============================================================================
def main():
    print("\n" + "=" * 60)
    print("🚀 YOLO 仿生鱼追踪与 RoboFish HTTP PID 控制转发系统正在启动...")
    print(f"📂 工作目录: {os.path.abspath(WORK_DIR)}")
    print(f"📐 目标捕获: {TARGET_WIDTH}x{TARGET_HEIGHT} @ {TARGET_FPS}FPS")
    print(f"📱 平板有线 TCP 端口: {TABLET_TCP_PORT}")
    print(f"🤖 RoboFish 机器鱼目标: http://{ROBOFISH_IP}:{ROBOFISH_PORT}")
    print("=" * 60 + "\n")

    cam = CameraStream(src=1).start()
    if not cam.ret:
        print("❌ 无法打开相机！请检查 USB 接口是否松动，或尝试修改索引。")
        cam.release()
        return

    time.sleep(0.5)
    print(f"✅ 相机流已就绪：{cam.real_width} x {cam.real_height}")

    # 1. 启动平板 TCP 服务器
    tablet_server = TabletTCPServer(host=TABLET_TCP_HOST, port=TABLET_TCP_PORT)

    # 2. 启动 MJPEG 视频推流服务器
    mjpeg_server = MJPEGServer()
    mjpeg_server.start()

    # 3. 启动 RoboFish ESP32 HTTP 通信与 PID 控制服务
    fish_comm = RoboFishComm(ip=ROBOFISH_IP, port=ROBOFISH_PORT)

    # 4. 启动 YOLO 视觉检测线程
    fish_detector = FishDetector(
        model_path=YOLO_MODEL_PATH,
        conf=YOLO_CONF_THRESHOLD,
        imgsz=YOLO_IMG_SIZE,
        device=YOLO_DEVICE
    ).start()

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    use_clahe = ENABLE_CLAHE_DEFAULT

    trajectory = deque(maxlen=150)

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    recorder = None
    is_recording = False
    current_status_text = "READY"
    is_tracing_active = False
    drawn_path_state = {
        'pixels': [],
        'drawing': False,
        'active': False,
        'segment': 0,
        'last_control_t': 0.0,
        'stop_requested': False,
    }

    calib_state = {
        'H': None,
        'is_calibrating': False,
        'manual_locked': False,
        'auto_locked': False,
        'auto_stable_count': 0,
        'auto_prev_points': None,
        'pts_raw': [],
        'pts_disp': []
    }

    # 像素平滑与平滑轨迹控制参数
    pixel_state = {'smooth_x': None, 'smooth_y': None}
    PIXEL_SMOOTH_ALPHA = 0.35      # 像素一阶平滑滤波系数
    MIN_TRAJ_DIST_SQ = 3.0 ** 2    # 移动阈值死区

    pos_state = {'smooth_x': None, 'smooth_y': None}
    velocity_state = {'prev_x': None, 'prev_y': None, 'prev_t': None, 'vx_smooth': 0.0, 'vy_smooth': 0.0}
    fps_state = {'last_loop_t': time.time(), 'loop_fps': 0.0}
    display_state = {'last_t': 0.0}

    last_motion_mode = None  # 记录上一次发出的运动模式

    window_name = "AUV YOLO Tracker (RoboFish PID)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    display_width = 960
    display_height = int(display_width * cam.real_height / cam.real_width)
    cv2.resizeWindow(window_name, display_width, display_height)

    def mouse_callback(event, x, y, flags, param):
        rx = int(np.clip(x, 0, cam.real_width - 1))
        ry = int(np.clip(y, 0, cam.real_height - 1))

        if event == cv2.EVENT_LBUTTONDOWN and calib_state['is_calibrating']:
            # WIN32 HighGUI 已将缩放窗口中的点击位置换算为原图坐标。
            calib_state['pts_raw'].append([rx, ry])
            calib_state['pts_disp'].append((rx, ry))
            if len(calib_state['pts_raw']) == 4:
                homography, error = build_calibration_homography(
                    calib_state['pts_raw'], cam.real_width, cam.real_height
                )
                if error:
                    print(f"❌ 手动标定失败：{error}，请重新按左上→右上→右下→左下点击。")
                    calib_state['pts_raw'].clear()
                    calib_state['pts_disp'].clear()
                else:
                    calib_state['H'] = homography
                    calib_state['manual_locked'] = True
                    calib_state['is_calibrating'] = False
                    print("✅ 手动标定完成并锁定！")

        elif not calib_state['is_calibrating']:
            if event == cv2.EVENT_LBUTTONDOWN:
                drawn_path_state['pixels'] = [(rx, ry)]
                drawn_path_state['drawing'] = True
                drawn_path_state['active'] = False
                drawn_path_state['segment'] = 0
                drawn_path_state['stop_requested'] = True
            elif event == cv2.EVENT_MOUSEMOVE and drawn_path_state['drawing']:
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
                drawn_path_state['stop_requested'] = True

    cv2.setMouseCallback(window_name, mouse_callback)
    print("\n🎥 系统运行中，等待平板 UI 指令...")
    print("👉 轨迹控制：左键拖动画线  [Enter] PROP模式启动  [Space] 停止  [右键] 清除  [q] 停止并退出")
    print("👉 其他功能：[a] 录像  [b] 抓拍  [t] 重新标定  [c] CLAHE  [[]/[]] 曝光\n")

    last_processed_frame_time = None

    try:
        while True:
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                fish_comm.stop_now()
                print("🛑 窗口已关闭，推进已停止。")
                break
            ret, frame, frame_time = cam.read()
            if not ret or frame is None:
                continue

            if frame_time == last_processed_frame_time:
                if time.time() - frame_time > 0.5:
                    fish_comm.stop_now()
                    print("🛑 摄像头画面中断，已停止推进并退出。")
                    break
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                    fish_comm.stop_now()
                    print("🛑 窗口已关闭，推进已停止。")
                    break
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    fish_comm.stop_now()
                    break
                elif key == ord(' '):
                    fish_comm.stop_now()
                    drawn_path_state['active'] = False
                    is_tracing_active = False
                elif key == ord('['):
                    cam.adjust_exposure(-1)
                elif key == ord(']'):
                    cam.adjust_exposure(1)
                time.sleep(0.001)
                continue
            last_processed_frame_time = frame_time

            # 计算主循环帧率
            now = time.time()
            loop_dt = now - fps_state['last_loop_t']
            fps_state['last_loop_t'] = now
            if loop_dt > 0:
                inst_loop_fps = 1.0 / loop_dt
                fps_state['loop_fps'] = 0.9 * fps_state['loop_fps'] + 0.1 * inst_loop_fps

            img_show = frame.copy()
            img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

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

            for point_index, point in enumerate(calib_state['pts_raw']):
                px, py = int(point[0]), int(point[1])
                cv2.circle(img_show, (px, py), 7, (0, 165, 255), -1)
                draw_outlined_text(
                    img_show, str(point_index + 1), (px + 10, py - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2
                )

            # 动态 CLAHE 直方图均衡化控制（应对水面强反光）
            if use_clahe:
                img_gray = clahe.apply(img_gray)

            # 提交帧给异步 YOLO 线程
            fish_detector.submit_frame(frame, frame_time)

            corner_pixels = {}
            if calib_state['H'] is None:
                corners, ids, _ = detector.detectMarkers(img_gray)

                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(img_show, corners, ids)
                    ids_flat = ids.flatten()
                    for i in range(len(ids_flat)):
                        marker_id = int(ids_flat[i])
                        cx, cy = np.mean(corners[i][0][:, 0]), np.mean(corners[i][0][:, 1])

                        if marker_id in [MARKER_TL, MARKER_TR, MARKER_BL, MARKER_BR]:
                            corner_pixels[marker_id] = [cx, cy]

            # 获取 YOLO 检测最新结果
            yolo_result = fish_detector.get_latest()
            auv_pixel = yolo_result['pixel']
            auv_id_detected = auv_pixel is not None
            yolo_confidence = yolo_result['confidence']
            yolo_infer_fps = yolo_result.get('infer_fps', 0.0)
            track_id = yolo_result.get('track_id', None)

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

                cv2.rectangle(img_show, (cx - 15, cy - 15), (cx + 15, cy + 15), (0, 0, 255), 2)
                cv2.circle(img_show, (cx, cy), 4, (0, 0, 255), -1)
                id_text = f"fish {yolo_confidence:.2f}"
                if track_id is not None:
                    id_text = f"ID:{track_id} {yolo_confidence:.2f}"
                cv2.putText(img_show, id_text, (cx + 20, cy - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                if len(trajectory) == 0:
                    trajectory.append((cx, cy))
                else:
                    last_pt = trajectory[-1]
                    dist_sq = (cx - last_pt[0]) ** 2 + (cy - last_pt[1]) ** 2
                    if dist_sq >= MIN_TRAJ_DIST_SQ:
                        trajectory.append((cx, cy))

                if len(trajectory) > 1:
                    pts = np.array(trajectory, np.int32).reshape((-1, 1, 2))
                    cv2.polylines(img_show, [pts], isClosed=False, color=(0, 255, 255), thickness=2, lineType=cv2.LINE_AA)
            else:
                pixel_state['smooth_x'] = None
                pixel_state['smooth_y'] = None

            # 自动标定四个角点
            if len(corner_pixels) == 4 and not calib_state['manual_locked'] and not calib_state['auto_locked']:
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
                        print(f"⚠️ 自动标定未锁定：{error}")
                        calib_state['auto_stable_count'] = 0
                    else:
                        calib_state['H'] = homography
                        calib_state['auto_locked'] = True
                        print("✅ 自动标定成功！四个场地角点已稳定并锁定。")
            elif len(corner_pixels) < 4:
                calib_state['auto_stable_count'] = 0
                calib_state['auto_prev_points'] = None

            # 解算物理坐标与平滑滤波
            current_x, current_y = None, None
            if calib_state['H'] is not None and auv_pixel is not None:
                pt_real = cv2.perspectiveTransform(
                    np.float32([[[auv_pixel[0], auv_pixel[1]]]]), calib_state['H']
                )
                raw_x, raw_y = float(pt_real[0][0][0]), float(pt_real[0][0][1])

                if pos_state['smooth_x'] is None or pos_state['smooth_y'] is None:
                    pos_state['smooth_x'] = raw_x
                    pos_state['smooth_y'] = raw_y
                else:
                    a_pos = POS_SMOOTHING_ALPHA
                    pos_state['smooth_x'] = a_pos * raw_x + (1 - a_pos) * pos_state['smooth_x']
                    pos_state['smooth_y'] = a_pos * raw_y + (1 - a_pos) * pos_state['smooth_y']

                current_x = float(pos_state['smooth_x'])
                current_y = float(pos_state['smooth_y'])
            else:
                pos_state['smooth_x'] = None
                pos_state['smooth_y'] = None

            # 计算运动速度与方向
            speed = None
            direction_deg = None
            if current_x is not None and current_y is not None:
                if velocity_state['prev_x'] is not None:
                    dt = frame_time - velocity_state['prev_t']
                    if dt > 0:
                        vx_raw = (current_x - velocity_state['prev_x']) / dt
                        vy_raw = (current_y - velocity_state['prev_y']) / dt
                        a_vel = VELOCITY_SMOOTHING_ALPHA
                        velocity_state['vx_smooth'] = a_vel * vx_raw + (1 - a_vel) * velocity_state['vx_smooth']
                        velocity_state['vy_smooth'] = a_vel * vy_raw + (1 - a_vel) * velocity_state['vy_smooth']
                        speed = float(np.hypot(velocity_state['vx_smooth'], velocity_state['vy_smooth']))
                        direction_deg = float(
                            np.degrees(np.arctan2(velocity_state['vy_smooth'], velocity_state['vx_smooth'])) % 360
                        )
                velocity_state['prev_x'] = current_x
                velocity_state['prev_y'] = current_y
                velocity_state['prev_t'] = frame_time
            else:
                velocity_state['vx_smooth'] *= 0.9
                velocity_state['vy_smooth'] *= 0.9
                velocity_state['prev_x'] = None
                velocity_state['prev_y'] = None
                velocity_state['prev_t'] = None

            # ------------------------------------------------------------------
            # 1. 处理 UI 传来的控制指令 (急停/停止)
            # ------------------------------------------------------------------
            cmd_msg = tablet_server.get_next_command()
            if cmd_msg:
                cmd = cmd_msg.get("cmd", "")
                if cmd in ["stop", "emergency_stop"]:
                    current_status_text = "EMERGENCY STOP" if cmd == "emergency_stop" else "STOPPED"
                    is_tracing_active = False
                    fish_comm.send_command(cmd)

            # ------------------------------------------------------------------
            # 2. 处理 UI 传来的轨迹规划，仅在模式改变时下发一次模式切换
            # ------------------------------------------------------------------
            traj_msg = tablet_server.get_latest_trajectory()
            if traj_msg:
                plan_pts = traj_msg.get("points", [])
                if len(plan_pts) > 1 and calib_state['H'] is not None:
                    try:
                        inv_H = np.linalg.inv(calib_state['H'])
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

            # 本机绘制轨迹：像素路径转换为米制路径，10 Hz 运行现有 PID。
            local_path_error = None
            if drawn_path_state['stop_requested']:
                fish_comm.stop_now()
                drawn_path_state['stop_requested'] = False
                is_tracing_active = False
                current_status_text = "STOPPED"

            if drawn_path_state['active']:
                if calib_state['H'] is None or len(drawn_path_state['pixels']) < 2:
                    fish_comm.stop_now()
                    drawn_path_state['active'] = False
                    is_tracing_active = False
                    current_status_text = "UNCALIBRATED"
                elif current_x is None or current_y is None:
                    fish_comm.stop_now()
                    drawn_path_state['active'] = False
                    is_tracing_active = False
                    current_status_text = "TARGET LOST"
                    print("🛑 YOLO目标丢失，已停止轨迹追踪。")
                else:
                    path_input = np.float32([
                        [[point[0], point[1]]]
                        for point in drawn_path_state['pixels']
                    ])
                    path_world = cv2.perspectiveTransform(
                        path_input, calib_state['H']
                    ).reshape((-1, 2))
                    local_path_error = compute_drawn_path_error(
                        (current_x, current_y), path_world,
                        drawn_path_state['segment']
                    )
                    if local_path_error is not None:
                        drawn_path_state['segment'] = max(
                            drawn_path_state['segment'],
                            local_path_error['seg_index']
                        )
                        if local_path_error['dist_m'] <= 0.12:
                            fish_comm.stop_now()
                            drawn_path_state['active'] = False
                            is_tracing_active = False
                            current_status_text = "ARRIVED"
                            print("✅ 已到达轨迹终点，推进已停止。")
                        elif now - drawn_path_state['last_control_t'] >= 0.1:
                            fish_comm.process_tracking_error(
                                local_path_error['cross_m'],
                                local_path_error['along_m'],
                                local_path_error['dist_m']
                            )
                            drawn_path_state['last_control_t'] = now
                            is_tracing_active = True
                            current_status_text = "PROP TRACKING"

            # 获取当前实时通信频率
            tab_tx_hz, tab_rx_hz = tablet_server.get_comm_fps()
            mcu_hz = fish_comm.get_mcu_hz()

            # ==============================================================================
            # 🔴 统一绘制 OSD HUD 图层 (黑边高对比度)
            # ==============================================================================
            # 1. 坐标与识别状态显示
            if current_x is not None and current_y is not None:
                pos_str = f"AUV Pos: X:{current_x:.2f}m Y:{current_y:.2f}m [{current_status_text}]"
                draw_outlined_text(img_show, pos_str, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            else:
                pos_str = f"AUV Pos: SEARCHING... [{current_status_text}]"
                draw_outlined_text(img_show, pos_str, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2)

            # 2. 鲜黄色实时速度与方向显示
            if speed is not None and direction_deg is not None:
                speed_str = f"Speed: {speed:.2f}m/s  Dir: {direction_deg:.1f}deg"
            else:
                speed_str = "Speed: 0.00m/s  Dir: 0.0deg"
            draw_outlined_text(img_show, speed_str, (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 255), 2)

            # 🔴🔴 3. [修改重点] 在视频画面左下角清晰渲染平板发来的轨迹追踪误差 OSD 🔴🔴
            display_error = local_path_error if local_path_error is not None else err_msg
            if display_error is not None:
                cross_m = display_error.get("cross_m", 0.0)
                along_m = display_error.get("along_m", 0.0)
                dist_m = display_error.get("dist_m", 0.0)
                seg_idx = display_error.get("seg_index", 0)

                error_source = "Local Path" if local_path_error is not None else "Tablet"
                err_line1 = f"[{error_source} Tracking Error] Seg Index: #{seg_idx} | Total Dist: {dist_m:.3f}m"
                err_line2 = f" Cross (Lateral): {cross_m:+.3f}m  |  Along (Tangential): {along_m:+.3f}m"

                # 绘制双行黑边高对比度误差文本 (洋红 + 亮黄)
                draw_outlined_text(img_show, err_line1, (20, frame.shape[0] - 60),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 255), 2)
                draw_outlined_text(img_show, err_line2, (20, frame.shape[0] - 25),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)

            # 4. 右上角系统性能 & 通信频率显示
            fps_text = f"Loop: {fps_state['loop_fps']:.1f}fps | Cam: {cam.measured_fps:.1f}fps | YOLO: {yolo_infer_fps:.1f}fps"
            text_size, _ = cv2.getTextSize(fps_text, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
            fps_color = (0, 255, 0) if fps_state['loop_fps'] >= TARGET_FPS * 0.8 else (0, 165, 255)
            draw_outlined_text(img_show, fps_text, (frame.shape[1] - text_size[0] - 20, 35),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.65, fps_color, 2)

            # 硬件通信频率与曝光状态栏
            comm_text = f"Tablet: TX {tab_tx_hz}Hz/RX {tab_rx_hz}Hz | MCU(ESP32): {mcu_hz}Hz | Exp: {cam.exposure_val}"
            comm_size, _ = cv2.getTextSize(comm_text, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
            draw_outlined_text(img_show, comm_text, (frame.shape[1] - comm_size[0] - 20, 70),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 200, 0), 2)

            # 5. 手动标定提示
            if calib_state['is_calibrating']:
                calibration_text = (
                    f"MANUAL CALIBRATION {len(calib_state['pts_raw'])}/4: "
                    "Click TL->TR->BR->BL"
                )
                draw_outlined_text(img_show, calibration_text,
                                   (20, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

            # 6. 录制指示红点
            if is_recording:
                cv2.circle(img_show, (30, 30), 10, (0, 0, 255), -1)

            # ==============================================================================
            # 构建 JSON 数据并推送/保存完整的画面
            # ==============================================================================
            json_data = {
                "timestamp": frame_time,
                "track_id": track_id,
                "auv_detected": auv_id_detected,
                "pixel": {
                    "u": float(auv_pixel[0]) if auv_pixel else None,
                    "v": float(auv_pixel[1]) if auv_pixel else None
                },
                "physical": {
                    "x": round(current_x, 4) if current_x is not None else None,
                    "y": round(current_y, 4) if current_y is not None else None
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
                    "yolo_fps": round(yolo_infer_fps, 1),
                    "yolo_confidence": round(yolo_confidence, 3),
                    "clahe_enabled": use_clahe,
                    "exposure": cam.exposure_val,
                    "tablet_tx_hz": tab_tx_hz,
                    "tablet_rx_hz": tab_rx_hz,
                    "mcu_hz": mcu_hz
                }
            }

            tablet_server.send(json_data)
            mjpeg_server.update(img_show)

            if is_recording and recorder is not None:
                csv_row = [
                    f"{frame_time:.4f}",
                    f"{auv_pixel[0]:.2f}" if auv_pixel else "",
                    f"{auv_pixel[1]:.2f}" if auv_pixel else "",
                    f"{current_x:.4f}" if current_x is not None else "",
                    f"{current_y:.4f}" if current_y is not None else ""
                ]
                recorder.submit(img_show, csv_row)

            _t3 = time.perf_counter()
            if _t3 - display_state['last_t'] >= 1.0 / DISPLAY_MAX_FPS:
                cv2.imshow(window_name, img_show)
                display_state['last_t'] = _t3

            # 键盘按键响应
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                print("\n🚪 用户按下 [q] 键，准备退出...")
                fish_comm.stop_now()
                break
            elif key == ord(' '):
                drawn_path_state['active'] = False
                is_tracing_active = False
                current_status_text = "STOPPED"
                fish_comm.stop_now()
            elif key in (10, 13):
                if calib_state['H'] is None:
                    print("❌ 无法启动：请先按 T 完成四角标定。")
                elif len(drawn_path_state['pixels']) < 2:
                    print("❌ 无法启动：请先用鼠标左键拖动画出轨迹。")
                elif current_x is None or current_y is None:
                    print("❌ 无法启动：YOLO 尚未识别到鱼。")
                else:
                    start_world = cv2.perspectiveTransform(
                        np.float32([[[drawn_path_state['pixels'][0][0],
                                     drawn_path_state['pixels'][0][1]]]]),
                        calib_state['H']
                    )[0][0]
                    start_distance = float(np.hypot(
                        current_x - start_world[0], current_y - start_world[1]
                    ))
                    if start_distance > 0.40:
                        print(f"❌ 无法启动：轨迹起点距离鱼 {start_distance:.2f}m，请从鱼附近开始画。")
                    elif fish_comm.ensure_prop_mode():
                        fish_comm.vision_seq = 0
                        drawn_path_state['active'] = True
                        drawn_path_state['segment'] = 0
                        drawn_path_state['last_control_t'] = 0.0
                        is_tracing_active = True
                        current_status_text = "PROP TRACKING"
                        print("▶️ PROP 螺旋桨轨迹 PID 已启动。")
                    else:
                        fish_comm.stop_now()
                        print("❌ 无法确认机器鱼进入 PROP 模式，未启动追踪。")
            elif key == ord('['):
                cam.adjust_exposure(-1)
            elif key == ord(']'):
                cam.adjust_exposure(1)
            elif key == ord('a'):
                is_recording = not is_recording
                if is_recording:
                    timestamp_str = time.strftime("%Y%m%d_%H%M%S")
                    record_fps = round(cam.measured_fps, 1)
                    recorder = AsyncVideoRecorder(
                        path=f'{WORK_DIR}/auv_record_{timestamp_str}.mp4',
                        fourcc=fourcc,
                        fps=record_fps,
                        frame_size=(cam.real_width, cam.real_height),
                        csv_path=f'{WORK_DIR}/auv_data_{timestamp_str}.csv'
                    )
                    print(f"🔴 开始录制... 帧率: {record_fps}fps  数据保存在: {WORK_DIR}")
                else:
                    if recorder:
                        recorder.close()
                        recorder = None
                    print("⏹️ 停止录制并保存文件。")

            elif key == ord('b'):
                ms = int((time.time() * 1000) % 1000)
                img_filename = f"{WORK_DIR}/auv_frame_{time.strftime('%Y%m%d_%H%M%S')}_{ms:03d}.jpg"
                cv2.imwrite(img_filename, img_show)
                print(f"📸 抓拍成功！已保存为: {img_filename}")

            elif key == ord('t'):
                calib_state['is_calibrating'] = not calib_state['is_calibrating']
                if calib_state['is_calibrating']:
                    fish_comm.stop_now()
                    drawn_path_state['active'] = False
                    drawn_path_state['pixels'].clear()
                    is_tracing_active = False
                    current_status_text = "CALIBRATING"
                    calib_state['H'] = None
                    calib_state['manual_locked'] = False
                    calib_state['auto_locked'] = False
                    calib_state['auto_stable_count'] = 0
                    calib_state['auto_prev_points'] = None
                    calib_state['pts_raw'].clear()
                    calib_state['pts_disp'].clear()
                    pos_state['smooth_x'] = None
                    pos_state['smooth_y'] = None
                    print("🔧 进入手动标定模式，请按顺序点击四个角点（左上→右上→右下→左下）")
                else:
                    calib_state['pts_raw'].clear()
                    calib_state['pts_disp'].clear()
                    current_status_text = "UNCALIBRATED"
                    print("⏹️ 已取消手动标定；当前没有有效标定矩阵。")

            elif key == ord('c'):
                use_clahe = not use_clahe
                print(f"🔄 CLAHE 状态切换: {'【开启】' if use_clahe else '【关闭】'}")

    except Exception as e:
        print(f"❌ 运行过程中发生未捕获异常: {e}")
        import traceback
        traceback.print_exc()

    finally:
        print("\n🧹 正在清理系统资源并关闭网络服务...")
        fish_comm.stop_now()
        fish_comm.close()
        tablet_server.close()
        fish_detector.close()
        cam.release()
        if recorder:
            recorder.close()
        cv2.destroyAllWindows()
        print("✅ 所有通信接口与相机资源已安全释放，程序优雅退出。")

if __name__ == "__main__":
    main()
