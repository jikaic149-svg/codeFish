"""只打开 USB 摄像头，用于观察实时画面延迟。"""

import time

import cv2


CAMERA_INDEX = 0  # 与正式 VISION 一致：外接 Global Shutter Camera
WIDTH = 1280
HEIGHT = 720
TARGET_FPS = 30
EXPOSURE = -6  # 1280x720/30 FPS default exposure step
GAIN = 100


def main():
    print("相机延迟测试：只启动摄像头，不加载 YOLO，不连接机器鱼。")
    print("快速左右移动手或让摄像头对准毫秒秒表，直接观察画面延迟。")
    print("关闭窗口、按 ESC 或 Q 即可退出。")

    camera = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    camera.set(cv2.CAP_PROP_FPS, TARGET_FPS)
    camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    camera.set(cv2.CAP_PROP_EXPOSURE, EXPOSURE)
    camera.set(cv2.CAP_PROP_GAIN, GAIN)

    if not camera.isOpened():
        print("无法打开摄像头，请检查 USB 连接或摄像头是否被其他程序占用。")
        return 1

    actual_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
    reported_fps = camera.get(cv2.CAP_PROP_FPS)
    print(
        f"摄像头已打开：{actual_width}x{actual_height}，"
        f"设备报告 {reported_fps:.1f} FPS"
    )

    window_name = "Camera Latency Test"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, actual_width, actual_height)

    smoothed_fps = 0.0
    previous_display_time = time.perf_counter()
    frame_number = 0

    try:
        while True:
            read_started = time.perf_counter()
            ok, frame = camera.read()
            read_finished = time.perf_counter()
            if not ok or frame is None:
                print("读取画面失败，继续重试。")
                cv2.waitKey(10)
                continue

            frame_number += 1
            display_time = time.perf_counter()
            interval = display_time - previous_display_time
            previous_display_time = display_time
            if interval > 0.0:
                instant_fps = 1.0 / interval
                smoothed_fps = (
                    instant_fps
                    if smoothed_fps <= 0.0
                    else 0.90 * smoothed_fps + 0.10 * instant_fps
                )

            wall_time = time.time()
            seconds = int(wall_time)
            milliseconds = int((wall_time - seconds) * 1000.0)
            clock_text = time.strftime("%H:%M:%S", time.localtime(seconds))
            read_ms = (read_finished - read_started) * 1000.0

            cv2.rectangle(frame, (8, 8), (390, 92), (20, 20, 20), -1)
            cv2.putText(
                frame,
                f"CAMERA ONLY  {actual_width}x{actual_height}",
                (18, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (80, 230, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                f"FPS {smoothed_fps:5.1f}   READ {read_ms:4.1f} ms",
                (18, 56),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                f"NOW {clock_text}.{milliseconds:03d}   #{frame_number}",
                (18, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (120, 255, 120),
                1,
                cv2.LINE_AA,
            )

            cv2.imshow(window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()
        print("相机延迟测试已退出。")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
