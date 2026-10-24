"""Single-S3 telemetry and ESP-IDF application OTA; stdlib only."""
import hashlib
import http.client
import json
import math
import struct
import time

FIRMWARE = 'robotfish'
HARDWARE = 'robotfish-esp32s3'
PROJECT = 'robotfish'
DEFAULT_CONTROL_PASSWORD = 'fishota'  # Matches the RobotFish firmware's factory configuration.
MAX_IMAGE_BYTES = 0x330000


def validate_identity(data):
    return (isinstance(data, dict) and data.get('firmware') == FIRMWARE
            and data.get('hardware') == HARDWARE and data.get('project_name') == PROJECT)


def normalize(raw, host, elapsed_ms=0):
    if not isinstance(raw, dict) or type(raw.get('version')) is not int or raw['version'] != 1:
        raise ValueError('不支持的 RobotFish 运动诊断版本')
    def number(value):
        return value if type(value) in (int, float) and math.isfinite(value) else None
    frame = raw.get('last_written', {})
    attempts = number(raw.get('attempts')) or 0
    actuators = {name: number(frame.get(key)) if attempts else None for name, key in (
        ('tail1', 'tail_front_deg'), ('tail2', 'tail_rear_deg'),
        ('left_fin', 'pectoral_left_deg'), ('right_fin', 'pectoral_right_deg'),
        ('left_esc', 'left_motor_pct'), ('right_esc', 'right_motor_pct'))}
    actuators['pitch'] = None
    target = raw.get('remote_target', {})
    dual = raw.get('lora_protocol') == 2
    radio = raw.get('lora', {}) if dual else raw.get('wifi', raw.get('lora', {}))
    sensor = raw.get('imu', {})
    angles = sensor.get('angles_deg', {})
    age = number(angles.get('age_ms'))
    if age is not None and age >= 0:
        age += max(0, elapsed_ms)
    attitude = angles.get('valid') is True and age is not None and 0 <= age <= 500
    attitude = attitude and all(number(angles.get(axis)) is not None for axis in ('x', 'y', 'z'))
    imu = {'model': sensor.get('model', ''), 'valid': attitude, 'age_ms': age,
           'roll': number(angles.get('x')) if attitude else None,
           'pitch': number(angles.get('y')) if attitude else None,
           'yaw': number(angles.get('z')) if attitude else None,
           'uart_ready': sensor.get('uart_ready') is True,
           'rx_bytes': number(sensor.get('rx_bytes')),
           'baud': number(sensor.get('baud')), 'frames': number(sensor.get('frames')),
           'checksum_errors': number(sensor.get('checksum_errors')),
           'receive_errors': number(sensor.get('receive_errors'))}
    feedback = raw.get('attitude_control', {})
    feedback_age = number(feedback.get('age_ms'))
    if feedback_age is not None:
        feedback_age += max(0, elapsed_ms)
    maximum_age = number(feedback.get('maximum_age_ms'))
    feedback_valid = feedback.get('body_valid') is True and feedback_age is not None and maximum_age is not None and 0 <= feedback_age <= maximum_age
    stabilizer = dict(supported=raw.get('attitude_control_supported') is True,
        enabled=feedback.get('enabled') is True, reason=feedback.get('reason', 'unavailable'),
        pitch_calibrated=feedback.get('pitch_calibrated') is True, roll_calibrated=feedback.get('roll_calibrated') is True,
        yaw_calibrated=feedback.get('yaw_calibrated') is True,
        pitch_active=feedback.get('pitch_active') is True and feedback_valid,
        roll_active=feedback.get('roll_active') is True and feedback_valid,
        yaw_active=feedback.get('yaw_active') is True and feedback_valid,
        heading_valid=feedback.get('heading_valid') is True and feedback_valid,
        heading_locked=feedback.get('heading_locked') is True and feedback_valid,
        manual_yaw=feedback.get('manual_yaw') is True,
        age_ms=feedback_age, maximum_age_ms=maximum_age, body_valid=feedback_valid,
        body_roll_deg=number(feedback.get('body_roll_deg')) if feedback_valid else None,
        body_pitch_deg=number(feedback.get('body_pitch_deg')) if feedback_valid else None,
        body_yaw_deg=number(feedback.get('body_yaw_deg')) if feedback_valid and feedback.get('heading_valid') is True else None,
        heading_target_deg=number(feedback.get('heading_target_deg')) if feedback_valid and feedback.get('heading_locked') is True else None,
        pitch_correction_pct=number(feedback.get('pitch_correction_pct')) if feedback_valid else None,
        roll_correction_pct=number(feedback.get('roll_correction_pct')) if feedback_valid else None,
        yaw_correction_pct=number(feedback.get('yaw_correction_pct')) if feedback_valid else None)
    source = raw.get('source', 'unknown')
    failsafe = raw.get('failsafe_active')
    mode, action = raw.get('mode'), raw.get('action')
    wifi = raw.get('wifi_control_allowed') is True and raw.get('control_protocol') == 1
    safety = ('LEASE_EXPIRED' if failsafe is True else 'RUNNING' if raw.get('remote_active') is True else 'WAIT_COMMAND') if wifi else ('LEASE_EXPIRED' if failsafe is True else 'UNKNOWN')
    return {'host': host, 'firmware': FIRMWARE, 'online': True, 'ready': None,
            'wifi_control_allowed': wifi, 'dual_link': dual, 'imu': imu, 'attitude_control': stabilizer, 'camera': False,
            'camera_stream_url': '', 'camera_capture_url': '',
            'battery': {'valid': False, 'voltage': None, 'age_ms': None,
                        'condition': 'unknown', 'condition_label': '未上报'},
            'motion': {'mode': mode, 'mode_label': {0:'待机',1:'仿生',2:'螺旋桨',3:'混合'}.get(mode,'未上报'),
                       'action': action, 'action_label': {0:'停止',1:'前进',2:'后退',3:'左转',4:'右转'}.get(action,'未上报'),
                       'power': abs(number(target.get('surge_pct')) or 0), 'failsafe': failsafe if type(failsafe) is bool else None},
            'actuators': actuators, 'command': {}, 'checked_at': time.time(),
            'link': {'received': number(radio.get('accepted')), 'sent': None,
                     'errors': number(radio.get('rejected', radio.get('tx_errors'))), 'round_trip_us': None,
                     'frame_attempts': attempts, 'frame_failures': number(raw.get('failures'))},
            'owner': source if source in ('lora', 'vision', 'wifi') else 'none' if wifi else 'unknown',
            'safety_mode': raw.get('safety_state', safety), 'telemetry_source': 'robotfish_http',
            'reason': ('指令租约到期：双桨中立、舵机回中' if failsafe is True else
                       'LoRa 主控，失联后 Wi-Fi 备用；IMU 姿态经 Wi-Fi 回传' if dual else
                       '2.4G Wi-Fi 运动控制与遥测；六路数值为驱动写入目标，非位置反馈' if wifi else
                       '单 ESP32-S3 实际驱动写入目标（非位置反馈）；电池状态未上报；LoRa 接收运动指令'),
            'control': {'mode': mode, 'source': source, 'route': raw.get('preferred_control', 'wifi' if wifi else 'lora'),
                        'motion_timeout_ms': number(raw.get('motion_timeout_ms')),
                        'lora_healthy': raw.get('lora', {}).get('healthy'), 'lora_age_ms': raw.get('lora', {}).get('age_ms'), 'axes': {
                name: number(target.get(key)) for name, key in (
                    ('surge', 'surge_pct'), ('yaw', 'yaw_pct'), ('heave', 'heave_pct'))}},
            'diagnostics': raw}


def inspect_image(data):
    """Check ESP-IDF headers, bounds, checksum and optional appended SHA256.

    Layout comes from the installed IDF esp_app_format.h / esp_app_desc.h.
    This detects corruption and wrong targets; it does not claim a signature.
    """
    if not 288 <= len(data) <= MAX_IMAGE_BYTES:
        raise ValueError('应用镜像大小须为 288 字节至 0x330000 字节')
    if data[0] != 0xe9 or not 1 <= data[1] <= 16 or struct.unpack_from('<H', data, 12)[0] != 9:
        raise ValueError('请选择 ESP32-S3 应用 BIN；不接受整片、bootloader 或其他芯片镜像')
    if data[23] not in (0, 1):
        raise ValueError('ESP-IDF 镜像哈希标记无效')
    offset, checksum = 24, 0xef
    for index in range(data[1]):
        if offset + 8 > len(data):
            raise ValueError('镜像段头被截断')
        _, length = struct.unpack_from('<II', data, offset)
        offset += 8
        if not length or length % 4 or offset + length > len(data):
            raise ValueError('镜像段长度无效或已截断')
        if index == 0:
            if length < 256 or struct.unpack_from('<I', data, offset)[0] != 0xabcd5432:
                raise ValueError('缺少 ESP-IDF 应用描述符；请选择应用固件')
            def field(start):
                value = data[offset + start:offset + start + 32]
                if b'\0' not in value:
                    raise ValueError('应用描述符字符串无效')
                return value.split(b'\0', 1)[0].decode('utf-8', errors='strict')
            version, project = field(16), field(48)
            if project != PROJECT or not version:
                raise ValueError('应用项目名必须为 robotfish，且必须包含版本')
        for byte in data[offset:offset + length]:
            checksum ^= byte
        offset += length
    checksum_offset = (offset // 16 + 1) * 16 - 1
    end = checksum_offset + 1
    if end > len(data) or data[checksum_offset] != checksum:
        raise ValueError('ESP-IDF 镜像校验和错误')
    if data[23]:
        if len(data) != end + 32 or hashlib.sha256(data[:end]).digest() != data[end:]:
            raise ValueError('ESP-IDF 镜像 SHA256 不匹配或长度无效')
    elif len(data) != end:
        raise ValueError('镜像尾部长度无效；不接受合并镜像')
    return {'firmware': FIRMWARE, 'hardware': HARDWARE, 'project_name': project,
            'version': version, 'size': len(data), 'format': 'bin', 'signed': False,
            'sha256': hashlib.sha256(data).hexdigest()}


def request(host, path, data=None, password=None, timeout=5):
    # Caller validates host through device_client.display_host (private IPv4 only).
    headers = {'Connection': 'close', 'Accept': 'application/json'}
    if data is not None:
        headers.update({'Content-Type': 'application/octet-stream', 'Content-Length': str(len(data))})
    if password is not None:
        headers['X-OTA-Password'] = password
    connection = http.client.HTTPConnection(host, timeout=timeout)
    try:
        connection.request('POST' if data is not None else 'GET', path, body=data, headers=headers)
        response = connection.getresponse()
        payload = response.read(65537)
        if len(payload) > 65536:
            raise ValueError('设备响应过大')
        if response.status == 401:
            raise ValueError('OTA 密码错误或设备拒绝授权')
        try:
            result = json.loads(payload)
        except ValueError as exc:
            raise ValueError(f'设备返回 HTTP {response.status}，未提供有效 JSON；请检查固件与设备状态') from exc
        if response.status != 200 or not isinstance(result, dict):
            raise ValueError(str(result.get('error', '设备响应错误')) if isinstance(result, dict) else '设备响应格式无效')
        return result
    finally:
        connection.close()


def install(host, password, data, log, progress):
    manifest = inspect_image(data)
    if not isinstance(password, str) or not 1 <= len(password.encode('utf8')) < 128 or any(ord(c) < 32 or ord(c) > 126 for c in password):
        raise ValueError('请输入有效 OTA 密码（1～127 个可打印 ASCII 字符）')
    health = request(host, '/get_ip')
    if not validate_identity(health):
        raise ValueError('设备身份不匹配：本 BIN 仅适用于 robotfish-esp32s3')
    old_slot = health.get('running_slot')
    if old_slot not in ('app0', 'app1') or not health.get('runtime_ready') or not health.get('ota_confirmed'):
        raise ValueError('设备启动尚未确认或 OTA 槽未知，请先检查设备')
    progress('上传 RobotFish 应用；等待设备确认写入')
    log('RobotFish Wi-Fi OTA：ESP32-S3 / robotfish 镜像检查通过；开始写备用槽')
    result = request(host, '/ota_upload', data, password, timeout=120)
    if result.get('success') is not True:
        raise ValueError('设备没有确认 OTA 提交')
    progress('已提交，等待重启、换槽及运行确认')
    for _ in range(40):
        time.sleep(1)
        try:
            health = request(host, '/get_ip', timeout=1)
        except (OSError, ValueError, http.client.HTTPException):
            continue
        if (validate_identity(health) and health.get('version') == manifest['version']
                and health.get('running_slot') in ('app0', 'app1')
                and health['running_slot'] != old_slot and health.get('ota_confirmed') is True
                and health.get('runtime_ready') is True):
            return {'complete': True, 'version': manifest['version'],
                    'slot': health['running_slot'], 'firmware': FIRMWARE}
    raise RuntimeError('镜像已提交，但未确认版本、换槽和健康启动；请检查设备，不能视为升级完成')
