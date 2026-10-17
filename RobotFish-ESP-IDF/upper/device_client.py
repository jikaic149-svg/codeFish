"""Device-side HTTP client for the rlyoulink probe.

The browser never talks to the device directly: this service holds the session
cookie and relays a small, fixed set of device operations. That keeps the loopback
`Host`/`Origin`/token model intact, avoids CORS entirely, and means the device
password never reaches page JavaScript.

Only standard library is used, so the host UI runs on a bare Windows install.
"""
import http.client
import ipaddress
import json
import math
import re
import robotfish
from credentials import KEYS
import threading
import time

DEVICE_HTTP_PORT = 80
DEVICE_STREAM_PORT = 81          # camera stream server = control port + 1
REQUEST_TIMEOUT_SECONDS = 1.5
BODY_LIMIT_BYTES = 64 * 1024

# Battery condition codes, mirrored from common/spi_link_protocol.h.
BATTERY_CONDITIONS = {
    0: ('normal', '正常'),
    1: ('low', '低压'),
    2: ('high', '过压'),
    3: ('unavailable', '不可用'),
}

# Drive modes and actions, mirrored from channel/MotionTypes.h.
DRIVE_MODES = {0: '待机', 1: '仿生', 2: '螺旋桨', 3: '混合'}
MOTION_ACTIONS = {0: '停止', 1: '前进', 2: '后退', 3: '左转', 4: '右转'}


class DeviceError(RuntimeError):
    """Raised with a message that is safe and useful to show in the UI."""


def parse_host(value):
    """Split an optional :port off a private LAN address.

    The real probe serves on 80; the bundled simulator uses another port, so the
    UI may pass "127.0.0.1:8900". Returns (address, port).
    """
    text = str(value or '').strip()
    port = DEVICE_HTTP_PORT
    if ':' in text:
        text, _, suffix = text.rpartition(':')
        if not suffix.isdigit():
            raise DeviceError('端口必须是数字')
        port = int(suffix)
        if not 0 < port < 65536:
            raise DeviceError('端口超出范围')
    try:
        address = ipaddress.IPv4Address(text)
    except Exception:
        raise DeviceError('请填写有效的设备 IPv4 地址')
    if (not address.is_private or address.is_multicast or address.is_unspecified
            or int(address) == 0xFFFFFFFF):
        raise DeviceError('请使用局域网内的设备 IPv4 地址')
    # Loopback is only legitimate for the bundled simulator.
    if address.is_loopback and port == DEVICE_HTTP_PORT:
        raise DeviceError('请使用局域网内的设备 IPv4 地址')
    return str(address), port


def valid_host(value):
    """Return a normalised address, raising DeviceError when unusable."""
    return parse_host(value)[0]


def display_host(value):
    """Human-facing address, keeping a non-default port visible."""
    address, port = parse_host(value)
    return address if port == DEVICE_HTTP_PORT else f'{address}:{port}'


def normalize_telemetry(raw):
    """Preserve raw bytes; decode only the explicitly defined ADC schema."""
    if not isinstance(raw, dict) or type(raw.get('schema')) is not int or raw.get('schema') != 1:
        raise DeviceError('不支持的原始遥测版本')
    def unsigned(value):
        return type(value) is int and 0 <= value <= 0xffffffff
    for key in ('stream', 'cursor', 'lost', 'overwritten', 'source_gaps'):
        if not unsigned(raw.get(key)):
            raise DeviceError('原始遥测游标格式无效')
    if not raw['stream'] or type(raw.get('online')) is not bool or type(raw.get('reset')) is not bool:
        raise DeviceError('原始遥测来源格式无效')
    records = raw.get('records')
    if not isinstance(records, list) or len(records) > 16:
        raise DeviceError('原始遥测批次过大或无效')
    result = {key: raw[key] for key in ('schema', 'stream', 'cursor', 'lost', 'overwritten', 'source_gaps', 'online', 'reset')}
    result['records'] = []
    for entry in records:
        if not isinstance(entry, dict) or any(not unsigned(entry.get(key)) for key in
                ('sequence', 'source_sequence', 'source_epoch', 'sampled_at_ms', 'received_at_ms')):
            raise DeviceError('原始遥测采样时间或序号无效')
        kind, length, data = entry.get('kind'), entry.get('length'), entry.get('data')
        if (not entry['sequence'] or not entry['source_sequence'] or type(kind) is not int or
                not 1 <= kind <= 255 or type(length) is not int or not 1 <= length <= 32 or
                entry.get('encoding') != 'hex' or type(entry.get('valid')) is not bool or
                not isinstance(data, str) or len(data) != length * 2 or not re.fullmatch('[0-9a-fA-F]+', data)):
            raise DeviceError('原始遥测负载格式无效')
        item = {key: entry[key] for key in ('sequence', 'source_sequence', 'source_epoch',
                'sampled_at_ms', 'received_at_ms', 'kind', 'length', 'valid', 'encoding')}
        item['data'] = data.lower()
        item['type'] = {1: 'adc_raw', 2: 'imu_raw', 3: 'serial_raw'}.get(kind, 'raw_' + str(kind))
        if kind == 1 and length == 8:
            payload = bytes.fromhex(data)
            item['adc'] = dict(zip(('battery_counts', 'reference_counts', 'reference_calibration_counts',
                'reference_calibration_mv'), (int.from_bytes(payload[index:index+2], 'little') for index in range(0, 8, 2))))
        result['records'].append(item)
    return result


class DeviceClient:
    """One device at a time, keyed by address; sessions are cached per device."""

    def __init__(self):
        self._guard = threading.Lock()
        self._sessions = {}          # host -> {'cookie': str, 'password': str}
        self._active_host = None
        self._profiles = {}

    def _robotfish_identity(self, host):
        try:
            status, data, _ = self._request(host, '/get_ip')
        except DeviceError:
            return None
        if status == 200 and robotfish.validate_identity(data):
            with self._guard:
                self._profiles[host] = 'robotfish'
                self._active_host = host
            return data
        return None

    def is_robotfish(self, host=None):
        with self._guard:
            host = host or self._active_host
            return self._profiles.get(host) == 'robotfish'

    # -- low level -----------------------------------------------------------
    def _request(self, host, path, method='GET', body=None, cookie=None,
                 timeout=REQUEST_TIMEOUT_SECONDS, port=None, password=None):
        address, parsed_port = parse_host(host)
        if port is None:
            port = parsed_port
        headers = {'Connection': 'close', 'Accept': 'application/json'}
        payload = None
        if body is not None:
            payload = json.dumps(body).encode()
            headers['Content-Type'] = 'application/json'
            headers['Content-Length'] = str(len(payload))
        if cookie:
            headers['Cookie'] = cookie
        if password is not None:
            headers['X-OTA-Password'] = password
        connection = http.client.HTTPConnection(address, port, timeout=timeout)
        try:
            connection.request(method, path, body=payload, headers=headers)
            response = connection.getresponse()
            raw = response.read(BODY_LIMIT_BYTES + 1)
            if len(raw) > BODY_LIMIT_BYTES:
                raise DeviceError('设备响应过大')
            try:
                data = json.loads(raw) if raw else {}
            except ValueError:
                raise DeviceError(f'设备返回了非 JSON 响应（HTTP {response.status}）')
            return response.status, data, response.getheader('Set-Cookie')
        except DeviceError:
            raise
        except (OSError, http.client.HTTPException) as exc:
            raise DeviceError(f'找不到设备 {host}：连接失败或超时')
        finally:
            connection.close()

    # -- session -------------------------------------------------------------
    def login(self, host, password, remember=False, persist=False, session=None):
        """Authenticate once; the cookie is reused for later reads."""
        host = display_host(host)
        if self.is_robotfish(host) or self._robotfish_identity(host):
            health = self._robotfish_identity(host)
            if not health or health.get('wifi_control_allowed') is not True or health.get('control_protocol') != 1:
                raise DeviceError('此固件尚未提供 Wi-Fi 控制，请先升级固件；遥测仍可查看')
            if not isinstance(password, str) or not 1 <= len(password) < 128 or any(not 32 <= ord(c) <= 126 for c in password):
                raise DeviceError('请输入设备控制密码（1～127 个可打印 ASCII 字符）')
            if not isinstance(session, str) or not 1 <= len(session) <= 64 or any(not 32 <= ord(c) <= 126 for c in session):
                raise DeviceError('无效控制会话，请刷新上位机后重新连接')
            if persist and (len(password) < 12 or password.isdigit()):
                raise DeviceError('保存密钥至少需要12位且不能全为数字；可取消保存后连接')
            status, data, _ = self._request(host, '/api/v1/control', 'POST',
                {'action': 'acquire', 'session': session}, password=password)
            if status != 200 or data.get('accepted') is not True:
                raise DeviceError('控制密码错误' if status == 401 else data.get('error') or f'连接控制失败（HTTP {status}）')
            with self._guard:
                self._sessions[host] = {'password': password, 'control_session': session, 'sequence': data.get('sequence', 0)}
                self._active_host = host
            if persist:
                try: KEYS.save(host, password)
                except RuntimeError as exc: raise DeviceError('已连接，但密钥未保存：' + str(exc))
            return dict(data, online=True, host=host, remembered=bool(remember), key_saved=KEYS.has(host))
        if not isinstance(password, str) or not 1 <= len(password) <= 256:
            raise DeviceError('请输入有效的设备密钥')
        if persist and (len(password) < 12 or password.isdigit()):
            raise DeviceError('保存密钥至少需要12位且不能全为数字；旧设备可取消保存后登录')
        status, data, cookie = self._request(
            host, '/api/v1/login', 'POST', {'password': password})
        if status == 401:
            raise DeviceError('设备管理密码错误')
        if status != 200:
            raise DeviceError(data.get('error') or f'登录失败（HTTP {status}）')
        token = None
        if cookie:
            token = cookie.split(';', 1)[0].strip()
        if not token:
            raise DeviceError('设备没有返回会话 Cookie，无法保持登录')
        with self._guard:
            self._sessions[host] = {
                'cookie': token,
                'password': password if remember else '',
            }
            self._active_host = host
        if persist:
            try: KEYS.save(host, password)
            except RuntimeError as exc: raise DeviceError('已登录，但密钥未保存：' + str(exc))
        return {'online': True, 'host': host, 'remembered': bool(remember), 'key_saved': KEYS.has(host)}

    def forget(self, host=None):
        with self._guard:
            if host:
                self._sessions.pop(display_host(host), None)
            else:
                self._sessions.clear()
        return {'online': False}

    def _cookie_for(self, host):
        with self._guard:
            session = self._sessions.get(host)
        return session.get('cookie') if session else None

    def wifi_control(self, host, action, session, sequence, **payload):
        """One authenticated single-S3 command; failures never reconnect or retry."""
        host = display_host(host)
        if not self.is_robotfish(host):
            raise DeviceError('请先检测并连接 RobotFish 控制')
        with self._guard:
            saved = self._sessions.get(host) or {}
            password = saved.get('password')
            connected = saved.get('control_session') == session and bool(password)
        if not connected:
            raise DeviceError('控制会话未连接或已被接管，请重新连接控制')
        if action not in ('target', 'mode', 'stop') or type(sequence) is not int or not 1 <= sequence <= 0xffffffff:
            raise DeviceError('无效控制操作或序号')
        body = dict(payload, action=action, session=session, sequence=sequence)
        try:
            status, result, _ = self._request(host, '/api/v1/control', 'POST', body, password=password)
            if status != 200 or result.get('accepted') is not True:
                raise DeviceError('控制密码错误，请重新连接' if status == 401 else
                    '控制会话已失效、指令过期或设备维护中，请重新连接' if status == 409 else
                    result.get('error') or f'控制失败（HTTP {status}）')
        except DeviceError:
            with self._guard:
                if self._sessions.get(host, {}).get('control_session') == session:
                    self._sessions.pop(host, None)
            raise
        with self._guard:
            if self._sessions.get(host, {}).get('control_session') == session:
                self._sessions[host]['sequence'] = sequence
        return dict(result, id=sequence, simulated=False)

    def _reauthenticate(self, host):
        """Retry once with a remembered password after a 401."""
        with self._guard:
            session = self._sessions.get(host) or {}
            password = session.get('password')
        if not password:
            return False
        self.login(host, password, remember=True)
        return True

    def management(self, host, action, payload=None):
        """Fixed management operations; never an arbitrary HTTP or motion proxy."""
        host = display_host(host)
        if self.is_robotfish(host) or self._robotfish_identity(host):
            if action in ('status', 'identity'):
                return self.identity(host)
            raise DeviceError('单 ESP32-S3 未实现此网络管理接口；请使用固件原有配网入口')
        payload = payload or {}
        if not isinstance(payload, dict):
            raise DeviceError('管理参数格式无效')
        read = {'status': '/api/v1/status', 'networks': '/api/v1/networks',
                'identity': '/api/v1/device'}
        if action in read:
            result = self._get(host, read[action])
        else:
            endpoint = {
                'scan': ('POST', '/api/v1/scan'),
                'connect': ('POST', '/api/v1/connect'),
                'network-add': ('POST', '/api/v1/networks'),
                'network-order': ('PUT', '/api/v1/networks/order'),
                'network-delete': ('DELETE', '/api/v1/networks/'),
                'scan-result': ('GET', '/api/v1/scan/'),
                'connect-result': ('GET', '/api/v1/connect/'),
            }.get(action)
            if not endpoint:
                raise DeviceError('不支持的管理操作')
            method, path = endpoint
            if path.endswith('/'):
                identifier = payload.get('id')
                if type(identifier) is not int or not 1 <= identifier <= 0xffffffff:
                    raise DeviceError('无效操作编号')
                path += str(identifier)
            if action in ('connect', 'network-add'):
                ssid, password = payload.get('ssid'), payload.get('password')
                if not isinstance(ssid, str) or not 1 <= len(ssid.encode()) <= 32 or not isinstance(password, str) or not (password == '' or 8 <= len(password.encode()) <= 63):
                    raise DeviceError('SSID为1～32字节；Wi-Fi密码为空或8～63字节')
            if action.startswith('network-'):
                if type(payload.get('revision')) is not int or not 0 <= payload['revision'] <= 0xffffffff:
                    raise DeviceError('网络配置版本无效，请刷新列表')
            if action == 'network-order':
                ids = payload.get('ids')
                if not isinstance(ids, list) or len(ids) > 8 or len(set(ids)) != len(ids) or any(type(i) is not int or not 1 <= i <= 0xffffffff for i in ids):
                    raise DeviceError('网络优先级参数无效')
            cookie = self._cookie_for(host)
            if not cookie:
                raise DeviceError('请先在设备与设置中登录设备')
            status, result, _ = self._request(host, path, method, None if method == 'GET' else payload, cookie=cookie)
            if status == 401 and self._reauthenticate(host):
                status, result, _ = self._request(host, path, method, None if method == 'GET' else payload, cookie=self._cookie_for(host))
            if status not in (200, 201, 202):
                labels = {'dap_client_active': '请先结束调试会话再配网', 'revision_conflict': '网络列表已变化，请刷新后重试',
                          'operation_busy': '设备正在执行网络操作', 'invalid_password': '设备密钥错误'}
                code = result.get('error', '')
                raise DeviceError(labels.get(code, f'管理操作失败：{code or status}'))
        # Firmware currently includes stored Wi-Fi passwords in this response.
        # The workstation has no need to send them back to page JavaScript.
        for item in result.get('items', []):
            if 'password' in item:
                item['has_password'] = bool(item.pop('password'))
        return result

    # -- read-only views -----------------------------------------------------
    def _get(self, host, path):
        cookie = self._cookie_for(host)
        if not cookie:
            raise DeviceError('尚未登录设备，请先填写管理密码')
        status, data, _ = self._request(host, path, cookie=cookie)
        if status == 401:
            if self._reauthenticate(host):
                status, data, _ = self._request(
                    host, path, cookie=self._cookie_for(host))
            if status == 401:
                raise DeviceError('设备会话已失效，请重新填写管理密码')
        if status != 200:
            raise DeviceError(data.get('error') or f'设备返回 HTTP {status}')
        return data

    def identity(self, host):
        """Device identity and link state; works before login."""
        host = display_host(host)
        data = self._robotfish_identity(host)
        if data:
            with self._guard:
                session = dict(self._sessions.get(host) or {})
            return dict(data, host=host, ssid=data.get('sta_ssid') or data.get('ssid', ''), dap_client=False,
                        camera=False, simulator=False, authenticated=bool(session.get('control_session')),
                        control_session=session.get('control_session', ''), key_saved=KEYS.has(host),
                        telemetry_auth_required=False)
        status, data, _ = self._request(host, '/api/v1/health')
        if status != 200:
            raise DeviceError(data.get('error') or f'设备返回 HTTP {status}')
        if data.get('firmware') != 'rlyoulink':
            raise DeviceError('该地址不是 rlyoulink 探针')
        with self._guard:
            self._profiles.pop(host, None)
            self._active_host = host
        return {
            'host': host,
            'firmware': data.get('firmware', ''),
            'version': data.get('version', ''),
            'hardware': data.get('hardware', ''),
            'running_slot': data.get('running_slot', ''),
            'ota_confirmed': bool(data.get('ota_confirmed')),
            'ssid': data.get('ssid', ''),
            'sta_ip': data.get('sta_ip', ''),
            'ap_ip': data.get('ap_ip', ''),
            'dap_client': bool(data.get('dap_client')),
            'simulator': bool(data.get('simulator')),
            'authenticated': bool(self._cookie_for(host)),
            'key_saved': KEYS.has(host),
        }

    def robot(self, host):
        """Normalised robot telemetry, ready for direct display."""
        host = display_host(host)
        if self.is_robotfish(host) or self._robotfish_identity(host):
            status, raw, _ = self._request(host, '/motion_status')
            if status != 200:
                raise DeviceError(raw.get('error') or f'设备返回 HTTP {status}')
            try:
                view = robotfish.normalize(raw, host)
                with self._guard:
                    session = (self._sessions.get(host) or {}).get('control_session')
                view['control']['connected'] = bool(session and raw.get('wifi', {}).get('session') == session)
                view['control']['session'] = raw.get('wifi', {}).get('session', '')
                return view
            except (ValueError, TypeError, AttributeError) as exc:
                raise DeviceError(str(exc)) from exc
        address, port = parse_host(host)
        raw = self._get(host, '/api/v1/robot')
        view = normalize_robot(raw, host)
        view.update(camera_stream_url=f'http://{address}:{port + 1}/stream',
                    camera_capture_url=f'http://{host}/capture')
        return view

    def telemetry(self, host, after=0, stream=0):
        host = display_host(host)
        if self.is_robotfish(host):
            raise DeviceError('RobotFish 未提供 F405 原始遥测；六路输出请查看实时状态与记录')
        if any(type(value) is not int or not 0 <= value <= 0xffffffff for value in (after, stream)):
            raise DeviceError('无效遥测游标')
        return normalize_telemetry(self._get(display_host(host),
            f'/api/v1/telemetry?after={after}&stream={stream}'))

    def command(self, host, body):
        """Relay one motion command to the device.

        The device answers 202 with the queued command id; whether the STM32
        actually accepts it is reported separately through telemetry
        (command_id / command_ack / command_result) and is deliberately not
        guessed here.
        """
        host = display_host(host)
        if self.is_robotfish(host):
            raise DeviceError('RobotFish 请使用已认证的三轴 Wi-Fi 控制会话')
        if not isinstance(body, dict):
            raise DeviceError('指令格式无效')
        cookie = self._cookie_for(host)
        if not cookie:
            raise DeviceError('尚未登录设备，请先填写管理密码')
        status, data, _ = self._request(
            host, '/api/v1/robot/command', 'POST', body, cookie=cookie)
        if status == 401:
            if self._reauthenticate(host):
                status, data, _ = self._request(
                    host, '/api/v1/robot/command', 'POST', body,
                    cookie=self._cookie_for(host))
            if status == 401:
                raise DeviceError('设备会话已失效，请重新填写管理密码')
        if status == 202:
            return {'id': data.get('id', 0), 'state': data.get('state', 'queued')}
        if status == 409:
            raise DeviceError({
                'queue_full': '控制器指令队列已满，请稍后重试',
                'controller_offline': '控制器离线，指令未入队',
            }.get(data.get('error'), '控制器当前无法接受指令'))
        if status == 400:
            raise DeviceError('指令被设备校验拒绝：' + str(data.get('error', 'invalid_command')))
        raise DeviceError(data.get('error') or f'设备返回 HTTP {status}')

    def simulator(self, host, action, value=None):
        """Drive the bundled simulator's switches (never present on real hardware).

        action: 'gate' | 'fault' | 'offline' | 'status'
        """
        host = display_host(host)
        if action == 'status':
            status, data, _ = self._request(host, '/__sim/gate')
            if status != 200:
                raise DeviceError('该设备不支持仿真控制')
            return data
        if action not in ('gate', 'fault', 'offline'):
            raise DeviceError('不支持的仿真操作')
        if action == 'gate':
            payload = {'open': bool(value)}
        elif action == 'offline':
            payload = {'offline': bool(value)}
        else:
            payload = {'fault': str(value or 'none')}
        status, data, _ = self._request(host, f'/__sim/{action}', 'POST', payload)
        if status != 200:
            raise DeviceError(f'仿真设备返回 HTTP {status}')
        return data


def normalize_robot(raw, host):
    condition, condition_label = BATTERY_CONDITIONS.get(
        int(raw.get('battery_state', 3)), ('unknown', '未知'))

    def number(key, default=None):
        value = raw.get(key, default)
        return value if type(value) in (int, float) and math.isfinite(value) else default

    voltage = number('battery_voltage')
    return {
        'host': host,
        'online': bool(raw.get('online')),
        'ready': bool(raw.get('ready')),
        'wifi_control_allowed': bool(raw.get('wifi_control_allowed')),
        'camera': bool(raw.get('camera')),
        # The stream lives on the camera stream server (control port + 1);
        # /capture is registered on the management server.
        'camera_stream_url': '',
        'camera_capture_url': '',
        'link': {
            'sent': number('sent', 0),
            'received': number('received', 0),
            'invalid': number('invalid', 0),
            'errors': number('errors', 0),
            'round_trip_us': number('round_trip_us', 0),
            'clock_offset_us': number('clock_offset_us', 0),
            'peer_time_us': number('peer_time_us', 0),
            'frame_attempts': number('frame_attempts', 0),
            'frame_failures': number('frame_failures', 0),
        },
        'battery': {
            'valid': bool(raw.get('battery_valid')),
            'voltage': voltage,
            'age_ms': number('battery_age_ms'),
            'condition': condition,
            'condition_label': condition_label,
        },
        'motion': {
            'mode': int(raw.get('mode', 0)),
            'mode_label': DRIVE_MODES.get(int(raw.get('mode', 0)), '未知'),
            'action': int(raw.get('action', 0)),
            'action_label': MOTION_ACTIONS.get(int(raw.get('action', 0)), '未知'),
            'power': number('power', 0),
            'failsafe': bool(raw.get('failsafe')),
        },
        'actuators': {
            'tail1': number('tail1'),
            'tail2': number('tail2'),
            'left_fin': number('left_fin'),
            'right_fin': number('right_fin'),
            'left_esc': number('left_esc'),
            'right_esc': number('right_esc'),
            'pitch': number('pitch'),
        },
        'command': {
            'id': number('command_id', 0),
            'ack': number('command_ack', 0),
            'result': number('command_result', 0),
        },
        'checked_at': time.time(),
    }


CLIENT = DeviceClient()
