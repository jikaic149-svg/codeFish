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
from lora_link import LoRaLink, available_ports
from credentials import KEYS
import threading
import time
from diagnostics import event
from windows_network import capture as capture_network_context, gateways

DEVICE_HTTP_PORT = 80
DEVICE_STREAM_PORT = 81          # camera stream server = control port + 1
REQUEST_TIMEOUT_SECONDS = 1.5
BODY_LIMIT_BYTES = 64 * 1024
TELEMETRY_CACHE_SECONDS = .075  # Coalesce tabs/recording; normal UI polls at 10 Hz.

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
    def __init__(self, message, *, code='device_error', retryable=False):
        super().__init__(message)
        self.code, self.retryable = code, retryable


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
        self._capabilities = {}
        self._motion_latest = {}
        self._motion_guard = threading.Lock()
        self._telemetry_guard = threading.Lock()
        self._telemetry_connections = {}
        self._telemetry_latency = {}
        self._telemetry_logged = {}
        self._sample_logged = {}
        self._pid_logged = {}
        self._telemetry_sessions = {}
        self._telemetry_failures = {}
        self._manual_orders = {}
        self._radio_mode = 3
        self.control_transport = 'auto'
        self.lora = LoRaLink()

    def _robotfish_identity(self, host, *, timeout=REQUEST_TIMEOUT_SECONDS, quiet=False):
        try:
            status, data, _ = self._request(host, '/get_ip', timeout=timeout, diagnose=not quiet)
        except DeviceError:
            return None
        if status == 200 and robotfish.validate_identity(data):
            with self._guard:
                self._profiles[host] = 'robotfish'
                self._capabilities[host] = dict(data)
                self._active_host = host
            return data
        return None

    def is_robotfish(self, host=None):
        with self._guard:
            host = host or self._active_host
            return self._profiles.get(host) == 'robotfish'

    def discover_robotfish(self, candidates=()):
        """Probe only known addresses/default gateways, never an entire subnet."""
        with self._guard:
            active = self._active_host
        hosts = []
        for value in (active, *gateways(), '192.168.4.1', *candidates):
            try:
                host = display_host(value)
            except DeviceError:
                continue
            if host not in hosts:
                hosts.append(host)
        for host in hosts[:4]:
            identity = self._robotfish_identity(host, timeout=.45, quiet=True)
            if identity:
                return dict(identity, host=host)
        return None

    def auto_connect(self, host, session, *, force=False):
        """Use a saved key or the factory key after verifying RobotFish identity."""
        host = display_host(host)
        if type(force) is not bool:
            raise DeviceError('无效连接参数')
        identity = self.identity(host)
        if identity.get('firmware') != 'robotfish':
            raise DeviceError('自动连接只适用于 RobotFish', code='wrong_device')
        view = self.robot(host)
        raw = view.get('diagnostics', {})
        if self.control_transport=='lora':
            return dict(accepted=False,telemetry_only=True,session=session,sequence=0,identity=identity)
        if raw.get('output_fault') or raw.get('output_recovery') or raw.get('ota_locked'):
            raise DeviceError('设备处于故障或维护状态，请先处理设备状态', code='device_not_ready')
        with self._guard:
            saved = self._sessions.get(host) or {}
            existing = saved.get('control_session')
            sequence = max(saved.get('client_sequence', 0), saved.get('sequence', 0))
        if existing and view['control']['connected'] and view['control']['session_confirmed']:
            return dict(accepted=True, session=existing, sequence=sequence, identity=identity)
        if not force and (existing or raw.get('wifi', {}).get('session') or raw.get('remote_active') is True):
            raise DeviceError('检测到已有控制会话，请点击“连接机器鱼”接管', code='control_session_changed')
        passwords = dict.fromkeys(filter(None, (KEYS.get(host), robotfish.DEFAULT_CONTROL_PASSWORD)))
        for password in passwords:
            try:
                result = self.login(host, password, session=session)
                break
            except DeviceError as exc:
                if exc.code != 'invalid_password':
                    raise  # Unknown outcomes are never retried automatically.
        else:
            raise DeviceError('默认密码已修改，请在自定义连接中输入一次新密码', code='custom_password_required')
        return dict(result, identity=dict(identity, authenticated=True, control_session=result['session']))

    # -- low level -----------------------------------------------------------
    def _request(self, host, path, method='GET', body=None, cookie=None,
                 timeout=REQUEST_TIMEOUT_SECONDS, port=None, password=None, diagnose=True):
        address, parsed_port = parse_host(host)
        if port is None:
            port = parsed_port
        persistent = method == 'GET' and path == '/motion_status'
        headers = {'Connection': 'keep-alive' if persistent else 'close', 'Accept': 'application/json'}
        payload = None
        if body is not None:
            payload = json.dumps(body).encode()
            headers['Content-Type'] = 'application/json'
            headers['Content-Length'] = str(len(payload))
        if cookie:
            headers['Cookie'] = cookie
        if password is not None:
            headers['X-OTA-Password'] = password
        connection = self._telemetry_connections.get(host) if persistent else None
        if connection is None:
            connection = http.client.HTTPConnection(address, port, timeout=timeout)
            if persistent:
                self._telemetry_connections[host] = connection
        started, status, failure = time.monotonic(), None, None
        try:
            connection.request(method, path, body=payload, headers=headers)
            response = connection.getresponse()
            status = response.status
            raw = response.read(BODY_LIMIT_BYTES + 1)
            if len(raw) > BODY_LIMIT_BYTES:
                raise DeviceError('设备响应过大，结果未确认', code='invalid_response', retryable=True)
            try:
                data = json.loads(raw) if raw else {}
            except ValueError:
                raise DeviceError(f'设备返回了非 JSON 响应（HTTP {response.status}）', code='invalid_response', retryable=True)
            if not isinstance(data, dict):
                raise DeviceError('设备响应不是 JSON 对象，结果未确认', code='invalid_response', retryable=True)
            return response.status, data, response.getheader('Set-Cookie')
        except DeviceError as exc:
            failure = type(exc).__name__
            if persistent:
                self._telemetry_connections.pop(host, None)
                connection.close()
            raise
        except (OSError, http.client.HTTPException) as exc:
            failure = type(exc).__name__
            if diagnose:
                capture_network_context()
            if persistent:
                self._telemetry_connections.pop(host, None)
                connection.close()
            raise DeviceError(f'找不到设备 {host}：连接失败或超时', code='transport_unavailable', retryable=True) from exc
        finally:
            elapsed = round((time.monotonic() - started) * 1000, 2)
            if diagnose and (not persistent or status != 200 or failure or elapsed >= 250):
                # Metadata only: never log bodies, passwords, cookies or sessions.
                event('device_http', host=host, method=method, path=path.split('?')[0],
                      status=status, elapsed_ms=elapsed, error_type=failure,
                      action=body.get('action') if isinstance(body, dict) and
                      body.get('action') in ('acquire', 'target', 'mode', 'stop', 'attitude_on', 'attitude_off', 'yaw_pid') else None)
            if not persistent:
                connection.close()

    # -- session -------------------------------------------------------------
    def login(self, host, password, remember=False, persist=False, session=None):
        """Authenticate once; the cookie is reused for later reads."""
        host = display_host(host)
        if self.is_robotfish(host) or self._robotfish_identity(host):
            health = self._robotfish_identity(host)
            if self.control_transport=='lora':
                return dict(accepted=False,telemetry_only=True,session=session,sequence=0,online=True,host=host)
            if not health or health.get('wifi_control_allowed') is not True or health.get('control_protocol') != 1:
                raise DeviceError('此固件尚未提供 Wi-Fi 控制，请先升级固件；遥测仍可查看')
            if not isinstance(password, str) or not 1 <= len(password) < 128 or any(not 32 <= ord(c) <= 126 for c in password):
                raise DeviceError('请输入设备控制密码（1～127 个可打印 ASCII 字符）')
            if not isinstance(session, str) or not 1 <= len(session) <= 64 or any(not 32 <= ord(c) <= 126 for c in session):
                raise DeviceError('无效控制会话，请刷新上位机后重新连接')
            status, data, _ = self._request(host, '/api/v1/control', 'POST',
                {'action': 'acquire', 'session': session}, password=password)
            if status != 200 or data.get('accepted') is not True:
                raise DeviceError('控制密码错误' if status == 401 else data.get('error') or f'连接控制失败（HTTP {status}）',
                                  code='invalid_password' if status == 401 else 'connect_failed',
                                  retryable=status in (408, 429, 500, 502, 503, 504))
            with self._guard:
                self._sessions[host] = {'password': password, 'control_session': session, 'sequence': data.get('sequence', 0)}
                self._active_host = host
            if persist:
                try: KEYS.save(host, password, minimum_length=1)
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
        if host is None:
            self.lora.close()
        with self._guard:
            if host:
                self._sessions.pop(display_host(host), None)
            else:
                self._sessions.clear()
        return {'online': False}

    def shutdown(self):
        """Submit one STOP per owned session, then release the local radio."""
        with self._guard:
            sessions = [(host, saved.get('control_session')) for host, saved in self._sessions.items()
                        if saved.get('control_session')]
        confirmed = True
        for host, session in sessions:
            try:
                result = self.motion_control(host, 'stop', session, 1,
                                             client_id='exit-' + str(time.monotonic_ns()))
                confirmed = result.get('accepted') is True and confirmed
                if result.get('queued'):
                    self.lora.step()  # One existing, bounded UART submission before closing.
            except (DeviceError, OSError, ValueError) as exc:
                confirmed = False
                event('shutdown_stop_unconfirmed', host=host, error_type=type(exc).__name__)
        radio=self.lora.status()
        if radio['connected']:
            try:
                self.lora.submit(radio['host'],'stop',{'mode':self._radio_mode})
                self.lora.step()
            except (OSError,ValueError):confirmed=False
        self.forget()
        return confirmed if sessions else False if radio['connected'] else None

    def _cookie_for(self, host):
        with self._guard:
            session = self._sessions.get(host)
        return session.get('cookie') if session else None

    def wifi_control(self, host, action, session, sequence, **payload):
        """Send once. Keep credentials on transient faults; never reacquire or replay."""
        host = display_host(host)
        if not self.is_robotfish(host):
            raise DeviceError('请先检测并连接 RobotFish 控制')
        with self._guard:
            saved = self._sessions.get(host) or {}
            password = saved.get('password')
            connected = saved.get('control_session') == session and bool(password)
        if not connected:
            raise DeviceError('控制会话未连接或已被接管，请重新连接控制', code='control_session_changed')
        if saved.get('recovery_required') and action != 'stop':
            raise DeviceError('连接恢复后需先确认全停，再接受新的操作', code='recovery_required', retryable=True)
        if action not in ('target', 'mode', 'stop', 'attitude_on', 'attitude_off', 'yaw_pid','heading_target') or type(sequence) is not int or not 1 <= sequence <= 0xffffffff:
            raise DeviceError('无效控制操作或序号')
        body = dict(payload, action=action, session=session, sequence=sequence)
        status, result = None, {}
        try:
            status, result, _ = self._request(host, '/api/v1/control', 'POST', body, password=password)
            if status != 200 or result.get('accepted') is not True:
                code = result.get('error')
                transient = status in (408, 429, 500, 502, 503, 504) or code in (
                    'lora_primary_active', 'neutral_pending','heading_unavailable', 'control_busy', 'control_timeout', 'stale_sequence', 'command_expired')
                raise DeviceError('控制密码错误，请重新连接' if status == 401 else
                    result.get('error') if status == 409 else
                    result.get('error') or f'控制失败（HTTP {status}）',
                    code=code if isinstance(code, str) else f'http_{status}', retryable=transient)
        except DeviceError as exc:
            with self._guard:
                if self._sessions.get(host, {}).get('control_session') == session:
                    if exc.retryable:
                        if exc.code not in ('lora_primary_active', 'neutral_pending','heading_unavailable'):
                            self._sessions[host]['recovery_required'] = True
                    else:
                        self._sessions.pop(host, None)
            if exc.code not in ('lora_primary_active', 'neutral_pending','heading_unavailable'):
                self.lora.clear_target()
            event('control_interrupted', host=host, action=action, status=status,
                  code=exc.code, retryable=exc.retryable)
            raise
        with self._guard:
            if self._sessions.get(host, {}).get('control_session') == session:
                self._sessions[host]['sequence'] = sequence
                if action == 'stop':
                    self._sessions[host].pop('recovery_required', None)
        return dict(result, id=sequence, simulated=False)

    def lora_status(self):
        status=self.lora.status()
        with self._guard:status.update(mode=self._radio_mode,control_transport=self.control_transport)
        return status

    def set_control_transport(self,transport):
        if transport not in ('lora','auto'):raise DeviceError('无效控制通道')
        with self._guard:self.control_transport=transport

    def lora_ports(self):
        try:return {'ports':available_ports()}
        except (OSError,ValueError,ImportError) as exc:
            raise DeviceError('扫描串口失败：'+str(exc)) from exc

    def configure_lora(self, host='', **options):
        host = display_host(host) if host else ''
        try:
            with self._motion_guard:
                previous=self.lora.status()
                if options.get('connected') is False and previous['connected']:
                    self.lora.submit(previous['host'],'stop',{'mode':self._radio_mode})
                    self.lora.step()
                self.lora.configure(host, **options)
            with self._guard:
                if options.get('connected',True):self.control_transport='auto'
                raw = self._motion_latest.get(host, ({}, 0))[0]
                if raw.get('mode') in (1,2,3):self._radio_mode=raw['mode']
            self.lora.synchronize(raw.get('lora', {}).get('last_source'), raw.get('lora', {}).get('last_sequence'))
            return self.lora_status()
        except (OSError, ValueError, ImportError) as exc:
            raise DeviceError(str(exc)) from exc

    @staticmethod
    def _order_motion(saved, sequence, client_id):
        # Each page orders its own requests; the device wire sequence is shared.
        orders = saved.setdefault('client_sequences', {})
        key = client_id or ''
        previous = orders.get(key, saved.get('client_sequence', 0) if client_id is None else 0)
        if sequence <= previous:
            raise DeviceError('旧控制请求已拒绝', code='stale_client_sequence')
        orders[key] = sequence
        saved['client_sequence'] = max(saved.get('client_sequence', 0), sequence)

    def _order_request(self, saved, session, sequence, client_id):
        # Keep one ordering barrier across Wi-Fi/radio transitions and page sessions.
        if saved:self._order_motion(saved,sequence,client_id)
        self._order_motion(self._manual_orders.setdefault(session,{}),sequence,client_id)

    def control_binding(self,host,radio_session=None):
        """Use the already authenticated session for the workstation-owned vision client."""
        with self._guard:
            saved=self._sessions.get(host) or {}
            if saved.get('password') and saved.get('control_session'):
                return saved['control_session']
        radio=self.lora.status()
        if radio_session and radio.get('connected') and radio.get('host') in ('',host):
            return radio_session
        raise DeviceError('请先连接 LoRa 控制或授权 Wi-Fi 控制，再启动视觉循迹',code='control_session_changed')

    def motion_control(self, host, action, session, sequence, *, client_id=None, **payload):
        host = display_host(host) if host else ''
        if action not in ('target', 'mode', 'stop', 'attitude_on', 'attitude_off', 'yaw_pid','heading_target') or type(sequence) is not int or not 1 <= sequence <= 0xffffffff:
            raise DeviceError('无效控制操作或序号')
        if not isinstance(session,str) or not 1<=len(session)<=64 or any(not 32<=ord(c)<=126 for c in session):
            raise DeviceError('无效控制会话')
        if client_id is not None and (not isinstance(client_id, str) or not 1 <= len(client_id) <= 64 or
                                     any(not 32 <= ord(c) <= 126 for c in client_id)):
            raise DeviceError('无效页面标识')
        if action == 'target':
            axes = payload.get('axes')
            if not isinstance(axes, dict) or set(axes) != {'surge', 'yaw', 'heave'} or any(
                    type(x) not in (int, float) or not math.isfinite(x) or not -100 <= x <= 100 for x in axes.values()):
                raise DeviceError('三轴目标超出范围')
            scale = payload.get('propulsion_scale', 100)
            if type(scale) not in (int, float) or not math.isfinite(scale) or not 0 <= scale <= 100:
                raise DeviceError('推进比例超出范围')
        if action == 'mode' and (type(payload.get('mode')) is not int or payload['mode'] not in (1, 2, 3)):
            raise DeviceError('无效运动模式')
        if action=='heading_target':
            if not {'heading_cw_deg','surge_pct'}<=set(payload) or set(payload)-{'heading_cw_deg','surge_pct','propulsion_scale'}:
                raise DeviceError('航向目标只接受 heading_cw_deg、surge_pct 和可选推进比例')
            for key,lo,hi in [('heading_cw_deg',-180,180),('surge_pct',0,100),('propulsion_scale',0,100)]:
                value=payload.get(key,100)
                if type(value) not in (int,float) or not math.isfinite(value) or not lo<=value<=hi:
                    raise DeviceError('航向或推进目标超出范围')
        if action == 'yaw_pid':
            pid=payload.get('pid')
            if set(payload)!={'pid'} or not isinstance(pid,dict) or set(pid)!={'kp','ki','kd'} or any(
                    type(x) not in (int,float) or not math.isfinite(x) or not 0<=x<=100 for x in pid.values()):
                raise DeviceError('PID 的 Kp、Ki、Kd 必须是 0～100 的有限数值',code='invalid_pid')
            payload={'pid':{key:round(value,3) for key,value in pid.items()}}
        with self._guard:
            dual = self._capabilities.get(host, {}).get('lora_protocol') == 2
            if action=='heading_target' and self._capabilities.get(host,{}).get('heading_target_supported') is not True:
                raise DeviceError('当前固件不支持 S3 航向目标，请烧录 2.7.0 或更新固件',code='unsupported_control')
            if action == 'yaw_pid' and self._capabilities.get(host, {}).get('yaw_pid_supported') is not True:
                raise DeviceError('当前固件未提供 yaw PID 调参，请先烧录新固件',code='unsupported_control')
            if action in ('attitude_on', 'attitude_off') and self._capabilities.get(host, {}).get('attitude_control_supported') is not True:
                raise DeviceError('当前固件未提供陀螺仪控制，请先更新固件', code='unsupported_control')
            if action == 'attitude_on' and self._capabilities.get(host, {}).get('attitude_control_calibrated') is not True:
                raise DeviceError('请先确认 IMU 安装轴向和执行器纠偏方向，完成标定后再开启', code='attitude_not_calibrated')
        if action in ('attitude_on', 'attitude_off') and payload:
            raise DeviceError('姿态辅助开关不接受运动目标或额外参数')
        with self._motion_guard:
            link=self.lora.status()
            with self._guard:
                forced=self.control_transport=='lora'
                if forced and not (link['connected'] and (not host or link['host'] in ('',host))):
                    raise DeviceError('LoRa 串口未连接，指令未发送；Wi-Fi 仅用于遥测',code='lora_unavailable')
                saved=self._sessions.get(host) or {}
                authorized=saved.get('control_session')==session and bool(saved.get('password'))
                raw,received=self._motion_latest.get(host,({},float('-inf')))
                fresh=time.monotonic()-received<2 and host not in self._telemetry_failures
                radio_only=link['connected'] and (not host or link['host'] in ('',host)) and (
                    forced or not authorized or not fresh or saved.get('recovery_required'))
                if radio_only:
                    self._order_request(saved if authorized else None,session,sequence,client_id)
                    if action in ('mode','attitude_on','heading_target'):
                        self._radio_mode=payload['mode'] if action=='mode' else 3
                        if authorized:saved['desired_mode']=self._radio_mode
                    mode=self._radio_mode
            if radio_only:
                return dict(self.lora.submit(link['host'],action,dict(payload,mode=mode)),
                            id=sequence,session=session,mode=mode)
        if not dual:
            with self._motion_guard:
                with self._guard:
                    saved = self._sessions.get(host) or {}
                    if saved.get('control_session') != session or not saved.get('password'):
                        raise DeviceError('控制会话未连接，请重新连接', code='control_session_changed')
                    self._order_request(saved, session, sequence, client_id)
                    if action=='mode':self._radio_mode=payload['mode']
                    raw = self._motion_latest.get(host, ({}, 0))[0]
                    reported = raw.get('wifi', {}).get('sequence', 0) if raw.get('wifi', {}).get('session') == session else 0
                    wire = max(saved.get('sequence', 0), reported if type(reported) is int else 0) + 1
                return self.wifi_control(host, action, session, wire, **payload)
        with self._motion_guard:
            with self._guard:
                saved = self._sessions.get(host) or {}
                if saved.get('control_session') != session or not saved.get('password'):
                    raise DeviceError('控制会话未连接，请重新连接', code='control_session_changed')
                if saved.get('recovery_required') and action != 'stop':
                    raise DeviceError('连接恢复后需先确认全停，再接受新的操作', code='recovery_required', retryable=True)
                self._order_request(saved, session, sequence, client_id)
                raw, received = self._motion_latest.get(host, ({}, 0))
                if action in ('mode','attitude_on','heading_target'):
                    saved['desired_mode'] = payload['mode'] if action=='mode' else 3
                    self._radio_mode=saved['desired_mode']
                mode = saved.get('desired_mode', raw.get('mode', 2))
                wire = max(saved.get('sequence', 0), raw.get('wifi', {}).get('sequence', 0))
            link = self.lora.status()
            fresh = time.monotonic() - received < 2
            available = link['connected'] and link['host'] in ('',host)
            healthy = raw.get('lora', {}).get('healthy')
            lora_payload = dict(payload, mode=mode)
            if action == 'stop':
                queued = self.lora.submit(link['host'], action, lora_payload) if available else None
                try:
                    return dict(self.wifi_control(host, action, session, wire + 1), transport='wifi', lora_queued=bool(queued))
                except DeviceError:
                    if queued:
                        return dict(queued, id=sequence)
                    raise
            if available and (not fresh or healthy is not False):
                return dict(self.lora.submit(link['host'], action, lora_payload), id=sequence)
            self.lora.clear_target()
            if not fresh:
                raise DeviceError('Wi-Fi 遥测已过期，无法确认备用通道', code='telemetry_stale', retryable=True)
            if healthy is not False:
                return {'accepted': False, 'waiting': True, 'transport': 'waiting', 'id': sequence, 'simulated': False}
            try:
                if action == 'target' and raw.get('mode') != mode:
                    self.wifi_control(host, 'mode', session, wire + 1, mode=mode)
                    wire += 1
                return dict(self.wifi_control(host, action, session, wire + 1, **payload), transport='wifi')
            except DeviceError as exc:
                if str(exc) in ('lora_primary_active', 'neutral_pending'):
                    return {'accepted': False, 'waiting': True, 'transport': 'waiting', 'id': sequence, 'simulated': False}
                raise

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
            # A single telemetry connection, independent of the motion lock.
            with self._telemetry_guard:
                with self._guard:
                    raw, received = self._motion_latest.get(host, ({}, float('-inf')))
                    saved = self._sessions.get(host) or {}
                    session, sequence = saved.get('control_session'), saved.get('sequence', 0)
                    generation = saved.setdefault('_generation', object()) if session else None
                previous_failure = self._telemetry_failures.get(host)
                if previous_failure and previous_failure['generation'] is generation and time.monotonic() < previous_failure['retry_at']:
                    raise DeviceError(previous_failure['message'], code=previous_failure['code'],
                                      retryable=previous_failure['retryable'])
                if time.monotonic() - received >= TELEMETRY_CACHE_SECONDS or (
                        session and (raw.get('wifi', {}).get('session') != session or
                                     raw.get('wifi', {}).get('sequence', 0) != sequence)):
                    started = time.monotonic()
                    try:
                        status, raw, _ = self._request(host, '/motion_status')
                        if status != 200:
                            raise DeviceError(raw.get('error') or f'设备返回 HTTP {status}',
                                              code=f'http_{status}', retryable=status in (408, 429, 500, 502, 503, 504))
                        # Reject invalid payloads before publishing a new cached sample.
                        robotfish.normalize(raw, host)
                    except (DeviceError, ValueError, TypeError, AttributeError) as exc:
                        error = exc if isinstance(exc, DeviceError) else DeviceError(str(exc), code='invalid_response')
                        failures = previous_failure['attempts'] + 1 if previous_failure and previous_failure['generation'] is generation else 1
                        # Coalesce failures too: multiple tabs must not queue serial 1.5 s timeouts.
                        self._telemetry_failures[host] = dict(message=str(error), code=error.code,
                            retryable=error.retryable, attempts=failures, generation=generation,
                            retry_at=time.monotonic() + min(3, .25 * 2 ** min(failures - 1, 4)))
                        with self._guard:
                            if session and self._sessions.get(host) is saved:
                                saved['recovery_required'] = True
                        if not self.lora.status()['connected']:self.lora.clear_target()
                        event('telemetry_interrupted', host=host, code=error.code, retryable=error.retryable)
                        if error is exc:
                            raise
                        raise error from exc
                    received = time.monotonic()
                    self._telemetry_latency[host] = (received - started) * 1000
                    self._telemetry_sessions[host] = generation
                    self._telemetry_failures.pop(host, None)
                    with self._guard:
                        self._motion_latest[host] = (raw, received)
                latency = self._telemetry_latency.get(host, 0)
            try:
                # Include Wi-Fi transit and cache residence; old samples never gain freshness.
                view = robotfish.normalize(raw, host, latency + max(0, time.monotonic() - received) * 1000)
                view['telemetry_rtt_ms'] = round(latency, 2)
                with self._guard:
                    publish_sample=received>self._sample_logged.get(host,float('-inf'))
                    if publish_sample:self._sample_logged[host]=received
                if publish_sample:
                    event('telemetry_sample',host=host,rtt_ms=round(latency,2),device_uptime_ms=raw.get('uptime_ms'),
                          imu=view['imu'],actuators=view['actuators'],motion=view['motion'],
                          body_imu={axis:view.get('attitude_control',{}).get('body_'+axis+'_deg') for axis in ('roll','pitch','yaw')},
                          attitude=view.get('attitude_control'),
                          axes=view['control']['axes'],route=view['control']['route'],owner=view['owner'],
                          lora={key:raw.get('lora',{}).get(key) for key in ('healthy','accepted','rejected','last_source','last_sequence','last_code','age_ms')},
                          command_result=raw.get('command_result'),safety=view['safety_mode'])
                    feedback=view.get('attitude_control',{})
                    if feedback.get('yaw_pid_supported'):
                        state=(feedback.get('enabled'),feedback.get('pid_period_ms'),tuple(feedback['pid'].get(key) for key in ('kp','ki','kd')),view['imu'].get('filter_stage_hz'))
                        uptime=raw.get('uptime_ms',0)
                        previous=self._pid_logged.get(host)
                        if previous is None or previous[0]!=state or uptime<previous[1]:
                            event('yaw_pid_confirmed',host=host,device_uptime_ms=uptime,enabled=state[0],period_ms=state[1],
                                  pid=feedback['pid'],mode=raw.get('mode'),filter_stage_hz=state[3])
                        self._pid_logged[host]=(state,uptime)
                logged, frames, rx_bytes = self._telemetry_logged.get(host, (0, 0, 0))
                if received - logged >= 1:
                    imu = view['imu']
                    interval = received - logged if logged else 0
                    event('telemetry', host=host, rtt_ms=round(latency, 2), imu=imu,
                          frames_per_s=round(max(0, (imu['frames'] or 0) - frames) / interval, 2) if interval else None,
                          bytes_per_s=round(max(0, (imu['rx_bytes'] or 0) - rx_bytes) / interval, 2) if interval else None,
                          route=view['control']['route'], safety=view['safety_mode'], attitude=view.get('attitude_control'))
                    self._telemetry_logged[host] = (received, imu['frames'] or 0, imu['rx_bytes'] or 0)
                self.lora.synchronize(raw.get('lora', {}).get('last_source'), raw.get('lora', {}).get('last_sequence'))
                view['lora_sender'] = self.lora.status()
                with self._guard:
                    saved = self._sessions.get(host) or {}
                    session = saved.get('control_session')
                view['control']['connected'] = bool(session and raw.get('wifi', {}).get('session') == session)
                view['control']['session'] = raw.get('wifi', {}).get('session', '')
                view['control']['session_confirmed'] = not session or self._telemetry_sessions.get(host) is saved.get('_generation')
                view['control']['recovery_required'] = bool(saved.get('recovery_required'))
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
