"""Explicit manual dynamics actions with passive recording and ordered STOP."""
import math
import threading
import time

import device_client
from dynamics_calibration import DynamicsRecorder, PARAMETERS


class DynamicsControl:
    def __init__(self, station, vision, recorder=None):
        self.station, self.vision = station, vision
        self.recorder = recorder or DynamicsRecorder(station, vision)
        self.guard = threading.RLock()
        self.orders = {}
        self.owner = None
        self.epoch = 0
        self.wire_sequence = 0
        self.bound_host = self.bound_session = None
        self.station_generation = None
        self.stop_confirmed = False
        self.control_error = ''

    @staticmethod
    def _settings(value):
        if not isinstance(value, dict): raise ValueError('请提供本段参数')
        payload = device_client.calibration_payload({key: value[key] for key in
            ('mode', *PARAMETERS, 'baseline') if key in value})
        motion = value.get('motion')
        if motion not in ('straight', 'left', 'right'): raise ValueError('请选择直行、左转或右转')
        result = {**payload, 'motion': motion}
        result.update(pool_width_m=3.,pool_height_m=1.6,camera_height_m=2.67)
        return result

    def _readiness(self, settings=None):
        settings = settings or self.recorder.settings or {'pool_width_m': 3., 'pool_height_m': 1.6}
        result = dict(ready=False, reason='', wifi_authorized=False, firmware_supported=False,
                      vision_ready=False, heading_ready=False, pool_matches=False, lora_healthy=False)
        host = self.station.host
        try:
            view = self.station.read()
            visual = self.vision.state()
            observed = visual.get('result') or {}
            frame_age = visual.get('frame_age_ms')
            result['firmware_supported'] = view.get('dynamics_calibration_supported') is True
            result['lora_healthy'] = (view.get('control') or {}).get('lora_healthy') is True
            result['vision_ready'] = bool(visual.get('running') and not observed.get('replay') and
                (observed.get('position') or {}).get('valid') and type(frame_age) in (int, float) and
                0 <= frame_age <= 600)
            heading = observed.get('initial_heading') or {}
            result['heading_ready'] = bool(heading.get('calibrated') and heading.get('imu_aligned') and
                                           (observed.get('body_yaw') or {}).get('usable'))
            pool = observed.get('pool') or {}
            result['pool_matches'] = bool(pool.get('perspective') and all(
                type(pool.get(key)) in (int, float) and abs(pool[key]-settings[wanted]) <= .005
                for key, wanted in (('width_m', 'pool_width_m'), ('height_m', 'pool_height_m'))))
            try:
                session = device_client.CLIENT.control_binding(host)
                result['wifi_authorized'] = bool(session)
            except (device_client.DeviceError, ValueError, OSError):
                session = None
            checks = [
                (bool(host) and view.get('online') is True, '请先连接机器鱼并等待有效遥测'),
                (result['wifi_authorized'], '请先在上位机授权 Wi-Fi 控制'),
                (result['firmware_supported'], '请先烧录 2.7.2-drive-calibration 固件并重新连接'),
                (result['vision_ready'], '请启动实时视觉、完成鱼体选择并等待新鲜画面'),
                (result['heading_ready'], '请在本页点选鱼头；完成后自动设为 0°'),
                (result['pool_matches'], '等待已有场地标定载入；首次使用才需点选四角'),
                (not (observed.get('tracking') or {}).get('active'), '请先停止自动循迹，再开始动力测量'),
            ]
            if self.recorder.active:
                checks.extend([
                    (host == self.bound_host and session == self.bound_session, '设备或授权已变化，请结束本段后重新开始'),
                    (self.station_generation == self.station.control_generation, '控制已被其他页面接管，请结束本段后重新开始'),
                    (not self.recorder.error, self.recorder.error or '采集文件写入失败'),
                    (self.recorder.phase == 'drive' or self.stop_confirmed,
                     '准备全停尚未确认，请点击全停并等待确认后再驱动'),
                ])
            for valid, reason in checks:
                if not valid:
                    result['reason'] = reason
                    return result
            if result['lora_healthy']:
                result['reason'] = '等待鱼端释放 LoRa 控制窗口；就绪后请再次人工按住动作'
            else:
                result['ready'] = True
            return result
        except (device_client.DeviceError, OSError, RuntimeError, ValueError) as exc:
            result['reason'] = str(exc)
            return result

    def state(self):
        state = self.recorder.state()
        state.update(readiness=self._readiness(state.get('settings')),
                     stop_confirmed=self.stop_confirmed, control_error=self.control_error)
        return state

    def _register(self, body):
        client, sequence = body.get('client_id'), body.get('sequence')
        if not isinstance(client, str) or not 1 <= len(client) <= 64 or any(not 32 <= ord(char) <= 126 for char in client):
            raise ValueError('无效测试页面标识')
        if type(sequence) is not int or not 1 <= sequence <= 0xffffffff:
            raise ValueError('无效操作序号')
        with self.guard:
            if sequence <= self.orders.get(client, 0): raise ValueError('操作已过期，请重新操作')
            self.orders[client] = sequence
            action = body['action']
            if action in ('release', 'stop', 'finish', 'start'):
                self.epoch += 1
            if action == 'start' and not self.recorder.active:
                self.owner = client
            if action not in ('start', 'stop') and self.owner is not None and self.owner != client:
                raise ValueError('本段由另一个测试页面操作；需要停止时请点击全停')
            return client, sequence, self.epoch

    def _current(self, ticket):
        with self.guard:
            client, sequence, epoch = ticket
            return self.orders.get(client) == sequence and self.epoch == epoch

    def _require_current(self, ticket):
        if not self._current(ticket): raise ValueError('该操作已被更新的全停或页面操作取消')

    def _invalidate_vision(self):
        with self.station.guard:
            self.station._revoke_vision()
            self.station.manual_epoch+=1
            self.station.heading_goal = None
            self.station_generation = self.station.control_generation

    def _send(self, action, payload=None, *, host=None, session=None):
        host = host or self.station.host
        session = session or device_client.CLIENT.control_binding(host)
        self.wire_sequence += 1
        return device_client.CLIENT.motion_control(host, action, session, self.wire_sequence,
            client_id='dynamics-controller', via_wifi=True, **(payload or {}))

    def _stop(self, reason):
        self.stop_confirmed = False
        self.recorder.note_event('stop_request', {'reason': reason, 'host': self.station.host})
        self._invalidate_vision()
        try:
            result = self._send('stop')
            if result.get('accepted') is not True:
                raise device_client.DeviceError('鱼端尚未确认全停命令')
            self.stop_confirmed = True
            self.control_error = ''
            self.recorder.note_event('stop_ack', result)
            return True
        except (device_client.DeviceError, OSError, RuntimeError, ValueError) as exc:
            self.control_error = '全停尚未确认：' + str(exc)
            self.recorder.note_event('stop_error', {'error': str(exc), 'reason': reason})
            return False

    def action(self, body):
        if not isinstance(body, dict) or body.get('action') not in ('start', 'drive', 'release', 'finish', 'stop'):
            raise ValueError('无效动力测量操作')
        action = body['action']
        if action == 'start' and (body.get('stationary_confirmed') is not True or body.get('heading_confirmed') is not True):
            raise ValueError('请人工确认鱼已静止、鱼头已完成标定')
        settings = self._settings(body.get('settings')) if action in ('start', 'drive') else None
        ticket = self._register(body)
        # Register before waiting for the shared sender: release invalidates a
        # preceding drive even when that drive is waiting on slow readiness I/O.
        if action in ('release', 'stop'):
            self.recorder.release()
        with self.station.command_guard:
            self._require_current(ticket)
            if action == 'start':
                if self.recorder.active: raise ValueError('请先完成当前测段')
                readiness = self._readiness(settings)
                if not readiness['ready'] and not (readiness['lora_healthy'] and
                        all(readiness[key] for key in ('wifi_authorized', 'firmware_supported', 'vision_ready', 'heading_ready', 'pool_matches')) and
                        readiness['reason'].startswith('等待鱼端释放')):
                    raise ValueError(readiness['reason'])
                host = self.station.host
                session = device_client.CLIENT.control_binding(host)
                self._require_current(ticket)
                settings.update(stationary_confirmed=True, heading_confirmed=True, head_confirmed=True)
                self.recorder.start(settings)
                self.bound_host, self.bound_session = host, session
                self._stop('segment_start')
                if device_client.CLIENT.lora_status().get('connected'):
                    device_client.CLIENT.configure_lora(host, connected=False)
                device_client.CLIENT.set_control_transport('auto')
                with self.station.guard: self.station.control_source = 'auto'
                self._require_current(ticket)
            elif action == 'drive':
                if not self.recorder.active or body.get('segment_id') != self.recorder.segment_id:
                    raise ValueError('测段已变化，请重新开始本段')
                readiness = self._readiness(settings)
                if not readiness['ready']: raise ValueError(readiness['reason'])
                if self.station.host != self.bound_host or device_client.CLIENT.control_binding(self.bound_host) != self.bound_session:
                    raise ValueError('本段设备或控制授权已变化，请重新开始')
                self._require_current(ticket)
                self._invalidate_vision()
                payload = {key: settings[key] for key in ('mode', *PARAMETERS, 'baseline')}
                self.recorder.note_event('drive_request', {'settings': settings, 'host': self.bound_host})
                self.recorder.drive(settings)
                self.stop_confirmed = False
                try:
                    self._require_current(ticket)
                    result = self._send('calibration_target', payload, host=self.bound_host, session=self.bound_session)
                    if result.get('accepted') is not True: raise device_client.DeviceError('鱼端未确认标定输出')
                    self.recorder.note_event('drive_ack', result)
                    self.control_error = ''
                    if not self._current(ticket):
                        self.recorder.release()
                        self._stop('drive_cancelled')
                        raise ValueError('动作已松开或被全停取消')
                except (device_client.DeviceError, OSError, RuntimeError, ValueError) as exc:
                    message = str(exc)
                    self.recorder.note_event('drive_error', {'error': message})
                    self.recorder.release()
                    if self._current(ticket): self._stop('drive_error')
                    self.control_error = message + ('；全停已确认' if self.stop_confirmed else '；全停未确认')
                    raise ValueError(self.control_error) from exc
            elif action in ('release', 'stop'):
                self._stop(str(body.get('reason') or action))
            elif action == 'finish':
                if not self.recorder.path or body.get('segment_id') != self.recorder.segment_id:
                    raise ValueError('测段已变化，不能完成其他测段')
                self.recorder.release()
                self._stop('segment_finish')
                self._require_current(ticket)
                self.recorder.note_event('finish_control_status', {'stop_confirmed': self.stop_confirmed,
                                                                   'error': self.control_error})
                self.recorder.finish()
        return self.state()

    def close(self):
        if self.recorder.active:
            with self.station.command_guard:
                self.recorder.release()
                self._stop('server_shutdown')
                self.recorder.note_event('shutdown_control_status', {'stop_confirmed': self.stop_confirmed,
                                                                   'error': self.control_error})
                self.recorder.finish()
        self.recorder.close()
