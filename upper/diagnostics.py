"""Private, bounded, asynchronous local diagnostics; no UI or console handler."""
import atexit
from collections import deque
import csv
from datetime import datetime, timezone
import json
import io
import logging
from logging.handlers import QueueHandler, QueueListener, RotatingFileHandler
from pathlib import Path
import queue
import sys
import threading
import traceback

LOG = logging.getLogger('robotfish.diagnostics')
LOG.addHandler(logging.NullHandler())
LOG.propagate = False
_listener = None
_hooks = None

def export_synchronous(format='csv',directory=None):
    """Export paired snapshots already logged; never poll or command a device."""
    if directory is None and _listener is not None:directory=Path(_listener.handlers[0].baseFilename).parent
    folder=Path(directory or Path(__file__).resolve().parent/'logs')
    samples=deque(maxlen=30000)
    for name in ('workstation.jsonl.3','workstation.jsonl.2','workstation.jsonl.1','workstation.jsonl'):
        path=folder/name
        if not path.is_file():continue
        with path.open(encoding='utf-8',errors='replace') as source:
            for line in source:
                try:sample=json.loads(line)
                except ValueError:continue
                if sample.get('event')!='telemetry_sample':continue
                imu=sample.get('imu',{});act=sample.get('actuators',{});axes=sample.get('axes',{});radio=sample.get('lora',{})
                feedback=sample.get('attitude') or {};pid=feedback.get('pid') or {}
                row=dict(time_utc=sample.get('time'),host=sample.get('host'),device_uptime_ms=sample.get('device_uptime_ms'),imu_valid=imu.get('valid'),
                    imu_sensor_valid=imu.get('sensor_valid'),imu_device_age_ms=imu.get('device_age_ms'),imu_age_upper_bound_ms=imu.get('age_ms'),
                    roll_deg=imu.get('roll'),pitch_deg=imu.get('pitch'),yaw_deg=imu.get('yaw'),
                    sensor_roll_deg=imu.get('sensor_roll'),sensor_pitch_deg=imu.get('sensor_pitch'),sensor_yaw_deg=imu.get('sensor_yaw'),
                    imu_filter_active=imu.get('filter_active'),imu_filter_stage_hz=imu.get('filter_stage_hz'),imu_filter_device_age_ms=imu.get('filter_device_age_ms'),
                    mode=sample.get('motion',{}).get('mode'),owner=sample.get('owner'),route=sample.get('route'),safety=sample.get('safety'),wifi_http_rtt_ms=sample.get('rtt_ms'),
                    imu_frames=imu.get('frames'),imu_checksum_errors=imu.get('checksum_errors'),imu_receive_errors=imu.get('receive_errors'),
                    lora_source=radio.get('last_source'),lora_sequence=radio.get('last_sequence'),lora_rx_age_ms=radio.get('age_ms'))
                for name in ('tail1','tail2','left_fin','right_fin'):row[name+'_command_deg']=act.get(name)
                for name in ('roll','pitch','yaw'):row['body_'+name+'_deg']=sample.get('body_imu',{}).get(name)
                for name in ('left_esc','right_esc'):row[name+'_pct']=act.get(name)
                for name in ('surge','yaw','heave'):row[name+'_target_pct']=axes.get(name)
                row.update(yaw_loop_enabled=feedback.get('enabled'),yaw_pid_period_ms=feedback.get('pid_period_ms'),
                    yaw_heading_locked=feedback.get('heading_locked'),yaw_loop_reason=feedback.get('reason'),
                    yaw_target_left_deg=-feedback['heading_target_deg'] if feedback.get('heading_target_deg') is not None else None,
                    yaw_error_left_deg=-feedback['yaw_error_deg'] if feedback.get('yaw_error_deg') is not None else None,
                    yaw_correction_right_pct=feedback.get('yaw_correction_pct'))
                row.update(tilt_preview_valid=feedback.get('tilt_preview_valid'),tilt_outputs_connected=feedback.get('tilt_outputs_connected'),
                    pitch_preview_body_pct=feedback.get('pitch_preview_pct'),roll_preview_body_pct=feedback.get('roll_preview_pct'))
                for name in ('kp','ki','kd'):row['yaw_pid_'+name]=pid.get(name)
                for name in ('p','i','d'):row['yaw_pid_'+name+'_right_pct']=feedback.get('yaw_'+name)
                samples.append(row)
    if not samples:raise ValueError('没有同步样本；需已有设备遥测。旧日志未保存的舵机角度无法还原。')
    if format=='json':return json.dumps(dict(schema=1,kind='paired_actuator_commands_imu',samples=list(samples)),ensure_ascii=False,indent=2).encode(),'application/json'
    if format!='csv':raise ValueError('请选择 CSV 或 JSON')
    output=io.StringIO(newline='');writer=csv.DictWriter(output,fieldnames=list(samples[0]));writer.writeheader()
    writer.writerows({key:"'"+value if isinstance(value,str) and value.startswith(('=','+','-','@')) else value for key,value in row.items()} for row in samples)
    return output.getvalue().encode('utf-8-sig'),'text/csv; charset=utf-8'


def export_pid_history(directory=None):
    folder=Path(directory or (Path(_listener.handlers[0].baseFilename).parent if _listener else Path(__file__).resolve().parent/'logs'))
    path=folder/'yaw-pid-history.jsonl'
    rows=deque(maxlen=30000)
    if path.is_file():
        with path.open(encoding='utf-8',errors='replace') as source:
            for line in source:
                try:item=json.loads(line)
                except ValueError:continue
                pid=item.get('pid') or {}
                rows.append(dict(time_utc=item.get('time'),event=item.get('event'),host=item.get('host'),
                    device_uptime_ms=item.get('device_uptime_ms'),enabled=item.get('enabled'),period_ms=item.get('period_ms'),filter_stage_hz=item.get('filter_stage_hz'),
                    kp=pid.get('kp'),ki=pid.get('ki'),kd=pid.get('kd'),mode=item.get('mode'),
                    transport=item.get('transport'),queued=item.get('queued'),accepted=item.get('accepted')))
    if not rows:raise ValueError('尚无 PID 参数记录；连接新固件或应用参数后自动记录。')
    output=io.StringIO(newline='');writer=csv.DictWriter(output,fieldnames=list(rows[0]));writer.writeheader()
    writer.writerows({key:"'"+value if isinstance(value,str) and value.startswith(('=','+','-','@')) else value for key,value in row.items()} for row in rows)
    return output.getvalue().encode('utf-8-sig'),'text/csv; charset=utf-8'


def active():
    return _listener is not None


class QuietQueueHandler(QueueHandler):
    def enqueue(self, record):
        try:
            self.queue.put_nowait(record)
        except queue.Full:
            pass  # Disk stalls must never block telemetry or motion commands.


class QuietFileHandler(RotatingFileHandler):
    def handleError(self, record):
        pass


def event(name, **fields):
    if LOG.isEnabledFor(logging.INFO):
        try:
            message = json.dumps(dict(time=datetime.now(timezone.utc).isoformat(timespec='milliseconds'),
                                      event=name, **fields), ensure_ascii=False, allow_nan=False)
        except (TypeError, ValueError):
            return
        LOG.info(message)


def start(directory=None):
    global _listener, _hooks
    if _listener is not None:
        return
    directory = Path(directory or Path(__file__).resolve().parent / 'logs')
    directory.mkdir(parents=True, exist_ok=True)
    sink = QuietFileHandler(directory / 'workstation.jsonl', maxBytes=5 * 1024 * 1024,
                            backupCount=3, encoding='utf-8')
    # Parameter changes have a separate append-only history, independent of telemetry rotation.
    history = QuietFileHandler(directory / 'yaw-pid-history.jsonl', encoding='utf-8')
    history.addFilter(lambda record:'"event": "yaw_pid_' in record.getMessage())
    handler = QuietQueueHandler(queue.Queue(maxsize=1024))
    LOG.handlers = [handler]
    LOG.setLevel(logging.INFO)
    _listener = QueueListener(handler.queue, sink, history)
    _listener.start()
    atexit.register(stop)
    _hooks = sys.excepthook, threading.excepthook
    def crash(kind, trace, **fields):
        event('uncaught_error', error_type=kind.__name__, **fields,
              stack=[dict(file=Path(frame.filename).name, function=frame.name, line=frame.lineno)
                     for frame in traceback.extract_tb(trace)])
    sys.excepthook = lambda kind, value, trace: crash(kind, trace)
    threading.excepthook = lambda args: crash(args.exc_type, args.exc_traceback, thread=args.thread.name)


def stop():
    global _listener, _hooks
    if _listener is not None:
        listener, _listener = _listener, None
        LOG.handlers = [logging.NullHandler()]
        if _hooks is not None:
            sys.excepthook, threading.excepthook = _hooks
            _hooks = None
        listener.queue.join()
        listener.stop()
        for sink in listener.handlers:
            sink.close()
