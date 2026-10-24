"""Local diagnostics and telemetry regression; no device or serial port is opened."""
from concurrent.futures import ThreadPoolExecutor
import copy
from contextlib import redirect_stderr, redirect_stdout
import io
import json
import logging
from pathlib import Path
import queue
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import device_client
import diagnostics
import robotfish

RAW = dict(version=1, imu=dict(model='JY61P', baud=115200, uart_ready=True,
    frames=300, rx_bytes=3300, checksum_errors=0, receive_errors=0,
    angles_deg=dict(valid=True, age_ms=10, x=-45, y=20, z=120)))


class RealtimeTests(unittest.TestCase):
    def test_filtered_display_keeps_raw_values_and_does_not_revive_stale_data(self):
        raw=copy.deepcopy(RAW)
        raw['imu'].update(filtered_angles_deg=dict(valid=True,age_ms=20,x=-44,y=19,z=119),filter_stage_hz=3)
        imu=robotfish.normalize(raw,'192.168.4.1')['imu']
        self.assertEqual(imu['roll'],-44);self.assertEqual(imu['sensor_roll'],-45)
        self.assertTrue(imu['filter_active']);self.assertEqual(imu['filter_stage_hz'],3)
        old=robotfish.normalize(raw,'192.168.4.1',500)['imu']
        self.assertFalse(old['valid']);self.assertIsNone(old['roll']);self.assertEqual(old['sensor_roll'],-45)
        raw['imu']['filtered_angles_deg']['valid']=False
        self.assertFalse(robotfish.normalize(raw,'192.168.4.1')['imu']['valid'])

    def test_pid_history_survives_telemetry_rotation_and_exports_confirmed_values(self):
        import csv
        with tempfile.TemporaryDirectory() as directory:
            diagnostics.start(directory)
            try:
                diagnostics.event('yaw_pid_requested',host='192.168.4.1',pid=dict(kp=1,ki=.02,kd=.1),queued=True)
                diagnostics.event('telemetry_sample',imu={})
                diagnostics.event('yaw_pid_confirmed',host='192.168.4.1',pid=dict(kp=1,ki=.02,kd=.1),enabled=False,period_ms=100)
                diagnostics._listener.queue.join()
                data,_=diagnostics.export_pid_history(directory)
                rows=list(csv.DictReader(io.StringIO(data.decode('utf-8-sig'))))
                self.assertEqual([row['event'] for row in rows],['yaw_pid_requested','yaw_pid_confirmed'])
                self.assertEqual(rows[1]['period_ms'],'100');self.assertEqual(rows[1]['enabled'],'False')
                self.assertEqual(rows[1]['ki'],'0.02')
            finally:diagnostics.stop()

    def test_paired_diagnostics_capture_one_snapshot_without_duplicate_cached_samples(self):
        client=device_client.DeviceClient();host='192.168.4.1';client._profiles[host]='robotfish'
        raw=copy.deepcopy(RAW)
        raw.update(uptime_ms=12345,attempts=1,last_written=dict(tail_front_deg=91,tail_rear_deg=92,pectoral_left_deg=89,pectoral_right_deg=88,left_motor_pct=35,right_motor_pct=35))
        with patch.object(client,'_request',return_value=(200,raw,None)),patch('device_client.event') as event,patch('device_client.time.monotonic',return_value=100):
            client.robot(host);client.robot(host)
        samples=[call.kwargs for call in event.call_args_list if call.args[0]=='telemetry_sample']
        self.assertEqual(len(samples),1)
        self.assertEqual(samples[0]['actuators']['tail1'],91)
        self.assertEqual(samples[0]['imu']['roll'],-45)
        self.assertEqual(samples[0]['device_uptime_ms'],12345)

    def test_synchronous_export_keeps_missing_values_and_never_uses_old_imu_only_events(self):
        import csv
        with tempfile.TemporaryDirectory() as directory:
            sample=dict(event='telemetry_sample',time='2026-10-24T08:00:00Z',device_uptime_ms=10,
                        imu=dict(valid=True,roll=2,pitch=3,yaw=4),actuators=dict(tail1=91),axes={},lora={},
                        attitude=dict(enabled=True,pid_period_ms=100,pid=dict(kp=1,ki=.02,kd=.1),
                                      heading_target_deg=-179,yaw_error_deg=2,yaw_correction_pct=4))
            path=Path(directory)/'workstation.jsonl'
            path.write_text(json.dumps(dict(event='telemetry',imu={'roll':99}))+'\n'+json.dumps(sample)+'\n{partial',encoding='utf-8')
            data,_=diagnostics.export_synchronous('csv',directory)
            rows=list(csv.DictReader(io.StringIO(data.decode('utf-8-sig'))))
            self.assertEqual(len(rows),1);self.assertEqual(rows[0]['tail1_command_deg'],'91')
            self.assertEqual(rows[0]['pitch_deg'],'3');self.assertEqual(rows[0]['left_fin_command_deg'],'')
            self.assertEqual(rows[0]['yaw_pid_ki'],'0.02');self.assertEqual(rows[0]['yaw_pid_period_ms'],'100')
            self.assertEqual(rows[0]['yaw_target_left_deg'],'179');self.assertEqual(rows[0]['yaw_error_left_deg'],'-2')
    def test_capture_records_wifi_rtt_and_leaves_missing_battery_empty(self):
        from station import Workstation
        station = Workstation()
        view = robotfish.normalize(copy.deepcopy(RAW), '192.168.4.1')
        view['telemetry_rtt_ms'] = 24.5
        station._append(view)
        row = station.data()['rows'][-1]
        self.assertEqual(row['latency_ms'], 24.5)
        self.assertEqual(row['roll_deg'], -45)
        self.assertIsNone(row['battery_v'])

    def test_capture_outage_keeps_device_source_and_can_resume(self):
        from station import Workstation
        station = Workstation()
        station.host = '192.168.4.1'
        with patch('station.device_client.CLIENT.is_robotfish', return_value=True):
            station.capture('start')
            with patch('station.time.sleep', side_effect=[None, RuntimeError('one pass')]), \
                    patch.object(station, 'read', side_effect=device_client.DeviceError('fixture offline')), \
                    patch.object(station, 'poll_raw'), self.assertRaisesRegex(RuntimeError, 'one pass'):
                station._run()
            self.assertEqual(station.rows[-1]['source'], 'robotfish_http')
            self.assertFalse(station.rows[-1]['valid'])
            station.capture('stop')
            self.assertTrue(station.capture('start')['recording'])

    def test_shared_sample_age_and_slow_response_are_never_marked_fresh(self):
        client, host, now = device_client.DeviceClient(), '192.168.4.1', [100.0]
        client._profiles[host] = 'robotfish'
        calls = []
        delay = [.3]
        def request(*args, **kwargs):
            calls.append(args)
            now[0] += delay[0]
            return 200, copy.deepcopy(RAW), None
        client._request = request
        with patch('device_client.time.monotonic', side_effect=lambda: now[0]):
            first = client.robot(host)
            self.assertTrue(first['imu']['valid'])
            self.assertAlmostEqual(first['imu']['age_ms'], 310)
            now[0] += .05
            cached = client.robot(host)
            self.assertEqual(len(calls), 1)
            self.assertAlmostEqual(cached['imu']['age_ms'], 360)
            self.assertEqual(RAW['imu']['angles_deg']['age_ms'], 10)
            now[0] += .05
            delay[0] = .6
            slow = client.robot(host)
            self.assertEqual(len(calls), 2)
            self.assertFalse(slow['imu']['valid'])
            self.assertAlmostEqual(slow['imu']['age_ms'], 610)
            self.assertIsNone(slow['imu']['yaw'])

    def test_concurrent_readers_share_one_request(self):
        client, host = device_client.DeviceClient(), '192.168.4.1'
        client._profiles[host] = 'robotfish'
        entered, release = threading.Event(), threading.Event()
        calls = []
        def request(*args, **kwargs):
            calls.append(args)
            entered.set()
            self.assertTrue(release.wait(2))
            return 200, copy.deepcopy(RAW), None
        client._request = request
        with ThreadPoolExecutor(max_workers=6) as executor:
            pending = [executor.submit(client.robot, host) for _ in range(6)]
            self.assertTrue(entered.wait(2))
            release.set()
            for result in pending:
                self.assertTrue(result.result(timeout=2)['imu']['valid'])
        self.assertEqual(len(calls), 1)

    def test_telemetry_connection_reuse_and_no_retry_after_failure(self):
        class Connection:
            instances = []
            def __init__(self, *args, **kwargs):
                self.requests, self.closed, self.fail = 0, False, False
                self.instances.append(self)
            def request(self, *args, **kwargs):
                self.requests += 1
                if self.fail:
                    raise OSError('fixture disconnect')
            def getresponse(self): return self
            def read(self, _): return self.payload
            def getheader(self, _): return None
            def close(self): self.closed = True
            status = 200
            payload = b'{"version":1}'
        client = device_client.DeviceClient()
        with patch('device_client.http.client.HTTPConnection', Connection):
            for _ in range(2):
                client._request('192.168.4.1', '/motion_status')
            connection = Connection.instances[0]
            self.assertEqual(len(Connection.instances), 1)
            self.assertEqual(connection.requests, 2)
            self.assertFalse(connection.closed)
            connection.fail = True
            with self.assertRaises(device_client.DeviceError):
                client._request('192.168.4.1', '/motion_status')
            self.assertEqual(connection.requests, 3)
            self.assertEqual(len(Connection.instances), 1)
            self.assertTrue(connection.closed)
            self.assertNotIn('192.168.4.1', client._telemetry_connections)
            client._request('192.168.4.1', '/motion_status')
            connection = Connection.instances[-1]
            connection.payload = b'invalid json'
            with self.assertLogs(diagnostics.LOG, level='INFO') as captured, self.assertRaises(device_client.DeviceError):
                client._request('192.168.4.1', '/motion_status')
            self.assertIn('DeviceError', captured.output[0])
            self.assertIn('"status": 200', captured.output[0])
            self.assertTrue(connection.closed)

    def test_logs_are_quiet_bounded_and_rotated(self):
        output, errors = io.StringIO(), io.StringIO()
        hooks = sys.excepthook, threading.excepthook
        with tempfile.TemporaryDirectory() as directory:
            try:
                with redirect_stdout(output), redirect_stderr(errors):
                    diagnostics.start(directory)
                    diagnostics.start(directory)  # Idempotent initialization.
                    sink = diagnostics._listener.handlers[0]
                    self.assertEqual((sink.maxBytes, sink.backupCount), (5 * 1024 * 1024, 3))
                    sink.maxBytes = 1024
                    for index in range(100):
                        diagnostics.event('rotation_check', index=index, message='中文诊断')
                    diagnostics.event('bad_field', value=float('nan'))
                    diagnostics.stop()
                files = list(Path(directory).glob('workstation.jsonl*'))
                self.assertEqual(len(files), 4)
                rows = [json.loads(line) for path in files for line in path.read_text(encoding='utf-8').splitlines()]
                self.assertTrue(any(row.get('index') == 99 for row in rows))
                self.assertTrue(all(row['event'] == 'rotation_check' for row in rows))
                self.assertTrue(all(path.stat().st_size <= 1024 for path in files))
            finally:
                diagnostics.stop()
        self.assertEqual((sys.excepthook, threading.excepthook), hooks)
        self.assertEqual(output.getvalue() + errors.getvalue(), '')
        handler = diagnostics.QuietQueueHandler(queue.Queue(maxsize=1))
        record = logging.makeLogRecord({'msg': 'fixture'})
        with redirect_stderr(errors):
            handler.enqueue(record)
            handler.enqueue(record)
        self.assertEqual(handler.queue.qsize(), 1)
        self.assertEqual(errors.getvalue(), '')

    def test_http_logs_exclude_control_secrets(self):
        class Connection:
            status = 200
            def __init__(self, *args, **kwargs): pass
            def request(self, *args, **kwargs): pass
            def getresponse(self): return self
            def read(self, _): return b'{"accepted":true}'
            def getheader(self, _): return None
            def close(self): pass
        with patch('device_client.http.client.HTTPConnection', Connection), self.assertLogs(diagnostics.LOG, level='INFO') as captured:
            device_client.DeviceClient()._request('192.168.4.1', '/api/v1/control', 'POST',
                dict(action='target', session='secret-session'), password='secret-password', cookie='secret-cookie')
        text = '\n'.join(captured.output)
        self.assertIn('device_http', text)
        self.assertIn('target', text)
        self.assertTrue(all(secret not in text for secret in ('secret-session', 'secret-password', 'secret-cookie')))


if __name__ == '__main__':
    unittest.main()
