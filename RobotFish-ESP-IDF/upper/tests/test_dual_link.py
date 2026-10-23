"""Offline checks only: fake UART, fake HTTP, deterministic clocks."""
import binascii
import copy
import csv
import io
import math
from pathlib import Path
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import device_client
from lora_link import LoRaLink, SOURCE, frame
import robotfish
from station import Workstation


class Serial:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs, self.writes, self.closed, self.partial = args, kwargs, [], False, False

    def write(self, data):
        self.writes.append(data)
        return len(data) - int(self.partial)

    def close(self):
        self.closed = True


def wire(data):
    assert data[:3] == bytes.fromhex('010220')
    text, crc = data[3:].rstrip(b'\n').rsplit(b',', 1)
    assert binascii.crc_hqx(text, 0xffff) == int(crc)
    return list(map(int, text.split(b',')))


class DualLinkTests(unittest.TestCase):
    def test_sender_latest_only_expiry_stop_crc_and_restart_sequence(self):
        self.assertEqual(binascii.crc_hqx(b'123456789', 0xffff), 0x29b1)
        now = [0.0]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'sequence.json'
            link = LoRaLink(Serial, path, lambda: now[0])
            link.thread = object()  # Drive the actual worker synchronously.
            self.assertFalse(link.status()['connected'])
            link.configure('192.168.4.1')
            uart = link.serial
            self.assertEqual(uart.args, ('COM28', 115200))
            link.step()
            self.assertEqual(wire(uart.writes[-1]), [102, 1, SOURCE, 1])
            link.submit(link.host, 'target', {'mode': 3, 'axes': {'surge': 10}})
            link.submit(link.host, 'target', {'mode': 3, 'axes': {'surge': -35, 'yaw': 40, 'heave': -100}})
            now[0] = .49; link.step()
            self.assertEqual(len(uart.writes), 1)
            now[0] = .5; link.step()
            self.assertEqual(wire(uart.writes[-1]), [101, 1, SOURCE, 2, 20, 3, -350, 400, -1000, 1000])
            now[0] = 1.5; link.step()
            self.assertEqual(wire(uart.writes[-1])[0], 102)  # No repeat of a motion goal.
            link.submit(link.host, 'target', {'mode': 2, 'axes': {'surge': 100}})
            now[0] = 2.6; link.step()
            self.assertEqual(wire(uart.writes[-1])[0], 102)  # Expired unsent goal discarded.
            link.submit(link.host, 'stop', {'mode': 2})
            self.assertTrue(link.submit(link.host, 'target', {'mode': 2})['waiting'])
            link.step()
            self.assertEqual(wire(uart.writes[-1])[4], 17)
            previous = link.sequence
            link.close(); link.configure('192.168.4.1'); link.step()
            self.assertEqual(link.sequence, previous + 1)
            link.serial.partial = True
            now[0] += 1; link.step()
            self.assertFalse(link.status()['connected'])
            self.assertIn('partial', link.status()['error'])
            count = link.sent; now[0] += 10; link.step()
            self.assertEqual(link.sent, count)
            link.close()

    def client(self):
        client = device_client.DeviceClient()
        host = '192.168.4.1'
        client._profiles[host] = 'robotfish'
        client._capabilities[host] = {'lora_protocol': 2}
        client._sessions[host] = {'control_session': 'browser', 'password': 'test', 'sequence': 0}
        raw = {'mode': 2, 'wifi': {'sequence': 0}, 'lora': {'healthy': True}}
        client._motion_latest[host] = raw, time.monotonic()
        class FakeLink:
            connected = True
            submitted = []
            cleared = 0
            def status(self): return {'connected': self.connected, 'host': host}
            def submit(self, host, action, payload):
                self.submitted.append((action, payload))
                return {'accepted': False, 'queued': True, 'transport': 'lora'}
            def clear_target(self): self.cleared += 1
        client.lora = FakeLink()
        calls = []
        def request(host, path, method, body, **kwargs):
            calls.append(body)
            return 200, {'accepted': True, 'sequence': body['sequence']}, None
        client._request = request
        return client, host, raw, calls

    def test_primary_fallback_recovery_and_no_old_goal_replay(self):
        client, host, raw, calls = self.client()
        axes = {'surge': 35, 'yaw': -40, 'heave': 50}
        result = client.motion_control(host, 'target', 'browser', 1, axes=axes)
        self.assertTrue(result['queued']); self.assertFalse(calls)
        raw['lora']['healthy'] = False
        result = client.motion_control(host, 'target', 'browser', 2, axes=axes)
        self.assertEqual(result['transport'], 'wifi')
        self.assertEqual(calls[-1]['axes'], axes)
        self.assertEqual(client.lora.cleared, 1)
        raw['lora']['healthy'] = True
        result = client.motion_control(host, 'target', 'browser', 3, axes={'surge': 0, 'yaw': 20, 'heave': 0})
        self.assertEqual(result['transport'], 'lora')
        self.assertEqual(len(calls), 1)
        self.assertEqual(client.lora.submitted[-1][1]['axes']['surge'], 0)
        client.lora.connected = False
        self.assertTrue(client.motion_control(host, 'target', 'browser', 4, axes=axes)['waiting'])
        self.assertEqual(len(calls), 1)  # Host also respects primary priority.
        raw['lora']['healthy'] = False
        client.motion_control(host, 'target', 'browser', 5, axes=axes)
        self.assertEqual(calls[-1]['sequence'], 2)
        before = len(calls)
        with self.assertRaises(device_client.DeviceError):
            client.motion_control(host, 'target', 'browser', 5, axes=axes)
        with self.assertRaises(device_client.DeviceError):
            client.motion_control(host, 'target', 'browser', 6, axes=dict(axes, yaw=math.nan))
        self.assertEqual(len(calls), before)

    def test_stale_telemetry_never_selects_wifi_and_route_races_keep_session(self):
        client, host, raw, calls = self.client()
        axes = {'surge': 10, 'yaw': 0, 'heave': 0}
        client._motion_latest[host] = raw, 0
        self.assertTrue(client.motion_control(host, 'target', 'browser', 1, axes=axes)['queued'])
        client.lora.connected = False
        with self.assertRaises(device_client.DeviceError):
            client.motion_control(host, 'target', 'browser', 2, axes=axes)
        self.assertFalse(calls)
        client._motion_latest[host] = raw, time.monotonic()
        raw['lora']['healthy'] = False
        client._request = lambda *args, **kwargs: (409, {'error': 'neutral_pending'}, None)
        self.assertTrue(client.motion_control(host, 'target', 'browser', 3, axes=axes)['waiting'])
        self.assertIn(host, client._sessions)
        client._request = lambda *args, **kwargs: (401, {'error': 'invalid_password'}, None)
        with self.assertRaises(device_client.DeviceError):
            client.motion_control(host, 'target', 'browser', 4, axes=axes)
        self.assertNotIn(host, client._sessions)

    def test_imu_native_axes_freshness_and_recording(self):
        raw = {'version': 1, 'lora_protocol': 2, 'wifi_control_allowed': True, 'control_protocol': 1,
               'imu': {'model': 'JY61P', 'angles_deg': {'valid': True, 'age_ms': 10, 'x': -45.25, 'y': 20.5, 'z': 120}},
               'preferred_control': 'lora', 'safety_state': 'WAIT_COMMAND'}
        view = robotfish.normalize(raw, '192.168.4.1')
        self.assertEqual([view['imu'][axis] for axis in ('roll', 'pitch', 'yaw')], [-45.25, 20.5, 120])
        station = Workstation(); station._append(view)
        csv_bytes, _ = station.export('csv')
        row = next(csv.DictReader(io.StringIO(csv_bytes.decode('utf-8-sig'))))
        self.assertEqual(row['roll_deg'], '-45.25')
        self.assertEqual(row['control_route'], 'lora')
        for change in ({'age_ms': 501}, {'valid': False}, {'z': math.nan}, {'age_ms': -1}):
            bad = copy.deepcopy(raw); bad['imu']['angles_deg'].update(change)
            view = robotfish.normalize(bad, '192.168.4.1')
            self.assertFalse(view['imu']['valid']); self.assertIsNone(view['imu']['yaw'])
            station._append(view)
            self.assertIsNone(station.rows[-1]['yaw_deg'])


if __name__ == '__main__':
    unittest.main()
