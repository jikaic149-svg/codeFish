"""Bounded fake-only checks of the manual calibration request lifecycle."""
import json
from pathlib import Path
import shutil
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import dynamics_calibration as recording
import dynamics_control as control


class DynamicsControlTests(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parents[2] / 'build' / ('dynamics-control-test-' + uuid.uuid4().hex)
        self.root.mkdir(parents=True)
        self.clock = 2000000000.
        self.frame = 0
        self.x = 0.
        self.outputs_active = False
        self.station = SimpleNamespace(host='192.168.1.42', guard=threading.RLock(),
            command_guard=threading.RLock(), control_generation=0, vision_sessions={},
            heading_goal=None, control_source='lora', read=self.fish)
        self.vision = SimpleNamespace(state=self.image)
        self.recorder = recording.DynamicsRecorder(self.station, self.vision, self.root)
        self.controller = control.DynamicsControl(self.station, self.vision, self.recorder)
        self.client = Mock()
        self.client.control_binding.return_value = 'authorized-session'
        self.client.lora_status.return_value = {'connected': False}
        self.client.motion_control.side_effect = self.send
        self.patches = [patch.object(recording.time, 'time', side_effect=lambda: self.clock),
            patch.object(control.device_client, 'CLIENT', self.client),
            patch.object(self.recorder, '_sample_loop')]
        for item in self.patches: item.start()
        self.settings = dict(mode=3, motion='right', tail_drive_pct=20., tail_bias_pct=10.,
            prop_surge_pct=0., prop_diff_pct=20., baseline=False,
            pool_width_m=3., pool_height_m=1.6, camera_height_m=2.8)

    def tearDown(self):
        self.recorder.close()
        for item in reversed(self.patches): item.stop()
        workspace = Path(__file__).resolve().parents[2]
        if self.root.resolve().is_relative_to(workspace / 'build'): shutil.rmtree(self.root)

    def fish(self):
        return {'online': True, 'checked_at': self.clock, 'dynamics_calibration_supported': True,
            'control': {'lora_healthy': False}, 'imu': {'yaw': 4., 'sensor_yaw': 5., 'valid': True},
            'diagnostics': {'uptime_ms': int(self.clock*1000),
                'dynamics_calibration': {'active': self.outputs_active, **{key: 0 for key in recording.PARAMETERS}}},
            'actuators': {'left_esc': 20. if self.outputs_active else 0.,
                'right_esc': -20. if self.outputs_active else 0., 'tail1': 90., 'tail2': 90.}}

    def image(self):
        return {'running': True, 'frame_age_ms': 0, 'result': {'frame': self.frame,
            'camera_sample_time': self.clock, 'camera_receipt_time': self.clock,
            'position': {'valid': True, 'x_m': self.x, 'y_m': 0., 'sample_time': self.clock},
            'optical': {'valid': True, 'world_deg': 0.},
            'initial_heading': {'calibrated': True, 'imu_aligned': True, 'visual_zero_world_deg': 0.},
            'body_yaw': {'usable': True, 'yaw_deg': 0.},
            'pool': {'width_m': 3., 'height_m': 1.6, 'perspective': {'valid': True}},
            'tracking': {'active': False}, 'replay': False}}

    def send(self, host, action, session, sequence, **payload):
        self.outputs_active = action == 'calibration_target'
        return {'accepted': True, 'action': action, 'sequence': sequence}

    def command(self, action, sequence, **extra):
        body = {'action': action, 'client_id': 'fake-page', 'sequence': sequence,
            'segment_id': self.recorder.segment_id, **extra}
        if action in ('start', 'drive'): body['settings'] = dict(self.settings)
        if action == 'start': body.update(stationary_confirmed=True, heading_confirmed=True)
        return self.controller.action(body)

    def sample(self, delay=.1):
        self.clock += delay; self.frame += 1; self.x += .01
        with self.recorder.lock:
            self.recorder._ingest('fish', self.fish())
            self.recorder._ingest('vision', self.image())

    def spawn_call(self, action, sequence):
        result = {}
        def run():
            try: result['state'] = self.command(action, sequence)
            except Exception as error: result['error'] = error
        thread = threading.Thread(target=run)
        thread.start()
        return thread, result

    def actions(self):
        return [call.args[1] for call in self.client.motion_control.call_args_list]

    def test_full_manual_duration_and_release_keeps_sampling_until_finish(self):
        self.assertTrue(self.command('start', 1)['readiness']['ready'])
        self.sample()
        self.command('drive', 2)
        self.sample(17.4)
        self.assertEqual(self.actions(), ['stop', 'calibration_target'])
        self.assertTrue(self.outputs_active)
        drive = self.client.motion_control.call_args_list[1]
        self.assertEqual(drive.kwargs, dict(client_id='dynamics-controller', via_wifi=True,
            mode=3, tail_drive_pct=20., tail_bias_pct=10., prop_surge_pct=0., prop_diff_pct=20., baseline=False))
        self.assertEqual(self.recorder.phase, 'drive')
        self.command('release', 3)
        released = self.recorder._release_at
        count = self.recorder.samples['vision']
        self.sample(.3); self.sample(.3)
        self.assertEqual(self.recorder.samples['vision'], count+2)
        self.assertTrue(self.recorder.active)
        self.assertEqual(self.recorder.phase, 'coast')
        self.command('release', 4)
        self.assertEqual(self.recorder._release_at, released)
        self.sample(.3)
        state = self.command('finish', 5)
        self.assertTrue(state['saved'], state['error'])
        self.assertEqual(self.recorder._release_at, released)
        self.assertEqual(self.actions(), ['stop', 'calibration_target', 'stop', 'stop', 'stop'])
        self.assertFalse(self.outputs_active)
        events = [json.loads(line) for line in (self.recorder.path/'events.jsonl').read_text(encoding='utf-8').splitlines()]
        self.assertEqual(sum(item['kind']=='release_phase' for item in events), 1)
        self.assertIsNotNone(state['summary']['output_zero_time_s'])
        self.assertEqual([item['kind'] for item in events if item['kind'].startswith('drive_')],
                         ['drive_request', 'drive_phase', 'drive_ack'])

    def test_higher_release_sequence_rejects_delayed_drive(self):
        self.command('start', 1)
        self.command('release', 3)
        with self.assertRaises(ValueError): self.command('drive', 2)
        self.assertNotIn('calibration_target', self.actions())

    def test_release_cancels_drive_waiting_for_readiness(self):
        self.command('start', 1)
        entered, proceed, released = threading.Event(), threading.Event(), threading.Event()
        real_readiness = self.controller._readiness
        def delayed(settings=None):
            entered.set()
            if not proceed.wait(3): raise RuntimeError('test readiness gate timed out')
            return real_readiness(settings)
        real_release = self.recorder.release
        def observed_release():
            result = real_release(); released.set(); return result
        with patch.object(self.controller, '_readiness', side_effect=delayed), patch.object(self.recorder, 'release', side_effect=observed_release):
            drive_thread, drive_result = self.spawn_call('drive', 2)
            self.assertTrue(entered.wait(3))
            release_thread, release_result = self.spawn_call('release', 3)
            self.assertTrue(released.wait(3))
            proceed.set()
            drive_thread.join(3); release_thread.join(3)
            self.assertFalse(drive_thread.is_alive()); self.assertFalse(release_thread.is_alive())
        self.assertIsInstance(drive_result.get('error'), ValueError)
        self.assertNotIn('error', release_result)
        self.assertNotIn('calibration_target', self.actions())
        self.assertEqual(self.actions()[-1], 'stop')

    def test_inflight_drive_is_followed_by_stop_after_release(self):
        self.command('start', 1)
        entered, proceed, released = threading.Event(), threading.Event(), threading.Event()
        def delayed(host, action, session, sequence, **payload):
            if action == 'calibration_target':
                entered.set()
                if not proceed.wait(3): raise RuntimeError('test send gate timed out')
            return self.send(host, action, session, sequence, **payload)
        self.client.motion_control.side_effect = delayed
        real_release = self.recorder.release
        def observed_release():
            result = real_release(); released.set(); return result
        with patch.object(self.recorder, 'release', side_effect=observed_release):
            drive_thread, drive_result = self.spawn_call('drive', 2)
            self.assertTrue(entered.wait(3))
            release_thread, release_result = self.spawn_call('release', 3)
            self.assertTrue(released.wait(3))
            earliest = self.recorder._release_at
            self.clock += .2
            proceed.set()
            drive_thread.join(3); release_thread.join(3)
            self.assertFalse(drive_thread.is_alive()); self.assertFalse(release_thread.is_alive())
        self.assertIsInstance(drive_result.get('error'), ValueError)
        self.assertNotIn('error', release_result)
        self.assertEqual(self.actions()[:2], ['stop', 'calibration_target'])
        self.assertTrue(all(action == 'stop' for action in self.actions()[2:]))
        self.assertFalse(self.outputs_active)
        self.assertEqual(self.recorder._release_at, earliest)


if __name__ == '__main__': unittest.main()
