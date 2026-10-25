"""Attitude configuration stays ordered and uses the existing command routes."""
import binascii
import copy
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import device_client
import robotfish
from lora_link import frame
from test_realtime import RAW
from station import Workstation

HOST, SESSION = '192.168.4.1', 'attitude-fixture'


class AttitudeTests(unittest.TestCase):
    def test_manual_heading_is_sent_once_with_left_positive_and_no_visual_feedback(self):
        station=Workstation();station.host=HOST
        view={'online':True,'checked_at':10.,'diagnostics':{'uptime_ms':1000},
              'attitude_control':{'heading_valid':True,'body_valid':True,'body_yaw_deg':40.,'age_ms':20.}}
        calls=[]
        client=SimpleNamespace(is_robotfish=lambda host:True,
            motion_control=lambda *args,**kwargs:calls.append((args,kwargs)) or {'accepted':True})
        def read():return dict(view,heading_reference=station.heading_reference_state(view))
        with patch('station.device_client.CLIENT',client),patch.object(station,'read',side_effect=read):
            reference=station.configure_control('heading_zero',None,'page',1)['reference']
            self.assertTrue(reference['valid']);self.assertEqual(calls,[])
            station.configure_control('heading_target',{'angle_deg':30.,'surge_pct':0.},'page',2)
            self.assertEqual(len(calls),1)
            self.assertEqual(calls[0][0][1],'heading_target')
            self.assertEqual(calls[0][1]['heading_cw_deg'],10.)
            self.assertEqual(calls[0][1]['surge_pct'],0.)
            view['attitude_control'].update(body_yaw_deg=10.,heading_target_active=True,requested_heading_cw_deg=10.)
            confirmed=read()['heading_reference']
            self.assertAlmostEqual(confirmed['current_deg'],30.)
            self.assertTrue(confirmed['target_confirmed'])
            self.assertEqual(len(calls),1)
            view['diagnostics']['uptime_ms']=10
            self.assertFalse(read()['heading_reference']['valid'])
            with self.assertRaises(device_client.DeviceError):
                station.configure_control('heading_target',{'angle_deg':30.,'surge_pct':35.},'page',3)
            self.assertEqual(len(calls),1)

    def test_visual_reference_only_sets_coordinates_and_survives_normal_uptime_wrap(self):
        station=Workstation();station.host=HOST
        view={'online':True,'checked_at':10.,'diagnostics':{'uptime_ms':0xfffffff0},
              'attitude_control':{'heading_valid':True,'body_valid':True,'body_yaw_deg':40.,'age_ms':20.}}
        calibration={'imu_zero_cw_deg':55.,'image_sample_time':9.7,'source_id':0xffffffe0,
                     'calibration_id':'optical-zero','visual_zero_world_deg':90.}
        with patch.object(station,'read',return_value=view),patch.object(device_client.CLIENT,'motion_control') as motion:
            reference=station.capture_heading_reference(calibration)
            self.assertEqual(reference['current_deg'],15.)
            view['diagnostics']['uptime_ms']=10
            self.assertTrue(station.heading_reference_state(view)['valid'])
            view['attitude_control']['body_valid']=False
            self.assertFalse(station.heading_reference_state(view)['imu_valid'])
            self.assertTrue(station.heading_reference_state(view)['valid'])
            motion.assert_not_called()

    def test_pid_gains_use_wifi_fallback_and_reject_invalid_payloads_without_losing_session(self):
        client=self.client();client._capabilities[HOST]['yaw_pid_supported']=True
        pid=dict(kp=1.25,ki=.02,kd=.1)
        with patch.object(client,'_request',return_value=(200,dict(accepted=True,session=SESSION,sequence=1),None)) as http:
            result=client.motion_control(HOST,'yaw_pid',SESSION,1,pid=pid)
            self.assertEqual(result['transport'],'wifi')
            self.assertEqual(http.call_args.args[3]['pid'],pid)
            for bad in ({'kp':1},dict(pid,ki=-1),dict(pid,kd=float('nan')),dict(pid,kp=True),dict(pid,kp=101)):
                with self.assertRaises(device_client.DeviceError):client.motion_control(HOST,'yaw_pid',SESSION,2,pid=bad)
            self.assertEqual(http.call_count,1)
        self.assertIn(HOST,client._sessions)
        client._capabilities[HOST].pop('yaw_pid_supported')
        with self.assertRaises(device_client.DeviceError) as exc:client.motion_control(HOST,'yaw_pid',SESSION,2,pid=pid)
        self.assertEqual(exc.exception.code,'unsupported_control')

    def test_pid_lora_packet_encodes_gains_without_motion(self):
        wire=frame(42,'yaw_pid',dict(mode=3,pid=dict(kp=1,ki=.02,kd=.1)))
        self.assertEqual(wire,b'101,1,23120,42,25,3,1000,20,100,1000,37623\n')

    def test_heading_target_packet_has_native_angle_units(self):
        wire=frame(42,'heading_target',dict(mode=3,heading_cw_deg=-179.55,surge_pct=35))
        self.assertEqual(wire,b'101,1,23120,42,26,3,-17955,350,0,1000,6570\n')

    def test_heading_target_validates_capability_and_uses_wifi_without_pc_yaw(self):
        client=self.client();client._capabilities[HOST]['heading_target_supported']=True
        with patch.object(client,'_request',return_value=(200,dict(accepted=True,session=SESSION,sequence=1),None)) as http:
            self.assertTrue(client.motion_control(HOST,'heading_target',SESSION,1,heading_cw_deg=-90,surge_pct=35)['accepted'])
            body=http.call_args.args[3]
            self.assertEqual(body['heading_cw_deg'],-90);self.assertEqual(body['surge_pct'],35)
            self.assertNotIn('axes',body)
            for payload in [dict(heading_cw_deg=181,surge_pct=0),dict(heading_cw_deg=0,surge_pct=-1),
                            dict(heading_cw_deg=float('nan'),surge_pct=0),dict(heading_cw_deg=0,surge_pct=True)]:
                with self.assertRaises(device_client.DeviceError):client.motion_control(HOST,'heading_target',SESSION,2,**payload)
            self.assertEqual(http.call_count,1)
        client._capabilities[HOST].pop('heading_target_supported')
        with self.assertRaises(device_client.DeviceError) as error:
            client.motion_control(HOST,'heading_target',SESSION,2,heading_cw_deg=0,surge_pct=0)
        self.assertEqual(error.exception.code,'unsupported_control')

    def client(self, dual=True):
        client = device_client.DeviceClient()
        client._profiles[HOST] = 'robotfish'
        client._capabilities[HOST] = dict(attitude_control_supported=True, attitude_control_calibrated=True, lora_protocol=2 if dual else 0)
        client._sessions[HOST] = dict(control_session=SESSION, password='fixture-secret', sequence=0)
        client._motion_latest[HOST] = (dict(mode=2, wifi=dict(session=SESSION, sequence=0),
            lora=dict(healthy=False)), device_client.time.monotonic())
        return client

    def test_attitude_frames_match_firmware_golden_packets(self):
        for action, expected in (
                ('attitude_on', b'101,1,23120,42,23,3,0,0,0,1000,7010\n'),
                ('attitude_off', b'101,1,23120,42,24,3,0,0,0,1000,27492\n')):
            wire = frame(42, action, {'mode': 3})
            self.assertEqual(wire, expected)
            body, crc = wire.strip().rsplit(b',', 1)
            self.assertEqual(binascii.crc_hqx(body, 0xffff), int(crc))

    def test_wifi_and_primary_lora_configuration_routes(self):
        for dual in (False, True):
            with self.subTest(dual=dual):
                client, calls = self.client(dual), []
                def request(host, path, method, body, **options):
                    calls.append(body)
                    return 200, dict(accepted=True, session=SESSION, sequence=body['sequence']), None
                client._request = request
                for sequence, action in enumerate(('attitude_on', 'attitude_off'), 1):
                    self.assertTrue(client.motion_control(HOST, action, SESSION, sequence, client_id='page')['accepted'])
                self.assertEqual([entry['action'] for entry in calls], ['attitude_on', 'attitude_off'])
                self.assertTrue(all(set(entry) == {'action', 'session', 'sequence'} for entry in calls))
        client, queued = self.client(), []
        client._motion_latest[HOST][0]['lora']['healthy'] = True
        client.lora = SimpleNamespace(status=lambda: dict(connected=True, host=HOST),
            submit=lambda host, action, payload: queued.append((action, payload)) or dict(queued=True))
        with patch.object(client, '_request') as request:
            self.assertTrue(client.motion_control(HOST, 'attitude_on', SESSION, 1)['queued'])
            self.assertEqual(queued, [('attitude_on', {'mode': 3})])
            request.assert_not_called()

    def test_old_firmware_does_not_receive_unsupported_configuration(self):
        client = self.client()
        client._capabilities[HOST].pop('attitude_control_supported')
        with patch.object(client, '_request') as request:
            with self.assertRaises(device_client.DeviceError) as raised:
                client.motion_control(HOST, 'attitude_on', SESSION, 1)
            self.assertEqual(raised.exception.code, 'unsupported_control')
            request.assert_not_called()
        self.assertIn(HOST, client._sessions)

    def test_uncalibrated_profile_cannot_be_enabled_but_can_be_disabled(self):
        client = self.client()
        client._capabilities[HOST]['attitude_control_calibrated'] = False
        with patch.object(client, '_request') as request:
            with self.assertRaises(device_client.DeviceError) as raised:
                client.motion_control(HOST, 'attitude_on', SESSION, 1)
            self.assertEqual(raised.exception.code, 'attitude_not_calibrated')
            request.assert_not_called()
            request.return_value = (200, dict(accepted=True, session=SESSION, sequence=1), None)
            self.assertTrue(client.motion_control(HOST, 'attitude_off', SESSION, 1)['accepted'])
        self.assertIn(HOST, client._sessions)

    def test_slow_telemetry_cannot_publish_active_or_fresh_feedback(self):
        raw = copy.deepcopy(RAW)
        raw['attitude_control_supported'] = True
        raw['attitude_control'] = dict(enabled=True, reason='active', body_valid=True,
            pitch_calibrated=True, roll_calibrated=True, pitch_active=True, roll_active=True,
            yaw_calibrated=True, yaw_active=True, heading_valid=True, heading_locked=True,
            age_ms=20, maximum_age_ms=200, body_roll_deg=12, body_pitch_deg=-8,
            body_yaw_deg=30, heading_target_deg=25, yaw_correction_pct=-5,
            pitch_correction_pct=8.2, roll_correction_pct=-12.4)
        fresh = robotfish.normalize(raw, HOST)['attitude_control']
        self.assertTrue(fresh['pitch_active'] and fresh['roll_active'])
        self.assertEqual(fresh['body_roll_deg'], 12)
        self.assertTrue(fresh['heading_locked'] and fresh['yaw_active'])
        self.assertEqual(fresh['heading_target_deg'], 25)
        stale = robotfish.normalize(raw, HOST, 250)['attitude_control']
        self.assertTrue(stale['enabled'])
        self.assertFalse(stale['body_valid'] or stale['pitch_active'] or stale['roll_active'])
        self.assertIsNone(stale['body_roll_deg'])
        self.assertIsNone(stale['pitch_correction_pct'])
        self.assertFalse(stale['heading_locked'] or stale['yaw_active'])
        self.assertIsNone(stale['heading_target_deg'])
        self.assertFalse(robotfish.normalize(RAW, HOST)['attitude_control']['supported'])


if __name__ == '__main__':
    unittest.main()
