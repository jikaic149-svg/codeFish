"""Manual radio control does not need Wi-Fi discovery, login, or telemetry."""
from pathlib import Path
import copy
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import device_client
from lora_link import LoRaLink, available_ports
from station import Workstation
from test_dual_link import Serial, wire
from test_robotfish import MOTION


class StandaloneRadioTests(unittest.TestCase):
    def test_selected_radio_never_sends_wifi_control_even_when_receiver_unhealthy(self):
        with tempfile.TemporaryDirectory() as directory:
            client,now=self.client(directory)
            host='192.168.4.1'
            client._profiles[host]='robotfish';client._capabilities[host]={'lora_protocol':2}
            client._sessions[host]={'control_session':'page','password':'fixture'}
            client._motion_latest[host]=({'mode':2,'wifi':{'sequence':0},'lora':{'healthy':False}},time.monotonic())
            client.configure_lora('',port='COM28')
            client.set_control_transport('lora')  # Explicit opt-in; normal connection preserves Wi-Fi fallback.
            with patch.object(client,'_request') as http:
                self.assertTrue(client.motion_control(host,'target','page',1,axes={'surge':35,'yaw':0,'heave':0})['queued'])
                client.lora.close()
                with self.assertRaisesRegex(device_client.DeviceError,'Wi-Fi 仅用于遥测'):
                    client.motion_control(host,'target','page',2,axes={'surge':35,'yaw':0,'heave':0})
                http.assert_not_called()

    def test_default_connection_falls_back_to_wifi_when_receiver_lora_is_lost(self):
        with tempfile.TemporaryDirectory() as directory:
            client,now=self.client(directory);host='192.168.4.1'
            client._profiles[host]='robotfish';client._capabilities[host]={'lora_protocol':2}
            client._sessions[host]={'control_session':'page','password':'fixture'}
            client._motion_latest[host]=({'mode':2,'wifi':{'sequence':0},'lora':{'healthy':False}},time.monotonic())
            self.assertEqual(client.configure_lora('',port='COM28')['control_transport'],'auto')
            with patch.object(client,'_request',return_value=(200,dict(accepted=True,session='page',sequence=1),None)) as http:
                result=client.motion_control(host,'target','page',1,axes={'surge':35,'yaw':0,'heave':0})
                self.assertEqual(result['transport'],'wifi');self.assertTrue(result['accepted'])
                self.assertEqual(http.call_args.args[3]['action'],'target')
            client.lora.close()

    def test_pending_pid_settings_survive_held_targets_but_stop_preempts(self):
        with tempfile.TemporaryDirectory() as directory:
            client,now=self.client(directory);client.configure_lora('',port='COM28');link=client.lora
            link.submit('','target',dict(mode=3,axes={'surge':35,'yaw':0,'heave':0}));link.step()
            now[0]=.1;pid=dict(mode=3,pid=dict(kp=1,ki=.02,kd=.1));link.submit('','yaw_pid',pid)
            self.assertTrue(link.submit('','target',dict(mode=3,axes={'surge':35,'yaw':1,'heave':0}))['waiting'])
            now[0]=.5;link.step();self.assertEqual(wire(link.serial.writes[-1])[4],25)
            now[0]=.6;link.submit('','yaw_pid',pid);link.submit('','stop',{'mode':3});link.step()
            self.assertEqual(wire(link.serial.writes[-1])[4],17);link.close()

    def test_radio_autoconnect_and_login_are_telemetry_only(self):
        client=device_client.DeviceClient();client.set_control_transport('lora')
        host='192.168.4.1';client._profiles[host]='robotfish'
        with patch.object(client,'identity',return_value={'firmware':'robotfish','host':host}), \
                patch.object(client,'robot',return_value={'diagnostics':{}}), \
                patch.object(client,'_request') as http:
            result=client.auto_connect(host,'page')
            self.assertTrue(result['telemetry_only']);self.assertFalse(result['accepted'])
            with patch.object(client,'_robotfish_identity',return_value={'firmware':'robotfish'}):
                self.assertTrue(client.login(host,'',session='page')['telemetry_only'])
            self.assertFalse(client._sessions);http.assert_not_called()

    def test_unchanged_input_renews_without_occupying_each_control_slot(self):
        with tempfile.TemporaryDirectory() as directory:
            client,now=self.client(directory)
            client.configure_lora('',port='COM28')
            link=client.lora
            target={'mode':2,'axes':{'surge':35,'yaw':0,'heave':0}}
            link.submit('','target',target);link.step()
            first=link.sequence
            for moment in (.15,.30,.45,.60):
                now[0]=moment;link.submit('','target',target);link.step()
                self.assertEqual(link.sequence,first)
            now[0]=.65
            link.submit('','target',dict(target,axes={'surge':35,'yaw':40,'heave':0}));link.step()
            self.assertEqual(wire(link.serial.writes[-1])[6:8],[350,400])
            self.assertEqual(link.sequence,first+1)
            link.close()

    def test_fresh_held_goal_renews_at_one_second_but_never_replays_after_release(self):
        with tempfile.TemporaryDirectory() as directory:
            client,now=self.client(directory)
            client.configure_lora('',port='COM28')
            link=client.lora
            target={'mode':2,'axes':{'surge':35,'yaw':0,'heave':0}}
            link.submit('','target',target);link.step()
            first=link.sequence
            now[0]=.5;link.submit('','target',target);link.step()
            self.assertEqual(link.sequence,first)
            now[0]=.9;link.submit('','target',target)
            now[0]=1;link.step()
            self.assertEqual(link.sequence,first+1)
            self.assertEqual(link.status()['last_command_timing']['sequence'],first)
            now[0]=1.1;link.submit('','target',target)
            link.submit('','stop',{'mode':2});link.step()
            stopped=link.sequence
            now[0]=2.1;link.step()
            self.assertEqual(link.sequence,stopped+1)
            self.assertEqual(wire(link.serial.writes[-1])[0],102)
            self.assertIsNone(link.last_target)
            link.close()

    def test_port_scan_returns_actual_names_without_opening_devices(self):
        found=[SimpleNamespace(device='COM119',description='USB Adapter B'),
               SimpleNamespace(device='COM1001',description='USB Adapter C'),
               SimpleNamespace(device='COM7',description='USB Adapter A')]
        with patch('serial.tools.list_ports.comports',return_value=found),patch('serial.Serial') as opened:
            self.assertEqual(available_ports(),[{'port':'COM7','description':'USB Adapter A'},
                                               {'port':'COM119','description':'USB Adapter B'},
                                               {'port':'COM1001','description':'USB Adapter C'}])
            opened.assert_not_called()

    def test_no_default_port_or_automatic_substitution_for_missing_device(self):
        with tempfile.TemporaryDirectory() as directory:
            client=device_client.DeviceClient()
            client.lora=LoRaLink(sequence_file=Path(directory)/'sequence.json')
            client.lora.thread=object()
            ports=[{'port':'COM13','description':'USB-SERIAL CH340'}]
            with patch('lora_link.available_ports',return_value=ports), \
                    patch('serial.Serial',side_effect=Serial) as opened:
                with self.assertRaisesRegex(device_client.DeviceError,'选择'):
                    client.configure_lora('')
                with self.assertRaisesRegex(device_client.DeviceError,'COM28.*未检测到'):
                    client.configure_lora('',port='COM28')
                opened.assert_not_called()
                self.assertTrue(client.configure_lora('',port='COM13')['connected'])
                self.assertEqual(opened.call_args.args,('COM13',115200))
            client.configure_lora('',connected=False)

    def client(self,directory):
        client=device_client.DeviceClient()
        now=[0.0]
        client.lora=LoRaLink(Serial,Path(directory)/'sequence.json',lambda:now[0])
        client.lora.thread=object()  # Run the actual sender one step at a time.
        return client,now

    def test_connect_mode_target_stop_without_any_wifi_or_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            client,now=self.client(directory)
            with patch.object(client,'_request') as http:
                status=client.configure_lora('',port='COM28')
                self.assertTrue(status['connected'])
                uart=client.lora.serial
                client.lora.step()
                self.assertEqual(wire(uart.writes[-1])[0],102)
                self.assertFalse(client._sessions)
                client.motion_control('', 'mode','radio-page',1,client_id='page',mode=2)
                now[0]=.5;client.lora.step()
                self.assertEqual(wire(uart.writes[-1])[4:6],[22,2])
                result=client.motion_control('','target','radio-page',2,client_id='page',
                    axes={'surge':35,'yaw':-40,'heave':100})
                self.assertTrue(result['queued'])
                now[0]=1;client.lora.step()
                self.assertEqual(wire(uart.writes[-1])[4:9],[20,2,350,-400,1000])
                client.motion_control('','stop','radio-page',3,client_id='page')
                client.lora.step()
                self.assertEqual(wire(uart.writes[-1])[4],17)
                http.assert_not_called()
                with self.assertRaises(device_client.DeviceError):
                    client.motion_control('','target','radio-page',2,client_id='page',axes={'surge':100,'yaw':0,'heave':0})
                client.configure_lora('',connected=False)

    def test_station_routes_manual_control_without_selecting_wifi_host(self):
        with tempfile.TemporaryDirectory() as directory:
            client,now=self.client(directory)
            client.configure_lora('',port='COM28')
            station=Workstation()
            with patch('station.device_client.CLIENT',client),patch.object(client,'_request') as http:
                self.assertTrue(station.configure_control('mode',1,'page',1,client_id='a')['queued'])
                self.assertTrue(station.target({'surge':25,'yaw':0,'heave':0},'page',2,'auto',client_id='a')['queued'])
                now[0]=.5;client.lora.step()
                self.assertEqual(wire(client.lora.serial.writes[-1])[4:7],[20,1,250])
                self.assertTrue(station.configure_control('stop',None,'page',3,client_id='a')['queued'])
                client.lora.step()
                self.assertEqual(wire(client.lora.serial.writes[-1])[4],17)
                http.assert_not_called()
            client.configure_lora('',connected=False)

    def test_disconnect_and_shutdown_submit_stop_without_replaying_old_target(self):
        with tempfile.TemporaryDirectory() as directory:
            client,now=self.client(directory)
            client.configure_lora('',port='COM28')
            uart=client.lora.serial
            client.motion_control('','target','page',1,axes={'surge':100,'yaw':0,'heave':0})
            client.configure_lora('',connected=False)
            self.assertTrue(uart.closed)
            self.assertEqual(wire(uart.writes[-1])[4],17)
            self.assertFalse(client.lora.pending)
            client.configure_lora('',port='COM28')
            uart=client.lora.serial
            client.lora.step()
            self.assertEqual(wire(uart.writes[-1])[0],102)
            client.shutdown()
            self.assertTrue(uart.closed)
            self.assertEqual(wire(uart.writes[-1])[4],17)

    def test_wifi_failure_preserves_radio_input_and_ordering_across_transports(self):
        with tempfile.TemporaryDirectory() as directory:
            client,now=self.client(directory)
            host='192.168.4.1'
            client._profiles[host]='robotfish'
            client._capabilities[host]={'lora_protocol':2}
            client._sessions[host]={'control_session':'page','password':'fixture','sequence':0}
            raw=copy.deepcopy(MOTION)
            raw.update(lora_protocol=2,lora={'healthy':True})
            raw['wifi']['session']='page'
            client._motion_latest[host]=(raw,time.monotonic()-3)
            client.configure_lora('',port='COM28')
            client.motion_control(host,'target','page',5,client_id='a',axes={'surge':35,'yaw':0,'heave':0})
            with patch.object(client,'_request',side_effect=device_client.DeviceError('offline',retryable=True)):
                with self.assertRaises(device_client.DeviceError):client.robot(host)
            self.assertEqual(client.lora.pending[0],'target')
            self.assertTrue(client._sessions[host]['recovery_required'])
            with patch.object(client,'_request') as http:
                client.motion_control(host,'target','page',6,client_id='a',axes={'surge':35,'yaw':40,'heave':100})
                client.lora.step()
                self.assertEqual(wire(client.lora.serial.writes[-1])[6:9],[350,400,1000])
                http.assert_not_called()
            client.configure_lora('',connected=False)
            client.set_control_transport('auto')  # Explicitly select Wi-Fi fallback.
            client._sessions[host].pop('recovery_required')
            client._telemetry_failures.clear()
            raw['lora']['healthy']=False
            client._motion_latest[host]=(raw,time.monotonic())
            with patch.object(client,'_request',return_value=(200,{'accepted':True,'session':'page'},None)) as http:
                with self.assertRaises(device_client.DeviceError):
                    client.motion_control(host,'target','page',5,client_id='a',axes={'surge':100,'yaw':0,'heave':0})
                http.assert_not_called()
                client.motion_control(host,'target','page',7,client_id='a',axes={'surge':25,'yaw':0,'heave':0})
                self.assertEqual(http.call_count,1)
                client.motion_control(host,'stop','page',1,client_id='another-page')
                self.assertEqual(http.call_count,2)

    def test_invalid_input_cannot_enter_unauthed_radio_queue(self):
        with tempfile.TemporaryDirectory() as directory:
            client,now=self.client(directory)
            client.configure_lora('',port='COM28')
            for axes in ({'surge':True,'yaw':0,'heave':0},{'surge':float('nan'),'yaw':0,'heave':0},
                         {'surge':101,'yaw':0,'heave':0},{'surge':20}):
                with self.subTest(axes=axes),self.assertRaises(device_client.DeviceError):
                    client.motion_control('','target','page',1,axes=axes)
                self.assertIsNone(client.lora.pending)
            client.configure_lora('',connected=False)

    def test_offline_mode_change_survives_wifi_telemetry_return(self):
        with tempfile.TemporaryDirectory() as directory:
            client,now=self.client(directory)
            host='192.168.4.1'
            client._profiles[host]='robotfish'
            client._capabilities[host]={'lora_protocol':2}
            client._sessions[host]={'control_session':'page','password':'fixture','desired_mode':3}
            client.configure_lora('',port='COM28')
            client.motion_control(host,'mode','page',1,mode=1)
            client.lora.step()
            raw={'mode':3,'wifi':{'sequence':0},'lora':{'healthy':True}}
            client._motion_latest[host]=(raw,time.monotonic())
            with patch.object(client,'_request') as http:
                client.motion_control(host,'target','page',2,axes={'surge':25,'yaw':0,'heave':0})
                now[0]=.5;client.lora.step()
                self.assertEqual(wire(client.lora.serial.writes[-1])[4:7],[20,1,250])
                http.assert_not_called()
            client.configure_lora('',connected=False)


if __name__=='__main__':unittest.main()
