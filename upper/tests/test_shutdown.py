"""Exit the owned workstation, release its port, and keep other programs intact."""
from http.server import ThreadingHTTPServer
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import unittest
from unittest.mock import patch
import urllib.error
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import device_client
import launch
import server


class ShutdownTests(unittest.TestCase):
    @unittest.skipUnless(os.name=='nt','Windows legacy process migration')
    def test_legacy_migration_handles_localized_netstat_and_pins_exact_script(self):
        output=b'\xbb\xee\xb6\xaf\xc1\xac\xbd\xd3\r\n  TCP    127.0.0.1:'+str(launch.PORT).encode()+b'    0.0.0.0:0    LISTENING    333\r\n'
        with patch.object(launch,'local_request',return_value={}), \
                patch.object(subprocess,'check_output',return_value=output), \
                patch.object(subprocess,'run') as stop:
            launch.legacy_stop()
            self.assertEqual(stop.call_args.kwargs['env']['ROBOTFISH_STOP_PID'],'333')
            self.assertEqual(stop.call_args.kwargs['env']['ROBOTFISH_SERVER_SCRIPT'],str(launch.ROOT/'src/server.py'))
            self.assertIn('Get-CimInstance',stop.call_args.args[0][-1])
        with patch.object(launch,'local_request',return_value={}), \
                patch.object(subprocess,'check_output',return_value=b'TCP 127.0.0.1:1 0.0.0.0:0 LISTENING 999'), \
                patch.object(subprocess,'run') as stop:
            with self.assertRaises(RuntimeError): launch.legacy_stop()
            stop.assert_not_called()

    def test_shutdown_stops_only_owned_sessions_and_closes_radio_after_failure(self):
        client = device_client.DeviceClient()
        client._sessions['192.168.4.1'] = dict(control_session='owned', password='fixture')
        with patch.object(client, 'motion_control', side_effect=device_client.DeviceError('offline')) as stop, \
                patch.object(client.lora, 'close') as close:
            self.assertFalse(client.shutdown())
            self.assertEqual(stop.call_args.args, ('192.168.4.1', 'stop', 'owned', 1))
            self.assertTrue(stop.call_args.kwargs['client_id'].startswith('exit-'))
            close.assert_called_once()
        self.assertFalse(client._sessions)

    def test_launcher_rejects_other_instance_before_any_stop(self):
        for identity in ({'service':'other'}, dict(service=launch.SERVICE_ID, instance='other-directory')):
            with patch.object(launch, 'local_request') as request, patch.object(launch, 'legacy_stop') as kill:
                with self.assertRaises(RuntimeError):
                    launch.stop_service(identity)
                request.assert_not_called()
                kill.assert_not_called()

    def test_exit_requires_token_and_explicit_record_discard(self):
        web = ThreadingHTTPServer(('127.0.0.1', 0), server.Handler)
        thread = threading.Thread(target=web.serve_forever, daemon=True)
        thread.start()
        origin = 'http://127.0.0.1:' + str(web.server_port)
        def post(body, token):
            request = urllib.request.Request(origin+'/api/shutdown',data=json.dumps(body).encode(),
                headers={'Content-Type':'application/json','Origin':origin,'X-AmphiLink-Token':token})
            with urllib.request.urlopen(request,timeout=3) as response:
                return json.load(response)
        try:
            with patch.object(server, 'PORT', web.server_port), \
                    patch.object(server.STATION,'data',return_value={'recording':False,'count':3}), \
                    patch.object(server.STATION,'telemetry',return_value={'count':0}), \
                    patch.object(server.STATION,'capture') as capture, \
                    patch.object(server.device_client.CLIENT,'shutdown',return_value=True) as close:
                for body, token, expected in (({},'bad',403), ({},server.TOKEN,409)):
                    with self.assertRaises(urllib.error.HTTPError) as error:
                        post(body,token)
                    self.assertEqual(error.exception.code,expected)
                self.assertTrue(thread.is_alive())
                close.assert_not_called()
                self.assertTrue(post({'discard_records':True},server.TOKEN)['exiting'])
                thread.join(timeout=2)
                self.assertFalse(thread.is_alive())
                capture.assert_called_once_with('stop')
                close.assert_called_once()
        finally:
            web.shutdown()
            web.server_close()

    def test_real_service_process_exits_and_launcher_can_start_again(self):
        with socket.socket() as port_socket:
            port_socket.bind(('127.0.0.1',0))
            port=port_socket.getsockname()[1]
        url='http://127.0.0.1:'+str(port)
        script="from http.server import ThreadingHTTPServer; import server; web=ThreadingHTTPServer(('127.0.0.1',server.PORT),server.Handler)\ntry: web.serve_forever()\nfinally: web.server_close()"
        env=dict(os.environ,ROBOFISH_HOST_PORT=str(port))
        def start():
            return subprocess.Popen([sys.executable,'-B','-c',script],cwd=launch.ROOT/'src',env=env,
                stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,
                creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        with patch.object(launch,'PORT',port), patch.object(launch,'URL',url):
            for _ in range(2):
                child=start()
                try:
                    deadline=time.monotonic()+5
                    identity=None
                    while time.monotonic()<deadline:
                        identity=launch.probe()
                        if identity: break
                        time.sleep(.05)
                    self.assertTrue(identity)
                    self.assertEqual(identity['pid'],child.pid)
                    launch.stop_service(identity)
                    self.assertEqual(child.wait(timeout=4),0)
                    self.assertIsNone(launch.probe())
                finally:
                    if child.poll() is None:
                        child.terminate()
                        child.wait(timeout=3)

    def test_launcher_stop_does_not_spawn_and_old_revision_is_replaced(self):
        old=dict(service=launch.SERVICE_ID,instance=str(launch.ROOT),revision='older')
        current=dict(old,revision=launch.REVISION)
        with patch.object(sys,'argv',['launch.py','--stop']), patch.object(launch,'probe',return_value=None), \
                patch.object(launch,'spawn') as spawn:
            self.assertEqual(launch.main(),0)
            spawn.assert_not_called()
        with patch.object(sys,'argv',['launch.py','--no-browser']), patch.object(launch,'probe',side_effect=[old,current]), \
                patch.object(launch,'stop_service') as stop, patch.object(launch,'spawn') as spawn:
            self.assertEqual(launch.main(),0)
            stop.assert_called_once_with(old)
            spawn.assert_called_once()


if __name__=='__main__':
    unittest.main()
