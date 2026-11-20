"""Loopback API regression: logout must finish after pending authentication."""
from concurrent.futures import ThreadPoolExecutor
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import server


class SessionOrderingTests(unittest.TestCase):
    def test_forget_and_logout_cannot_be_undone_by_pending_login(self):
        for login, logout in (('device/auto-connect', 'device/forget'),
                              ('device/login', 'device/forget'),
                              ('device/login', 'device/logout')):
            with self.subTest(login=login, logout=logout):
                entered, release, logout_requested, forgotten = (threading.Event() for _ in range(4))
                values, keys, order = {}, {}, []
                host = '192.168.4.1'
                def authenticate(host, password=None, **options):
                    entered.set()
                    if not release.wait(3):
                        raise RuntimeError('fixture authentication was not released')
                    values[host] = 'fixture-session'
                    if options.get('persist'):
                        keys[host] = 'encrypted-fixture'
                    order.append('login')
                    return {'accepted': True, 'session': 'fixture-session'}
                def forget(host):
                    order.append('logout')
                    values.pop(host, None)
                    forgotten.set()
                    return {'online': False}
                class Handler(server.Handler):
                    def do_POST(self):
                        if self.path == '/api/' + logout:
                            logout_requested.set()
                        super().do_POST()
                web = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
                client = SimpleNamespace(is_robotfish=lambda *_: False, auto_connect=authenticate,
                                         login=authenticate, forget=forget)
                station = SimpleNamespace(command_guard=threading.RLock(), guard=threading.RLock(),
                    mode='mission', host=host, recording=False, read=lambda: {'online': True})
                store = SimpleNamespace(get=lambda address: keys.get(address, ''), forget=lambda address: keys.pop(address, None))
                origin = 'http://127.0.0.1:' + str(web.server_port)
                def post(action):
                    body = json.dumps(dict(host=host, password='fixture-secret', session='fixture-session', remember=True)).encode()
                    request = urllib.request.Request(origin + '/api/' + action, data=body,
                        headers={'Content-Type': 'application/json', 'Origin': origin, 'X-AmphiLink-Token': server.TOKEN})
                    with urllib.request.urlopen(request, timeout=5) as response:
                        return json.load(response)
                thread = threading.Thread(target=web.serve_forever, daemon=True)
                thread.start()
                try:
                    with patch.object(server, 'PORT', web.server_port), patch.object(server, 'STATION', station), \
                            patch.object(server, 'KEYS', store), patch.object(server.device_client, 'CLIENT', client), \
                            ThreadPoolExecutor(max_workers=2) as executor:
                        acquired = executor.submit(post, login)
                        self.assertTrue(entered.wait(2))
                        try:
                            exited = executor.submit(post, logout)
                            self.assertTrue(logout_requested.wait(2))
                            self.assertFalse(forgotten.wait(.1), 'Logout passed the pending authentication')
                        finally:
                            release.set()
                        self.assertTrue(acquired.result(timeout=3)['accepted'])
                        self.assertFalse(exited.result(timeout=3)['online'])
                        self.assertEqual(order, ['login', 'logout'])
                        self.assertFalse(values)
                        self.assertEqual(keys, {host: 'encrypted-fixture'} if logout == 'device/logout' else {})
                finally:
                    release.set()
                    web.shutdown()
                    web.server_close()
                    thread.join(timeout=2)


if __name__ == '__main__':
    unittest.main()
