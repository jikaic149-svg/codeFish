"""Real browser/API/radio sender, loopback fish and fake UART; no hardware opens."""
import copy
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from unittest.mock import patch

from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tests')]
import device_client
from credentials import KeyStore
from lora_link import LoRaLink
import robotfish
import server
from test_dual_link import Serial,wire
from test_realtime import RAW
from test_robotfish import HEALTH,Target as ControlTarget


class Target(ControlTarget):
    CONTROL_PASSWORD=robotfish.DEFAULT_CONTROL_PASSWORD
    available=False

    def do_GET(self):
        if not self.available:return self.reply({'error':'fixture_offline'},503)
        if self.path=='/get_ip':return self.reply(dict(HEALTH,lora_protocol=2))
        if self.path=='/motion_status':
            raw=copy.deepcopy(self.motion)
            raw.update(lora_protocol=2,lora={'healthy':True},imu=copy.deepcopy(RAW['imu']))
            return self.reply(raw)
        super().do_GET()


def wait_frame(page,uart,predicate,start=0):
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        frames=[wire(data) for data in list(uart.writes)[start:]]
        if any(predicate(frame) for frame in frames):return
        page.wait_for_timeout(50)
    raise AssertionError(frames)


def main():
    Target.reset()
    client=device_client.DeviceClient()
    target=ThreadingHTTPServer(('127.0.0.1',0),Target)
    web=ThreadingHTTPServer(('127.0.0.1',0),server.Handler)
    server.PORT=web.server_port
    host=f'127.0.0.1:{target.server_port}'
    for http in (target,web):threading.Thread(target=http.serve_forever,daemon=True).start()
    with tempfile.TemporaryDirectory() as directory:
        client.lora=LoRaLink(Serial,Path(directory)/'sequence.json')
        keys=KeyStore(Path(directory)/'keys.json')
        ports=[]
        with patch('device_client.CLIENT',client),patch('device_client.KEYS',keys),patch('server.KEYS',keys), \
                patch.object(client,'discover_robotfish',return_value=None), \
                patch.object(server.VISION,'cameras',return_value={'cameras':[{'index':2,'width':1280,'height':720}]}), \
                patch('device_client.available_ports',side_effect=lambda:list(ports)), \
                patch('device_client.capture_network_context',return_value=None):
            server.STATION.start()
            try:
                with sync_playwright() as playwright:
                    browser=playwright.chromium.launch(channel='msedge',headless=True)
                    page=browser.new_page(viewport={'width':1440,'height':1000})
                    errors=[]
                    page.on('pageerror',lambda error:errors.append(str(error)))
                    page.goto(f'http://127.0.0.1:{web.server_port}')
                    page.wait_for_function('localRadio.sent === 0 && !autoConnectPending')
                    page.wait_for_function('!cameraScanning && $("vision-camera-index").value === "2"')
                    assert page.locator('#lora-control-state').is_visible()
                    assert page.locator('[data-axis=forward]').is_disabled()
                    page.locator('#lora-open-settings').click()
                    assert page.locator('#lora-config').is_visible()
                    page.wait_for_function('!radioPortsBusy')
                    assert page.locator('#lora-port').input_value()==''
                    assert page.locator('#lora-connect').is_disabled()
                    assert '未检测到串口' in page.locator('#lora-ports-detail').inner_text()
                    assert client.lora.serial is None
                    refusal=page.evaluate("async()=>{try{await service('station/lora',{connected:true});return '';}catch(error){return error.message;}}")
                    assert '选择' in refusal and client.lora.serial is None
                    ports.extend([{'port':'COM7','description':'USB Adapter A'},
                                  {'port':'COM119','description':'USB Adapter B'}])
                    page.locator('#lora-refresh-ports').click()
                    page.wait_for_function("$('lora-port').options.length === 3 && !radioPortsBusy")
                    assert page.locator('#lora-port').input_value()==''
                    assert page.locator('#lora-connect').is_disabled()
                    page.locator('#lora-port').select_option('COM119')
                    page.locator('#lora-refresh-ports').click()
                    page.wait_for_function('!radioPortsBusy')
                    assert page.locator('#lora-port').input_value()=='COM119'
                    ports.pop()
                    page.locator('#lora-refresh-ports').click()
                    page.wait_for_function("$('lora-port').options.length === 2 && !radioPortsBusy")
                    assert page.locator('#lora-port').input_value()==''
                    assert page.locator('#lora-connect').is_disabled()
                    ports.append({'port':'COM119','description':'USB Adapter B'})
                    page.locator('#lora-refresh-ports').click()
                    page.wait_for_function("$('lora-port').options.length === 3 && !radioPortsBusy")
                    page.locator('#lora-port').select_option('COM119')
                    assert client.lora.serial is None, 'Selecting a port opened it automatically'
                    output=ROOT.parent/'build/checks/host-lora'
                    output.mkdir(parents=True,exist_ok=True)
                    page.screenshot(path=str(output/'port-picker-desktop.png'),full_page=True)
                    page.set_viewport_size({'width':390,'height':844})
                    page.screenshot(path=str(output/'port-picker-mobile.png'),full_page=True)
                    assert page.evaluate('document.documentElement.scrollWidth <= innerWidth')
                    page.set_viewport_size({'width':1440,'height':1000})
                    page.locator('#lora-connect').click()
                    page.wait_for_function('loraControlReady()')
                    uart=client.lora.serial
                    assert uart.args==('COM119',115200)
                    assert not client._sessions and not Target.controls
                    page.locator('[data-page=overview]').click()
                    assert page.evaluate('localRadio.control_transport === "auto"')
                    # This legacy branch explicitly exercises strict radio-only mode.
                    page.locator('#control-source').select_option('lora')
                    page.wait_for_function('!controlConfigPending && localRadio.control_transport === "lora"')
                    assert page.locator('#lora-control-state').inner_text()=='LoRa 发送端已打开 · COM119'
                    assert page.locator('#wifi-telemetry-state').inner_text()=='Wi-Fi 遥测未连接'
                    assert page.locator('#control-stop').is_enabled()
                    page.locator('#robot-control-mode').select_option('2')
                    page.wait_for_function('!controlConfigPending && localRadio.mode === 2')
                    wait_frame(page,uart,lambda f:f[0]==101 and f[4:6]==[22,2])
                    # A slow Wi-Fi connection must not hold the LoRa command queue.
                    page.evaluate('''host=>{
                        $('robot-host').value=host;
                        const original=service;
                        window.originalSlowService=original;
                        service=(path,body)=>path==='device/auto-connect'?new Promise((resolve,reject)=>{window.rejectSlowConnect=reject;}):original(path,body);
                        autoConnect(true);
                    }''',host)
                    page.wait_for_function('typeof window.rejectSlowConnect === "function"')
                    start=len(uart.writes);page.evaluate('document.activeElement.blur()');page.keyboard.down('w')
                    wait_frame(page,uart,lambda f:f[0]==101 and f[4]==20 and f[6]==350,start)
                    assert page.evaluate('autoConnectPending'), 'Radio waited for Wi-Fi connection to finish'
                    page.keyboard.up('w')
                    page.evaluate('service=window.originalSlowService;window.rejectSlowConnect(new Error("Synthetic slow Wi-Fi connection"))')
                    page.wait_for_function('!autoConnectPending')
                    page.evaluate('document.activeElement.blur()')
                    for key in ('w','d','r'):page.keyboard.down(key)
                    wait_frame(page,uart,lambda f:f[0]==101 and f[4:9]==[20,2,350,400,1000])
                    assert '鱼端执行未确认' in page.locator('#control-axis-value').inner_text()
                    assert page.locator('#imu-roll').inner_text()=='—'
                    for key in ('w','d','r'):page.keyboard.up(key)
                    wait_frame(page,uart,lambda f:f[0]==101 and f[4]==20 and f[6:9]==[0,0,0])
                    start=len(uart.writes)
                    page.locator('#control-stop').click()
                    wait_frame(page,uart,lambda f:f[0]==101 and f[4]==17,start)
                    page.locator('[data-page=data]').click()
                    page.wait_for_function('Number.isFinite(localRadio.last_command_timing?.host_ms)')
                    assert page.locator('#data-lora-timing').is_visible()
                    assert page.locator('#data-lora-state').inner_text()=='发送端已测量'
                    assert 'ms' in page.locator('#data-lora-host').inner_text()
                    assert page.locator('#data-live-latency').inner_text()=='—'
                    page.screenshot(path=str(output/'lora-data-desktop.png'),full_page=True)
                    page.set_viewport_size({'width':390,'height':844})
                    page.screenshot(path=str(output/'lora-data-mobile.png'),full_page=True)
                    assert page.evaluate('document.documentElement.scrollWidth <= innerWidth')
                    page.set_viewport_size({'width':1440,'height':1000})
                    page.locator('[data-page=overview]').click()
                    page.evaluate('window.scrollTo(0,0)')
                    page.screenshot(path=str(output/'lora-offline-desktop.png'),full_page=True)
                    page.set_viewport_size({'width':390,'height':844})
                    page.screenshot(path=str(output/'lora-offline-mobile.png'),full_page=True)
                    assert page.evaluate('document.documentElement.scrollWidth <= innerWidth')
                    page.set_viewport_size({'width':1440,'height':1000})

                    # Connect Wi-Fi afterwards, then lose it while holding a LoRa goal.
                    Target.available=True
                    page.locator('#lora-open-settings').click()
                    page.evaluate("$('connection-advanced').open=true")
                    page.locator('#robot-host').fill(host)
                    page.wait_for_function('robotIdentity?.firmware === "robotfish" && !modePending')
                    assert page.locator('#control-video-start').is_enabled(), 'S3 profile disabled the external preview'
                    assert page.locator('#lora-config').is_visible(), 'S3 profile hid the radio panel'
                    page.locator('#robot-refresh').click()
                    page.wait_for_function('robotState?.online && !autoConnectPending && localRadio.control_transport === "lora"')
                    assert not page.evaluate('wifiArmed') and not Target.controls
                    page.locator('[data-page=overview]').click()
                    page.evaluate('document.activeElement.blur()')
                    start=len(uart.writes)
                    page.keyboard.down('w')
                    wait_frame(page,uart,lambda f:f[0]==101 and f[4]==20 and f[6]==350,start)
                    Target.available=False
                    page.wait_for_function('robotState === null && loraControlReady()')
                    assert page.evaluate('heldKeys.has("forward") && loraControlReady()')
                    start=len(uart.writes)
                    page.keyboard.down('a')
                    wait_frame(page,uart,lambda f:f[0]==101 and f[4]==20 and f[6:9]==[350,-400,0],start)
                    assert page.locator('[data-axis=forward]').is_enabled()
                    assert page.locator('#wifi-telemetry-state').inner_text()=='Wi-Fi 遥测未连接'
                    assert page.locator('#robot-device-badge').inner_text()=='RobotFish 遥测已断线'
                    assert '鱼端状态未知' in page.locator('#lora-control-detail').inner_text()
                    page.keyboard.up('w');page.keyboard.up('a')
                    wait_frame(page,uart,lambda f:f[0]==101 and f[4]==20 and f[6:9]==[0,0,0],start)

                    # Actual UART failure disables controls and never replays held keys.
                    page.keyboard.down('w')
                    uart.partial=True
                    page.wait_for_function('!loraControlReady() && heldKeys.size === 0')
                    page.keyboard.up('w')
                    assert uart.closed
                    assert page.locator('[data-axis=forward]').is_disabled()
                    page.locator('#lora-open-settings').click()
                    page.locator('#lora-connect').click()
                    page.wait_for_function('loraControlReady()')
                    replacement=client.lora.serial
                    page.wait_for_timeout(650)
                    assert all(wire(data)[0]==102 for data in replacement.writes), replacement.writes
                    page.locator('#lora-disconnect').click()
                    page.wait_for_function('!localRadio.connected')
                    assert replacement.closed
                    assert wire(replacement.writes[-1])[4]==17
                    assert not errors,errors
                    browser.close()
                print(json.dumps({'checks':'port selection; LoRa sender timing without Wi-Fi telemetry; standalone control, Wi-Fi loss, UART error, no replay passed','uart_frames':len(uart.writes)}))
            finally:
                client.shutdown()
                for connection in client._telemetry_connections.values():connection.close()
                for http in (web,target):http.shutdown();http.server_close()


if __name__=='__main__':main()
