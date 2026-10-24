"""Real browser + real YOLO subprocess, synthetic yaw telemetry; never drives fish."""
from http.server import ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from unittest.mock import patch

from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'upper'))
import device_client
import server
from vision_service import VisionService


def main():
    service=VisionService()
    output=ROOT/'build/host-vision/yolo-imu'
    output.mkdir(parents=True,exist_ok=True)
    service.output=output/'browser-recordings'
    test_config=output/'test-configs'
    test_config.mkdir(parents=True,exist_ok=True)
    web=ThreadingHTTPServer(('127.0.0.1',0),server.Handler)
    server.PORT=web.server_port
    yaw={'value':40.}
    def telemetry():return {'valid':True,'yaw_cw_deg':yaw['value'],'sample_time':time.time(),'source':'SYNTHETIC_TEST_IMU'}
    controls=[]
    try:
        with patch.object(server,'VISION',service),patch.object(server.STATION,'vision_telemetry',telemetry), \
             patch.object(server.STATION,'target',side_effect=lambda *a,**kw:controls.append(a) or {}), \
             patch('device_client.gateways',return_value=[]),patch('device_client.capture_network_context',return_value=None), \
             patch.dict(os.environ,{'ROBOFISH_VISION_CONFIG':str(test_config)}), \
             sync_playwright() as playwright:
            threading.Thread(target=web.serve_forever,daemon=True).start()
            browser=playwright.chromium.launch(channel='msedge',headless=True)
            page=browser.new_page(viewport={'width':1440,'height':1000})
            errors=[];page.on('pageerror',lambda error:errors.append(str(error)))
            page.goto(f'http://127.0.0.1:{web.server_port}/')
            page.wait_for_function("visionConfiguration?.default_model==='vision/fish_body_yolo/best.pt'")
            page.locator('[data-page=vision]').click()
            page.locator('#vision-source').select_option('sample')
            page.locator('#vision-start').click()
            page.wait_for_function('visionState?.ready && visionState?.result?.auv_detected',timeout=25000)
            page.wait_for_function("visionState.result.system.detector_ready===true")
            service.action({'task':'POOL_SIZE','width_m':4.2,'height_m':2.1}) # test coordinates only
            time.sleep(.2)
            for x,y in [(.195,.19),(.87,.17),(.87,.82),(.19,.80)]:
                service.action({'kind':'down','x':x,'y':y});time.sleep(.1)
            page.wait_for_function('visionState.result.calibrated===true',timeout=5000)
            service.action({'task':'HEAD_DIRECTION'});time.sleep(.1)
            state=service.state();point=state['result']['pixel']
            service.action({'kind':'down','x':(point['u']-35)/(state['width']-1),
                            'y':point['v']/(state['height']-1)})
            page.wait_for_function('visionState.result.body_yaw.usable && Math.abs(visionState.result.body_yaw.yaw_deg)<0.01',timeout=5000)
            zero=service.state()['result']['body_yaw']
            yaw['value']=65.
            page.wait_for_function('Math.abs(visionState.result.body_yaw.yaw_deg+25)<0.1',timeout=5000)
            rotated=service.state()['result']['body_yaw']
            assert abs(rotated['world_yaw_rad']-zero['world_yaw_rad'])>math.radians(20)
            assert 'IMU yaw' in page.locator('#vision-yaw-state').inner_text()
            service.action({'task':'START'})
            page.wait_for_function("visionState.result.tracking.status==='REPLAY PREVIEW ONLY'",timeout=5000)
            page.locator('[data-vision-task=RECORD]').click()
            page.wait_for_function('visionState.recording.active && visionState.recording.frames>=3',timeout=8000)
            page.locator('[data-vision-task=RECORD]').click()
            page.wait_for_function('visionState.recording.saved && !visionState.recording.active',timeout=8000)
            recording=service.state()['recording']
            assert Path(recording['path']).stat().st_size>1000
            page.screenshot(path=str(output/'cascade-desktop.png'),full_page=True)
            page.set_viewport_size({'width':390,'height':844})
            page.screenshot(path=str(output/'cascade-mobile.png'),full_page=True)
            assert not errors,errors
            assert not controls,controls
            page.locator('#vision-stop').click()
            page.wait_for_function('!visionState.running',timeout=10000)
            browser.close()
            report={'checks':'Actual YOLO model, exact source replay, pool event, 0-yaw calibration, IMU-driven arrow, replay not used as live feedback, recording flush, desktop/mobile, exit, no control commands',
                    'zero_yaw_deg':zero['yaw_deg'],'rotated_yaw_deg':rotated['yaw_deg'],
                    'browser_errors':errors,'device_control_calls':len(controls),
                    'recording_saved':recording['saved'],
                    'note':'Pool dimensions and yaw are test fixtures, not physical calibration.'}
            (output/'browser_verification.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
            print(json.dumps(report,indent=2),flush=True)
    finally:
        service.stop();web.shutdown();web.server_close()


if __name__=='__main__':main()
