"""Existing pool calibration with the exact replay, without an orientation test."""
from http.server import ThreadingHTTPServer
import json
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

POINTS=[[272,151],[1201,122],[1206,633],[260,607]]


def main():
    output=ROOT/'build/host-vision/perspective-test';output.mkdir(parents=True,exist_ok=True)
    configuration=output/('integrated-config-'+str(time.time_ns()));configuration.mkdir()
    service=VisionService();service.output=output/'exports'
    web=ThreadingHTTPServer(('127.0.0.1',0),server.Handler);server.PORT=web.server_port
    url=f'http://127.0.0.1:{web.server_port}';controls=[];errors=[]
    def forbid_control(*args,**kwargs):controls.append(str(args));raise AssertionError('Unexpected device command')
    try:
        with patch.object(server,'VISION',service), \
             patch.object(server.STATION,'target',side_effect=forbid_control), \
             patch.object(server.STATION,'prepare_vision',side_effect=forbid_control), \
             patch.object(server.STATION,'configure_control',side_effect=forbid_control), \
             patch.object(server.STATION,'vision_telemetry',return_value={'valid':False}), \
             patch.object(device_client.CLIENT,'discover_robotfish',return_value=[]), \
             patch('device_client.gateways',return_value=[]), \
             patch('device_client.capture_network_context',return_value=None), \
             patch.dict(os.environ,{'ROBOFISH_VISION_CONFIG':str(configuration)}), \
             sync_playwright() as playwright:
            threading.Thread(target=web.serve_forever,daemon=True).start()
            browser=playwright.chromium.launch(channel='msedge',headless=True)
            page=browser.new_page(viewport={'width':1440,'height':1000})
            page.on('pageerror',lambda error:errors.append(str(error)))
            page.goto(url+'/#vision')
            page.wait_for_function("visionConfiguration?.default_model==='vision/fish_body_yolo/best.pt'")
            page.locator('[data-page=vision]').click()
            assert page.locator('#vision-profile').count()==0
            page.locator('#vision-source').select_option('sample');page.locator('#vision-start').click()
            page.wait_for_function('visionState.ready && visionState.result_ready',timeout=30000)
            page.locator('[data-vision-task=POOL_CALIB]').first.click()
            page.wait_for_function('visionState.result.pool?.calibrating',timeout=8000)
            page.locator('#vision-result').scroll_into_view_if_needed()
            for count,(x,y) in enumerate(POINTS,1):
                box=page.locator('#vision-result').bounding_box()
                size=page.evaluate("({w:document.getElementById('vision-result').naturalWidth,h:document.getElementById('vision-result').naturalHeight})")
                scale=min(box['width']/size['w'],box['height']/size['h'])
                left=box['x']+(box['width']-size['w']*scale)/2;top=box['y']+(box['height']-size['h']*scale)/2
                page.mouse.click(left+x*scale,top+y*scale)
                if count<4:page.wait_for_function(f'visionState.result.pool.selected_points==={count}',timeout=5000)
            page.wait_for_function('visionState.result.pool.perspective && !visionState.result.pool.calibrating',timeout=8000)
            calibrated=service.state();matrix=calibrated['result']['pool']['perspective']
            assert matrix['physical_width_m']==4 and matrix['physical_height_m']==2
            transformed=page.evaluate("async points=>await service('vision/transform',{points})",matrix['points_tl_tr_br_bl'])
            for actual,expected in zip(transformed['points_m'],[[0,0],[4,0],[4,2],[0,2]]):
                assert max(abs(a-b) for a,b in zip(actual,expected))<1e-5
            for kind in ('image','json'):
                response=page.request.get(url+'/api/vision/perspective-test?kind='+kind)
                assert response.status==200
                data=response.body();assert len(data)>100
                (output/('rectified-pool.png' if kind=='image' else 'calibration-result.json')).write_bytes(data)
            assert '/api/vision/stream' in page.locator('#vision-result').get_attribute('src')
            page.screenshot(path=str(output/'upper-desktop.png'),full_page=True)
            page.set_viewport_size({'width':390,'height':844})
            page.screenshot(path=str(output/'upper-mobile.png'),full_page=True)
            page.locator('#vision-stop').click();page.wait_for_function('!visionState.running',timeout=12000)
            page.locator('#vision-start').click()
            page.wait_for_function('visionState.ready && visionState.result.pool?.perspective',timeout=30000)
            reloaded=service.state()['result']['pool']['perspective']
            assert reloaded['image_to_field']==matrix['image_to_field']
            assert page.request.get(url+'/api/vision/perspective-test?kind=json').status==200
            assert not errors,errors
            assert not controls,controls
            page.locator('#vision-stop').click();page.wait_for_function('!visionState.running',timeout=12000)
            browser.close()
            report={'source':calibrated['source'],'shape':[calibrated['width'],calibrated['height']],
                    'field_m':[4,2],'selected_points':matrix['points_tl_tr_br_bl'],
                    'mapped_corners_m':transformed['points_m'],'automatically_reloaded':True,
                    'integrated_with_existing_pool_calibration':True,'new_ui_mode':False,
                    'rectified_image_not_streamed':True,'fish_heading_calibration_tested':False,
                    'device_controls':len(controls),'browser_errors':errors,
                    'test_export':calibrated['result']['pool']['test'],
                    'note':'Test corners selected from the supplied video; dimensions assumed 4 x 2 m.'}
            (output/'browser-verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
            print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
    except Exception:
        (output/'browser-error-state.json').write_text(json.dumps(service.state(),ensure_ascii=False,indent=2),encoding='utf-8')
        raise
    finally:
        service.stop();web.shutdown();web.server_close()


if __name__=='__main__':main()
