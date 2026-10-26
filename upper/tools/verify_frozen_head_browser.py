"""Real page and image transport with a synthetic fish; no camera or device calls."""
from pathlib import Path
import json
import os
import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import patch
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'vision')]
sys.path.append(str(Path(sys.base_prefix)/'Lib/site-packages'))
import cv2
import numpy as np
from flask import request, send_from_directory
from playwright.sync_api import sync_playwright
from observer_runtime import ObserverSession
from pool_perspective import PoolPerspective
from web_runtime import WebBridge


def main():
    output = ROOT/'build/host-vision/frozen-head'
    output.mkdir(parents=True, exist_ok=True)
    with patch.dict(os.environ, {'ROBOFISH_PARENT_TOKEN':'fixture', 'ROBOFISH_VISION_TOKEN':'fixture'}):
        bridge = WebBridge(observer=True)
    image = np.full((600,1000,3), 200, np.uint8)
    cv2.ellipse(image, (500,250), (80,23), 0, 0, 360, (20,20,20), -1)
    detector = SimpleNamespace(device='cpu', last_speed={}, update=lambda *_:SimpleNamespace(
        metrics={'body_bbox':(400,150,600,350), 'track_id':1}, position=(500,250), confidence=.9))
    session = ObserverSession('fixture.mp4', 'fixture.pt', bridge, detector=detector,
                              output=output, calibration=output/'unused-pool.json')
    session.accept_frame(1, time.time(), image, 0.)
    session.field = PoolPerspective(4.,2.).calibrate([[100,70],[900,30],[940,540],[60,480]], image.shape)
    errors = []
    def observe():
        index = 1
        try:
            while not bridge.closed.wait(1/30):
                index += 1
                session.actions();session.accept_frame(index, time.time(), image, index/30)
                session.render_preview();session.publish()
        except Exception as error:
            errors.append(repr(error));bridge.closed.set()
    bridge.app.add_url_rule('/', 'page', lambda:send_from_directory(ROOT/'upper', 'index.html'))
    bridge.app.add_url_rule('/<path:name>', 'assets', lambda name:send_from_directory(ROOT/'upper', name))
    for kind in ('frame','stream'):
        bridge.app.add_url_rule('/api/vision/'+kind, 'proxy_'+kind,
                               lambda kind=kind:bridge.app.view_functions[kind](request.args.get('kind','raw')))
    bridge.start(poll_telemetry=False)
    worker = threading.Thread(target=observe, daemon=True);worker.start()
    fixtures = {
        'session':{'token':'fixture','features':{'attitude_control':True}},
        'state':{'connected':False,'features':{'attitude_control':True},'logs':[]},
        'station':{'online':False,'mode':'mission'}, 'station/lora':{'connected':False,'mode':3},
        'station/lora/ports':{'ports':[]}, 'device/discover':{'robotfish':None},
        'links':{'usb':{'state':'unknown'},'http':{},'wifi':{},'enabled':False},
        'data':{'rows':[],'count':0,'recording':False,'capacity':30000},
        'telemetry':{'latest':{},'count':0}, 'vision/cameras':{'cameras':[]},
        'vision/config':{'models':[],'default_model':'','model_required':False,'calibration':[],
                         'supported_tasks':['HEAD_DIRECTION','POOL_CALIB','VIEW']}}
    def route(route):
        url = urlsplit(route.request.url)
        if url.hostname != '127.0.0.1':return route.abort()
        endpoint = url.path.removeprefix('/api/')
        if not url.path.startswith('/api/') or endpoint in ('vision/frame','vision/stream'):return route.continue_()
        headers = {'Authorization':'Bearer fixture'}
        with bridge.app.test_client() as client:
            if endpoint == 'vision/state':
                data = dict(client.get('/state', headers=headers).json, running=True, source='fixture.mp4', process_id=1)
            elif endpoint == 'vision/action':
                response = client.post('/action', headers=headers, json=route.request.post_data_json['event'])
                return route.fulfill(status=response.status_code, content_type='application/json', body=response.get_data())
            else:data = fixtures.get(endpoint,{})
        return route.fulfill(status=200, content_type='application/json', body=json.dumps(data))
    try:
        with sync_playwright() as pw:
            browser = pw.chromium.launch(channel='msedge', headless=True)
            page = browser.new_page(viewport={'width':1440,'height':1000}, extra_http_headers={'Authorization':'Bearer fixture'})
            page.on('pageerror', lambda error:errors.append(str(error)));page.route('**/*', route)
            page.goto(f'http://127.0.0.1:{bridge.server.server_port}/', wait_until='domcontentloaded')
            page.locator('[data-page=vision]').click()
            cases = []
            for view,space in [('perspective','field'),('raw','image')]:
                page.wait_for_function('visionState.ready')
                page.locator('#vision-view').select_option(view)
                page.wait_for_function('(space)=>visionState.display_space===space', arg=space)
                page.locator('[data-vision-task=HEAD_DIRECTION]').first.click()
                page.wait_for_function("visionState.selection_id && $('vision-result').dataset.pointerReady==='true' && $('vision-result').dataset.selectionId===visionState.selection_id", timeout=3000)
                assert page.locator('#vision-result').get_attribute('src').startswith('/api/vision/frame?')
                frozen = session.frozen;point = frozen['pose']['front_px']
                if space == 'field':
                    world = session.field.transform([point])[0];w,h = session.field_size()
                    normalized = [world[0]/session.field.width_m*w/(w-1), world[1]/session.field.height_m*h/(h-1)]
                else:normalized = [point[0]/999, point[1]/599]
                client_point = page.locator('#vision-result').evaluate('''(img,p)=>{
                    const r=img.getBoundingClientRect(),s=Math.min(r.width/img.naturalWidth,r.height/img.naturalHeight);
                    return [r.left+(r.width-img.naturalWidth*s)/2+p[0]*img.naturalWidth*s,
                            r.top+(r.height-img.naturalHeight*s)/2+p[1]*img.naturalHeight*s];}''', normalized)
                page.mouse.click(*client_point)
                page.wait_for_function('!visionState.selection_id && visionState.result.initial_heading.calibrated', timeout=3000)
                assert session.display_space == space
                np.testing.assert_allclose(session.head['head_pixel'], point, atol=1.5)
                cases.append({'view':view,'head_selected':True,'view_preserved':True})
            # Four-corner markers still refresh while the base image stays frozen.
            page.locator('[data-vision-task=POOL_CALIB]').first.click()
            page.wait_for_function("visionState.selection_id && $('vision-result').dataset.selectionId===visionState.selection_id && $('vision-result').dataset.pointerReady==='true'")
            old_src = page.locator('#vision-result').get_attribute('src')
            page.locator('#vision-result').click()
            page.wait_for_function('visionState.result.pool.selected_points===1')
            page.wait_for_function('(old)=>$("vision-result").src!==new URL(old,location.href).href', arg=old_src)
            page.locator('[data-vision-task=POOL_CALIB]').first.click()
            page.wait_for_function('!visionState.selection_id')
            assert not errors, errors
            report = {'cases':cases,'frozen_markers_refresh':True,'cancel_resumes':True,'physical_calls':0,'errors':errors}
            (output/'verification.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
            print(json.dumps(report), flush=True);browser.close()
    except Exception:
        print(json.dumps({'errors':errors,'status':session.status,'last_task':bridge.last_task,
                          'metadata':bridge.metadata}), flush=True)
        raise
    finally:
        bridge.closed.set();worker.join(timeout=2);bridge.server.shutdown()


if __name__ == '__main__':main()
