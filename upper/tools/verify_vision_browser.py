"""Real browser and vision process; synthetic file input, never a camera."""
from http.server import ThreadingHTTPServer
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
from unittest.mock import patch

from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tests'),str(ROOT/'tools')]
import device_client
import diagnostics
import server
from credentials import KeyStore
from verify_data import Target
from vision_service import VisionService


def main():
    samples=ROOT.parent/'build/checks/host-vision'
    if not (samples/'auv_record_synthetic.mp4').is_file():
        raise RuntimeError('Run verify_video_recording.py in the installed OpenCV environment first')
    Target.reset()
    target=ThreadingHTTPServer(('127.0.0.1',0),Target)
    web=ThreadingHTTPServer(('127.0.0.1',0),server.Handler)
    server.PORT=web.server_port
    for http in (target,web):threading.Thread(target=http.serve_forever,daemon=True).start()
    host='127.0.0.1:'+str(target.server_port)
    service=VisionService()
    service.output=samples/'browser-recordings'
    with tempfile.TemporaryDirectory() as directory:
        keys=KeyStore(Path(directory)/'keys.json')
        diagnostics.start(directory)
        try:
            with patch('device_client.gateways',return_value=[host]),patch('device_client.KEYS',keys), \
                    patch('server.KEYS',keys),patch.object(server,'VISION',service), \
                    patch.dict(os.environ,{'ROBOFISH_VISION_SAMPLES':str(samples)}), \
                    patch('device_client.capture_network_context',return_value=None),sync_playwright() as playwright:
                browser=playwright.chromium.launch(channel='msedge',headless=True)
                page=browser.new_page(viewport={'width':1440,'height':1000})
                errors=[]
                page.on('pageerror',lambda error:errors.append(str(error)))
                page.goto(f'http://127.0.0.1:{web.server_port}/')
                page.wait_for_function('wifiControlReady() && visionConfiguration?.model_required === false')
                page.locator('[data-page=vision]').click()
                page.locator('#vision-source').select_option('sample')
                assert page.locator('#vision-start').is_enabled()
                page.locator('#vision-start').click()
                page.wait_for_function('visionState?.ready && visionState?.result_ready',timeout=15000)
                record=page.locator('[data-vision-task=RECORD]')
                record.click()
                page.wait_for_function('visionState?.recording?.active && visionState.recording.frames >= 10')
                assert record.inner_text()=='停止录制'
                assert '正在录制' in page.locator('#vision-recording-state').inner_text()
                record.click()
                page.wait_for_function('visionState?.recording?.saved && !recordingRequest')
                first=page.evaluate('visionState.recording')
                assert Path(first['path']).stat().st_size>1000
                assert '已保存' in page.locator('#vision-recording-state').inner_text()
                record.click()
                page.wait_for_function('visionState?.recording?.active && visionState.recording.frames >= 5')
                page.locator('#vision-stop').click()
                page.wait_for_function('!visionState?.running && visionState?.recording?.saved')
                final=page.evaluate('visionState.recording')
                assert final['file']!=first['file']
                output=ROOT.parent/'build/checks/host-vision'
                page.screenshot(path=str(output/'vision-recording-browser.png'),full_page=True)
                assert not errors,errors
                assert not any(any(entry.get('axes',{}).values()) for entry in Target.controls)
                browser.close()
            print(json.dumps({'checks':'S3 vision navigation, model-free start, real preview, record/stop label, saved file, exit flush, no nonzero motion passed','recording':final['file']}))
        finally:
            service.stop()
            diagnostics.stop()
            for connection in device_client.CLIENT._telemetry_connections.values():connection.close()
            for http in (target,web):http.shutdown();http.server_close()


if __name__=='__main__':main()
