"""Actual video/YOLO/UI with a fake control endpoint. Never opens a camera or fish."""
import copy
import csv
from http.server import ThreadingHTTPServer
import io
import json
import os
from pathlib import Path
import sys
import threading
import time
from unittest.mock import patch

from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'upper'),str(ROOT/'upper/tests')]
import device_client
import robotfish
import server
from test_realtime import RAW
from vision_service import VisionService


def main():
    output=ROOT/'build/host-vision/heading-observer/browser';output.mkdir(parents=True,exist_ok=True)
    configs=output/('configs-'+str(time.time_ns()));configs.mkdir()
    service=VisionService();service.output=output/'exports'
    web=ThreadingHTTPServer(('127.0.0.1',0),server.Handler);server.PORT=web.server_port
    url=f'http://127.0.0.1:{web.server_port}';commands=[];errors=[];requests=[]
    view=robotfish.normalize(copy.deepcopy(RAW),'fixture-control')
    view['attitude_control'].update(heading_target_supported=True,heading_valid=True,body_valid=True,body_yaw_deg=40.)
    view['heading_reference']={'valid':True,'imu_valid':True,'zero_cw_deg':40.,'current_deg':0.,
        'target_deg':None,'target_confirmed':False,'source':'MANUAL_IMU_ZERO','calibration_id':'synthetic-native-zero'}
    def configure(action,value,session,sequence,**kwargs):
        commands.append({'action':action,'value':value,'sequence':sequence})
        assert action in ('heading_target','stop'),commands
        if action=='heading_target':view['heading_reference'].update(target_deg=value['angle_deg'],target_confirmed=True)
        else:view['heading_reference']['target_confirmed']=False
        return {'accepted':True,'message':'Synthetic target accepted; no hardware connected'}
    def forbidden(*args,**kwargs):raise AssertionError('Unexpected physical/automatic control')
    try:
        with patch.object(server,'VISION',service),patch.object(server.STATION,'read',side_effect=lambda:copy.deepcopy(view)), \
             patch.object(service,'cameras',return_value={'cameras':[]}), \
             patch.object(server.STATION,'configure_control',side_effect=configure), \
             patch.object(server.STATION,'target',side_effect=forbidden), \
             patch.object(server.STATION,'prepare_vision',side_effect=forbidden), \
             patch.object(server.STATION,'vision_command',side_effect=forbidden), \
             patch.object(server.STATION,'vision_telemetry',return_value={'valid':False}), \
             patch.object(device_client.CLIENT,'motion_control',side_effect=forbidden), \
             patch.object(device_client.CLIENT.lora,'status',return_value={'connected':True,'mode':3}), \
             patch.object(device_client.CLIENT,'discover_robotfish',return_value=[]), \
             patch('device_client.gateways',return_value=[]),patch('device_client.capture_network_context',return_value=None), \
             patch.dict(os.environ,{'ROBOFISH_VISION_CONFIG':str(configs)}),sync_playwright() as playwright:
            threading.Thread(target=web.serve_forever,daemon=True).start()
            browser=playwright.chromium.launch(channel='msedge',headless=True)
            page=browser.new_page(viewport={'width':1440,'height':1000})
            page.on('pageerror',lambda error:errors.append(str(error)))
            page.on('request',lambda request:requests.append(request.url) if request.method=='POST' else None)
            page.goto(url+'/#vision');page.wait_for_function('visionConfiguration?.automatic_tracking===true')
            page.locator('[data-page=vision]').click();page.locator('#vision-source').select_option('sample')
            page.locator('#vision-start').click()
            page.wait_for_function('visionState.ready && visionState.result_ready',timeout=35000)
            for task in ('AUTO_START','AUTO_STOP','RECORD','BINARY','TURN_CALIB','CLEAR_PATH'):
                assert page.locator('[data-vision-task='+task+']').first.is_visible()
            for task in ('START','STOP'):
                assert page.locator('[data-vision-task='+task+']').count()==0
                assert task not in service.configuration()['supported_tasks']
            assert page.locator('[data-vision-task=AUTO_START]').is_disabled()
            page.locator('[data-vision-task=POOL_CALIB]').first.click()
            page.wait_for_function("visionState.result.interaction==='POOL_CALIB'")
            for index,(x,y) in enumerate([(271,150),(1200,121),(1205,632),(259,606)],1):
                service.action({'kind':'down','x':x/1383,'y':y/775})
                if index<4:page.wait_for_function(f'visionState.result.pool.selected_points==={index}')
            page.wait_for_function('visionState.result.pool.perspective && visionState.result.optical.direction_world')
            page.wait_for_function("visionState.display_space==='field' && document.getElementById('vision-result').naturalWidth===1000 && document.getElementById('vision-result').naturalHeight===500")
            for width,height in ((1440,1000),(2280,994),(2191,1159),(390,844)):
                page.set_viewport_size({'width':width,'height':height})
                geometry=page.locator('#vision-result').evaluate("image=>{const a=image.getBoundingClientRect(),b=image.parentElement.getBoundingClientRect();return {top:a.top-b.top,bottom:a.bottom-b.bottom,left:a.left-b.left,right:a.right-b.right,fit:getComputedStyle(image).objectFit};}")
                assert geometry['top']>=-1 and geometry['left']>=-1 and geometry['bottom']<=1 and geometry['right']<=1,geometry
                assert geometry['fit']=='contain'
            page.set_viewport_size({'width':1440,'height':1000})
            page.locator('#vision-view').select_option('raw')
            page.wait_for_function("visionState.display_space==='image' && document.getElementById('vision-result').naturalWidth===1384 && document.getElementById('vision-result').naturalHeight===776")
            page.locator('#vision-view').select_option('perspective')
            page.wait_for_function("visionState.display_space==='field'")
            # The click selects a visual axis sign, without pretending to label the real head automatically.
            service.action({'task':'HEAD_DIRECTION'})
            page.wait_for_function("visionState.result.interaction==='HEAD_DIRECTION'")
            frozen=service.state()['result']['optical'];point=frozen['front_px']
            world=service.transform({'points':[point]})['points_m'][0]
            service.action({'kind':'down','x':world[0]/4*1000/999,'y':world[1]/2*500/499,'space':'field'})
            page.wait_for_function("visionState.result.initial_heading.calibrated && visionState.preview_enabled && visionState.display_space==='field'")
            assert page.locator('#vision-result').is_visible()
            assert page.locator('#vision-result-empty').is_hidden()
            assert not service.state()['result']['body_yaw']['usable'] # replay is never live IMU feedback
            page.locator('.vision-input-preview').evaluate('element=>{element.open=true;}')
            page.wait_for_function("document.getElementById('vision-raw').naturalWidth===1384 && document.getElementById('vision-raw').naturalHeight===776")
            page.locator('[data-vision-task=RECORD]').click()
            page.wait_for_function('visionState.recording.active && visionState.recording.frames>=3')
            page.locator('[data-vision-task=PREVIEW]').click()
            page.wait_for_function('!visionState.preview_enabled && visionState.recording.frames>=5')
            page.locator('[data-vision-task=RECORD]').click()
            page.wait_for_function('visionState.recording.saved && !visionState.recording.active')
            recording=service.state()['recording'];assert Path(recording['path']).stat().st_size>1000
            page.locator('[data-vision-task=PREVIEW]').click()
            page.wait_for_function('visionState.preview_enabled && visionState.result_ready')
            page.locator('[data-vision-task=BINARY]').click()
            page.wait_for_function('visionState.result.debug.binary')
            page.locator('[data-vision-task=BINARY]').click()
            page.wait_for_function('!visionState.result.debug.binary')
            service.action({'task':'THRESHOLD','value':120})
            page.wait_for_function('visionState.result.debug.threshold===120')
            for kind,x in [('down',.3),('move',.45),('up',.6)]:service.action({'kind':kind,'x':x,'y':.5,'space':'field'})
            page.wait_for_function('visionState.result.path_pixels.length>=2')
            assert not service.state()['result']['tracking']['active']
            assert page.locator('[data-vision-task=AUTO_START]').is_disabled()
            page.locator('[data-vision-task=CLEAR_PATH]').click()
            page.wait_for_function('visionState.result.path_pixels.length===0')
            page.locator('[data-page=overview]').click()
            page.evaluate("""() => {stationMode='mission';autoConnectAttempted=true;
              robotIdentity={firmware:'robotfish',heading_target_supported:true};localRadio={connected:true,mode:3};renderRobot();}""")
            page.locator('#heading-send').evaluate("element => {for(let p=element.parentElement;p;p=p.parentElement)if(p.tagName==='DETAILS')p.open=true;}")
            page.wait_for_function("!document.getElementById('heading-send').disabled")
            page.locator('#heading-angle').fill('30');page.locator('#heading-surge').fill('35')
            page.evaluate('() => {visionState.samples={active:true};}') # stale UI cache must not skip starting the real CSV
            page.locator('#heading-send').click()
            page.wait_for_function("document.getElementById('heading-experiment-state').textContent.includes('目标已回读确认')")
            page.wait_for_function('visionState.samples.active')
            time.sleep(.6)
            assert len(commands)==1 and commands[0]['value']=={'angle_deg':30,'surge_pct':35},commands
            page.locator('[data-page=vision]').click()
            page.wait_for_function('visionState.samples.rows>=3')
            page.locator('[data-vision-task=SAMPLE_STOP]').click()
            page.wait_for_function('visionState.samples.saved && !visionState.samples.active')
            downloads={}
            for kind in ('csv','json'):
                response=page.request.get(url+'/api/vision/samples?format='+kind);assert response.status==200
                data=response.body();(output/('heading-samples.'+kind)).write_bytes(data);downloads[kind]=len(data)
            rows=list(csv.DictReader(io.StringIO((output/'heading-samples.csv').read_text(encoding='utf-8-sig'))))
            assert any(row['vision_valid']=='True' for row in rows)
            assert all(row['x_m'] and row['y_m'] for row in rows)
            assert all(not row['imu_aligned_cw_deg'] and not row['native_target_cw_deg'] for row in rows)
            # A stop while auto-sampling is being prepared cancels the unsent goal.
            page.locator('[data-page=overview]').click()
            page.evaluate("() => {beginVisionSamples=()=>new Promise(resolve=>{window.resumeObserverSampling=resolve;});}")
            page.locator('#heading-send').click()
            page.wait_for_function("typeof window.resumeObserverSampling==='function'")
            page.locator('#control-stop').click()
            page.wait_for_function("document.getElementById('robot-command-result').textContent.includes('已全停')")
            page.evaluate('window.resumeObserverSampling()');time.sleep(.3)
            assert [command['action'] for command in commands]==['heading_target','stop'],commands
            page.locator('[data-page=vision]').click()
            page.evaluate('window.scrollTo(0,0)');page.screenshot(path=str(output/'observer-desktop.png'),full_page=True)
            page.set_viewport_size({'width':390,'height':844});page.screenshot(path=str(output/'observer-mobile.png'),full_page=True)
            page.locator('#vision-stop').click();page.wait_for_function('!visionState.running',timeout=12000)
            time.sleep(.4)
            assert len(commands)==2,commands
            assert not any('/api/vision/control' in request for request in requests)
            assert page.request.get(url+'/api/vision/samples?format=csv').status==200
            assert not errors,errors
            report={'exact_video_and_model':True,'configured_pool_m':[4,2],'independent_optical_angle_and_trajectory':True,
                'head_click_is_test_axis_sign_only':True,'replay_never_pairs_live_imu':True,
                'one_manual_heading_request':[command for command in commands if command['action']=='heading_target'],
                'full_stop_cancels_a_pending_heading_request':True,'visual_pause_and_exit_do_not_change_native_goal':True,
                'preview_disabled_but_sampling_ready':True,'perspective_preview_shape':[1000,500],'raw_shape':[1384,776],
                'complete_image_at_user_viewport_sizes':True,'recording_works_with_preview_off':recording['saved'],
                'debug_tools_preserved':True,'tracking_preview_removed':True,'automatic_controls_preserved':True,'csv_rows':len(rows),'downloads_bytes':downloads,
                'physical_control_calls':0,'browser_errors':errors}
            (output/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
            print(json.dumps(report,ensure_ascii=False,indent=2),flush=True);browser.close()
    except Exception:
        (output/'error-state.json').write_text(json.dumps(service.state(),ensure_ascii=False,indent=2),encoding='utf-8');raise
    finally:service.stop();web.shutdown();web.server_close()


if __name__=='__main__':main()
