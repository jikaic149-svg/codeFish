"""Record a synthetic replay through the real vision process, without a camera."""
import csv
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import secrets
import sys
import threading
import time
from unittest.mock import patch

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'upper'))
from vision_service import VisionService


def main():
    work=ROOT/'build/host-vision'
    work.mkdir(parents=True,exist_ok=True)
    source=work/'auv_record_synthetic.mp4'
    writer=cv2.VideoWriter(str(source),cv2.CAP_FFMPEG,cv2.VideoWriter_fourcc(*'mp4v'),20,(320,240))
    assert writer.isOpened()
    for index in range(400):
        frame=np.full((240,320,3),70,np.uint8)
        cv2.circle(frame,(50+index%220,120),12,(240,240,240),-1)
        writer.write(frame)
    writer.release()
    calls=[]
    class Parent(BaseHTTPRequestHandler):
        def log_message(self,*_):pass
        def do_POST(self):
            body=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            calls.append(body)
            data=b'{"accepted":true}'
            self.send_response(200);self.send_header('Content-Length',str(len(data)))
            self.end_headers();self.wfile.write(data)
    parent=ThreadingHTTPServer(('127.0.0.1',0),Parent)
    threading.Thread(target=parent.serve_forever,daemon=True).start()
    service=VisionService()
    service.output=work/'recordings'
    def wait(predicate,timeout=8):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            state=service.state()
            if predicate(state):return state
            if not state['running']:raise AssertionError(state['logs'][-8:])
            time.sleep(.05)
        raise AssertionError(service.state())
    def inspect(recording):
        assert recording['saved'] and not recording['error'],recording
        path=Path(recording['path'])
        capture=cv2.VideoCapture(str(path))
        assert capture.isOpened(),path
        assert abs(capture.get(cv2.CAP_PROP_FPS)-20)<.01
        frames=0
        while True:
            ok,frame=capture.read()
            if not ok:break
            assert frame.shape[:2]==(240,320)
            frames+=1
        capture.release()
        assert frames==recording['frames'] and frames>=4,(frames,recording)
        with Path(recording['csv_path']).open(encoding='utf-8-sig',newline='') as file:
            rows=list(csv.reader(file))
            assert len(rows)==frames+1
            assert len({row[0] for row in rows[1:]})==frames,'Duplicate source frames were recorded'
        return {'file':path.name,'frames':frames,'bytes':path.stat().st_size}
    try:
        with patch.dict(os.environ,{'ROBOFISH_VISION_SAMPLES':str(work)}):
            service.start('sample',secrets.token_hex(24),parent.server_port)
        wait(lambda state:state.get('ready') and state.get('result_ready'))
        service.action({'task':'RECORD'})
        wait(lambda state:state.get('recording',{}).get('frames',0)>=20)
        service.action({'task':'RECORD'})
        stopped=wait(lambda state:state.get('recording',{}).get('saved'))['recording']
        first=inspect(stopped)
        service.action({'task':'RECORD'})
        wait(lambda state:state.get('recording',{}).get('active') and state['recording'].get('frames',0)>=10)
        final=service.stop()
        assert service.process.poll()==0,final
        second=inspect(final['recording'])
        assert first['file']!=second['file']
        assert not any(any(body.get('axes',{}).values()) for body in calls),calls
        print(json.dumps({'normal_stop':first,'exit_flush':second,'source':'synthetic replay','camera_opened':False,'nonzero_targets':0}))
    finally:
        service.stop()
        parent.shutdown();parent.server_close()


if __name__=='__main__':main()
