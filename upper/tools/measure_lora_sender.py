"""Explicit real-UART benchmark: STOP and zero targets only; no Wi-Fi or RF ACK claims."""
import argparse
import binascii
import hashlib
from datetime import datetime,timezone
import importlib.util
import json
from pathlib import Path
import sys
import threading
import time

import serial

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',required=True)
    parser.add_argument('--implementation',type=Path,default=ROOT/'lora_link.py')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--address',default='0102')
    parser.add_argument('--channel',default='20')
    parser.add_argument('--samples',type=int,default=12)
    options=parser.parse_args()
    if not 5<=options.samples<=40:parser.error('samples must be 5..40')
    spec=importlib.util.spec_from_file_location('measured_sender',options.implementation)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    writes=[];written=threading.Event()
    class TimedSerial(serial.Serial):
        def write(self,data):
            text,crc=data[3:].rstrip(b'\n').rsplit(b',',1)
            assert binascii.crc_hqx(text,0xffff)==int(crc)
            fields=list(map(int,text.split(b',')))
            assert data[:3]==bytes.fromhex(options.address+options.channel)
            assert fields[0]==102 or (fields[4] in (17,20) and fields[6:9]==[0,0,0]), 'Only neutral test commands are permitted'
            began=time.perf_counter()
            count=super().write(data)
            finished=time.perf_counter()
            writes.append(dict(began=began,finished=finished,bytes=count,sequence=fields[3],
                               scale_tenths=fields[9] if len(fields)>9 else None,
                               code=21 if fields[0]==102 else fields[4],write_ms=(finished-began)*1000))
            written.set()
            return count
    link=module.LoRaLink(TimedSerial)
    samples=[]
    def measure(action,payload,phase):
        submitted=time.perf_counter()
        with link.lock:
            first=len(writes)
            link.submit('',action,payload)
        code=17 if action=='stop' else 20
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            written.clear()
            match=next((row for row in list(writes)[first:] if row['code']==code),None)
            if match:
                row=dict(match,action=action,phase=phase,submitted=submitted,
                         queue_ms=(match['began']-submitted)*1000,total_ms=(match['finished']-submitted)*1000)
                samples.append(row)
                return row
            written.wait(.05)
        raise RuntimeError('UART submission not observed within two seconds')
    link.configure('',port=options.port,address=options.address,channel=options.channel)
    try:
        for index in range(options.samples):
            time.sleep(.571+(index%7)*.007)
            measure('stop',{'mode':2},'idle_stop')
        for index in range(options.samples):
            time.sleep(.55)
            measure('stop',{'mode':2},'before_target')
            time.sleep((.025,.125,.225,.325,.425)[index%5])
            measure('target',{'mode':2,'axes':{'surge':0,'yaw':0,'heave':0}},'rate_limited_zero')
        for index in range(options.samples):
            time.sleep(.55)
            measure('stop',{'mode':2},'before_hold')
            time.sleep(.55)
            payload={'mode':2,'axes':{'surge':0,'yaw':0,'heave':0},'propulsion_scale':100}
            initial=measure('target',payload,'hold_start')
            for repeat in range(1,5):
                time.sleep(max(0,initial['finished']+.15*repeat-time.perf_counter()))
                link.submit('','target',payload)
            time.sleep(max(0,initial['finished']+.65-time.perf_counter()))
            # Change a legal payload field while all motion axes remain zero.
            measure('target',dict(payload,propulsion_scale=99),'change_after_hold')
        received=link.serial.read(min(link.serial.in_waiting,4096))
        summaries={}
        for phase in ('idle_stop','rate_limited_zero','change_after_hold'):
            rows=[row for row in samples if row['phase']==phase]
            summaries[phase]={}
            for key in ('queue_ms','write_ms','total_ms'):
                values=sorted(row[key] for row in rows)
                summaries[phase][key]=dict(n=len(values),minimum=round(values[0],3),
                    median=round(values[len(values)//2],3),p95=round(values[int((len(values)-1)*.95)],3),maximum=round(values[-1],3))
        report=dict(time_utc=datetime.now(timezone.utc).isoformat(),port=options.port,
            implementation=str(options.implementation.resolve()),scope='host submission to Windows UART write completion; RF arrival/execution not measured',
            implementation_sha256=hashlib.sha256(options.implementation.read_bytes()).hexdigest(),
            benchmark_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            radio_parameters_changed=False,nonzero_targets_sent=0,rx_bytes=len(received),rx_hex=received.hex(),summary=summaries,samples=samples,uart_writes=writes)
        options.output.parent.mkdir(parents=True,exist_ok=True)
        options.output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(dict(summary=summaries,rx_bytes=len(received),output=str(options.output.resolve())),ensure_ascii=False),flush=True)
    finally:
        link.submit('','stop',{'mode':2});link.step();link.close()


if __name__=='__main__':main()
