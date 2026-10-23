# Fixed bench source/sequence: reboot ESP32 before replay; only sends STOP (17).
import serial,json,time,pathlib,tempfile
ports=[];rows=[];packets=[];probes=[]
SOURCE=23120
def opened(name):
 s=serial.Serial(port=None,baudrate=115200,timeout=0.05,write_timeout=1);s.dtr=s.rts=False;s.port=name;s.open();ports.append(s);return s
try:
 usb=opened('COM26');radio=opened('COM28')
 def sample(seconds,sequence=None):
  end=time.monotonic()+seconds;last=None
  while time.monotonic()<end:
   line=usb.readline();at=line.find(b'USBSTAT ')
   if at>=0:
    try:last=json.loads(line[at+8:]);rows.append(last)
    except ValueError:continue
    if sequence is not None and last['source']==SOURCE and last['sequence']==sequence and last['code']==17 and last['result']==0:return last
  return last
 before=sample(1)
 if not before:before=sample(6)
 if not before:raise RuntimeError('No COM26 USB telemetry')
 def wire(dest,seq):
  payload=f'100,1,{SOURCE},{seq},17\n'.encode()
  frame=bytes((dest>>8,dest&255,0x20))+payload
  assert frame[-1]==10
  radio.write(frame);radio.flush()
  return frame.hex()
 def send(dest,seq):
  started=time.monotonic();hexdata=wire(dest,seq);s=sample(2.5,seq)
  ok=s is not None and s['source']==SOURCE and s['sequence']==seq and s['code']==17 and s['result']==0
  row={'destination':hex(dest),'sequence':seq,'wire_hex':hexdata,'accepted':bool(ok),'elapsed_ms':round((time.monotonic()-started)*1000,1)}
  print('LoRa',hex(dest),seq,'PASS' if ok else 'NO_RECEIPT',flush=True)
  return row,s
 dest=0x0101;probe,last=send(dest,1);probes.append(probe)
 if not probe['accepted']:
  dest=0x0102;probe,last=send(dest,2);probes.append(probe)
 if probe['accepted']:
  packets.append(probe);first_seq=probe['sequence']
  for seq in range(first_seq+1,first_seq+10):
   row,last=send(dest,seq);packets.append(row)
   if not row['accepted']:break
  baseline=sample(0.3);last_seq=packets[-1]['sequence']
  wire(dest,last_seq);duplicate=sample(0.8)
  wire(dest,0);stale=sample(0.8)
  wire(0x017e,last_seq+1);after=sample(1.1)
  checks={'lora_10_of_10':len(packets)==10 and all(r['accepted'] for r in packets),
   'duplicate_ignored':duplicate['accepted']==baseline['accepted'] and duplicate['duplicates']>baseline['duplicates'],
   'old_sequence_rejected':stale['accepted']==baseline['accepted'] and stale['stale']>duplicate['stale'],
   'wrong_address_filtered':after['accepted']==stale['accepted'] and after['rejected']==stale['rejected'],
   'no_lora_rx_errors':after['lora_errors']==before['lora_errors'],
   'imu_valid_continuous':after['angles_valid']==1 and after['angle_age_ms']<200 and after['imu_frames']>before['imu_frames'],
   'no_new_imu_errors':after['imu_crc_errors']==before['imu_crc_errors'] and after['imu_rx_errors']==before['imu_rx_errors'],
   'props_neutral':all(r['left']==r['right']==0 and r['written_mask']==63 for r in rows),
   'rf_reply_disabled':all(r['rf_tx']==0 for r in rows)}
 else:after=rows[-1];checks={'lora_10_of_10':False}
 report={'version':'2.3.2-lora-imu','transport':'USB COM26','sender_port':'COM28','wifi_changed':False,'only_command':17,
  'source':SOURCE,'resolved_destination':hex(dest),'probes':probes,'packets':packets,'checks':checks,
  'all_passed':bool(checks) and all(checks.values()),'before':before,'after':after,'samples':rows}
 path=pathlib.Path(tempfile.gettempdir())/'robotfish-usb-final-test.json';path.write_text(json.dumps(report,indent=2),encoding='utf-8')
 print(json.dumps({'all_passed':report['all_passed'],'checks':checks,'packets':len(packets),'last':after,'path':str(path)},indent=2),flush=True)
finally:
 for s in ports:s.close()

raise SystemExit(0 if report.get('all_passed') else 1)
