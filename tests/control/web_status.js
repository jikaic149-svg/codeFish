'use strict';
// Run with node tests/control/web_status.js [path/to/control.html]. No hardware.
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),{webcrypto}=require('node:crypto');
const html=fs.readFileSync(process.argv[2]||path.resolve(__dirname,'../../main/control.html'),'utf8');
const script=html.match(/<script>([\s\S]*?)<\/script>/)[1];
new vm.Script(script);
const nodes={},elements=[],events={},calls=[];
for(const match of html.matchAll(/<([a-z][\w-]*)\b([^>]*)>/g)){
 const attrs=Object.fromEntries(Array.from(match[2].matchAll(/([\w-]+)="([^"]*)"/g),m=>[m[1],m[2]]));
 const classes=new Set((attrs.class||'').split(' '));
 const node={dataset:{},style:{},textContent:'',value:attrs.value||'',files:[],disabled:match[2].includes(' disabled'),hidden:match[2].includes(' hidden'),handlers:{},
  addEventListener(name,handler){this.handlers[name]=handler;},setAttribute(name,value){attrs[name]=value;},getAttribute(name){return attrs[name];},
  classList:{add(name){classes.add(name);},remove(name){classes.delete(name);},toggle(name,on){on?classes.add(name):classes.delete(name);}},
  reportValidity(){return true;},matches(){return false;},focus(){},setPointerCapture(){}};
 Object.defineProperty(node,'valueAsNumber',{get(){return Number(this.value);}});
 for(const [key,value] of Object.entries(attrs))if(key.startsWith('data-'))node.dataset[key.slice(5)]=value;
 if(attrs.id)nodes[attrs.id]=node;
 elements.push(node);
}
let body={mode:3,source:'none',safety_state:'WAIT_COMMAND',uptime_ms:10000,wifi_control_allowed:true,control_protocol:1,
 lora:{healthy:false,accepted:1,rejected:0},wifi:{session:'',sequence:0,accepted:0,rejected:0},
 last_written:{tail_front_deg:91,tail_rear_deg:92,left_motor_pct:40,right_motor_pct:-35},
 imu:{uart_ready:true,baud:115200,frames:1,angles_deg:{valid:true,x:45,y:-45,z:90,age_ms:20}},
 attitude_control_supported:true,attitude_control:{enabled:false,yaw_pid_supported:true,body_valid:true,heading_valid:true,yaw_calibrated:true,
  age_ms:20,maximum_age_ms:200,body_roll_deg:2,body_pitch_deg:3,body_yaw_deg:4,pid:{kp:2.5,ki:.02,kd:.1}}};
let fail=false,errorAction='',delayTarget=null,uploadBytes=null;
const context=vm.createContext({
 document:{getElementById:id=>{assert(nodes[id],id);return nodes[id];},querySelectorAll(selector){return elements.filter(node=>selector.split(',').some(part=>{const attr=part.trim().match(/^\[data-([\w-]+)\]$/);return attr&&attr[1] in node.dataset;}));},addEventListener(name,handler){events[name]=handler;}},
 window:{addEventListener(name,handler){events[name]=handler;}},crypto:webcrypto,Uint32Array,Uint8Array,DataView,TextDecoder,AbortController,Date,
 setTimeout(){return 1;},clearTimeout(){},setInterval(){},
 async fetch(url,options={}){
  calls.push({url,options});if(fail)throw Error('offline');
  let reply=url==='/get_ip'?{version:'test',runtime_ready:true}:structuredClone(body),ok=true;
  if(url==='/api/v1/control'){
   const c=JSON.parse(options.body);
   if(c.action===errorAction){reply={error:'control_session_changed'};ok=false;}
   else{
    if(c.action==='target'&&delayTarget)await delayTarget;
    if(c.action==='acquire'){body.wifi.session=c.session;body.wifi.sequence=0;}
    else{assert.equal(c.session,body.wifi.session);assert(c.sequence>body.wifi.sequence);body.wifi.sequence=c.sequence;}
    if(c.action==='yaw_pid')body.attitude_control.pid=c.pid;
    if(c.action==='mode')body.mode=c.mode;
    reply={accepted:true,session:c.session,sequence:body.wifi.sequence};
   }
  }
  return {ok,status:ok?200:409,text:async()=>JSON.stringify(reply)};
 },
 XMLHttpRequest:class{
  constructor(){this.upload={};this.status=200;this.responseText='{"success":true}';}
  open(method,url){assert.equal(method,'POST');assert.equal(url,'/ota_upload');}
  setRequestHeader(name,value){assert.equal(name,'Content-Type');assert.equal(value,'application/octet-stream');}
  send(file){uploadBytes=file;this.upload.onprogress({lengthComputable:true,loaded:file.size,total:file.size});this.onload();}
 }
});
vm.runInContext(script.replace(/updateControls\(\);readIdentity\(\)\.catch\([\s\S]*?\);poll\(\);\s*$/,''),context);
const run=code=>vm.runInContext(code,context);
const controls=()=>calls.filter(c=>c.url==='/api/v1/control').map(c=>JSON.parse(c.options.body));
const settle=async()=>{await new Promise(resolve=>setImmediate(resolve));};
const image=(chip=9)=>{
 const bytes=new Uint8Array(320),view=new DataView(bytes.buffer);bytes[0]=0xe9;view.setUint16(12,chip,true);view.setUint32(32,0xabcd5432,true);
 bytes.set(Buffer.from('test-new'),48);bytes.set(Buffer.from('robotfish'),80);
 return {name:'robotfish.bin',size:bytes.length,slice:(start,end)=>({arrayBuffer:async()=>bytes.slice(start,end).buffer})};
};
(async()=>{
 await run('poll()');assert.equal(controls().length,0,'Opening the page must never take control');
 assert.match(nodes.imu.textContent,/45.0, -45.0, 90.0/);assert.equal(nodes['yaw-value'].textContent,'4.0');
 assert.equal(nodes['tail-front'].textContent,'91.0°');assert(nodes.stop.disabled);
 const saved=nodes['tail-front'].textContent;fail=true;await run('poll()');assert.equal(nodes['tail-front'].textContent,saved);assert.equal(nodes.connection.textContent,'状态读取失败');fail=false;await run('poll()');assert.equal(nodes['last-sync'].dataset.tone,'');
 await run('connect()');assert.equal(controls().at(-1).action,'acquire');assert.equal(nodes.stop.disabled,false);
 run("changeInput('key:w','forward')");await settle();assert.equal(controls().at(-1).axes.surge,35);
 run("changeInput('key:a','left')");await settle();assert.deepEqual(controls().at(-1).axes,{surge:35,yaw:-40,heave:0});
 run("changeInput('key:w',null)");run("changeInput('key:a',null)");await settle();assert.deepEqual(controls().at(-1).axes,{surge:0,yaw:0,heave:0});
 let release;delayTarget=new Promise(resolve=>{release=resolve;});
 run("changeInput('key:w','forward')");await settle();run("changeInput('key:d','right')");const stopped=run('stop()');
 release();await stopped;await settle();delayTarget=null;
 assert.equal(controls().at(-1).action,'stop');assert.equal(run('held.size'),0,'STOP must discard queued movement');
 await run("operate('mode',{mode:2})");assert.deepEqual(controls().slice(-2).map(c=>c.action),['stop','mode']);
 await run('poll()');nodes['pid-kp'].value='3.125';run('pidDirty=true');await run('poll()');assert.equal(nodes['pid-kp'].value,'3.125','Polling must preserve edits');
 await run("operate('yaw_pid',{pid:{kp:3.125,ki:.02,kd:.1}},'pid-result')");await run('poll()');assert.match(nodes['pid-result'].textContent,/回读确认/);
 body.lora.healthy=true;await run('poll()');assert.equal(run('canDrive()'),false);assert.equal(nodes.stop.disabled,false,'LoRa priority must preserve STOP');
 const before=controls().length;await run("operate('yaw_pid',{pid:{kp:99,ki:0,kd:0}})");assert.equal(controls().length,before);
 body.lora.healthy=false;await run('poll()');errorAction='target';run("changeInput('key:w','forward')");await settle();assert.equal(run('session'),'');assert.equal(run('held.size'),0);errorAction='';await run('connect()');
 const beforeFile=calls.length;context.badImage=image(0);await run('chooseFile(badImage)');assert(nodes.upload.disabled);assert.equal(calls.length,beforeFile);
 context.goodImage=image();await run('chooseFile(goodImage)');assert.equal(nodes.upload.disabled,false);await run('upload()');assert.equal(uploadBytes,context.goodImage);assert.match(nodes['ota-result'].textContent,/固件已提交/);assert.equal(run('session'),'');
 body.uptime_ms=100;await run('poll()');assert.match(nodes['ota-result'].textContent,/重新上线/);
 body.attitude_control.body_valid=false;body.imu.angles_deg.valid=false;await run('poll()');assert.equal(nodes['yaw-value'].textContent,'—');
 console.log('PASS: telemetry, stale snapshots, explicit sessions, combined axes, release, queued STOP, modes, PID readback, LoRa priority, session loss, BIN validation and OTA/reboot.');
})().catch(error=>{console.error(error);process.exitCode=1;});
