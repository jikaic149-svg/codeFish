'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const html=fs.readFileSync(path.resolve(__dirname,'../../main/control.html'),'utf8');
const script=html.match(/<script>([\s\S]*?)<\/script>/)[1];
new vm.Script(script);
const nodes={},calls=[],timers=[];
let body={mode:2,safety_state:'WAIT_COMMAND',lora:{accepted:1,last_ack:'910,0,1,1,0,2,1\n'},
 command_result:{available:true,accepted:true,source:0,sequence:1,code:2,result:'accepted'},
 last_written:{left_motor_pct:0,right_motor_pct:0},imu:{uart_ready:true,baud:115200,frames:0}};
let fail=false;
const element=id=>nodes[id]||(nodes[id]={textContent:'',files:[],addEventListener(name,handler){this[name]=handler;}});
const context=vm.createContext({
 document:{getElementById:element},
 setTimeout(fn,ms){assert.equal(ms,500);timers.push(fn);},
 async fetch(url,options={}){calls.push({url,options});if(fail)throw Error('offline');return {ok:true,json:async()=>body,text:async()=>'{"success":true}'};}
});
vm.runInContext(script.replace(/poll\(\);\s*$/,''),context);
(async()=>{
 await vm.runInContext('poll()',context);
 assert.match(nodes['command-result'].textContent,/accepted/);
 assert.match(nodes.imu.textContent,/115200/);
 assert.equal(calls[0].url,'/motion_status');
 const saved=nodes.outputs.textContent;
 fail=true;await vm.runInContext('poll()',context);
 assert.match(nodes.connection.textContent,/offline/);assert.equal(nodes.outputs.textContent,saved);
 fail=false;body={...body,imu:{uart_ready:true,baud:115200,frames:1,angles_deg:{valid:true,x:45,y:-45,z:90,age_ms:20}}};
 await vm.runInContext('poll()',context);assert.match(nodes.imu.textContent,/45.0, -45.0, 90.0/);
 assert.match(nodes.attitude.textContent,/尚未提供/);
 body={...body,attitude_control_supported:true,attitude_control:{enabled:true,reason:'uncalibrated',pitch_calibrated:false,roll_calibrated:false,pitch_correction_pct:0,roll_correction_pct:0}};
 await vm.runInContext('poll()',context);assert.match(nodes.attitude.textContent,/未标定/);
 await nodes.upload.click();assert.equal(calls.filter(c=>c.options.method==='POST').length,0);
 nodes.firmware.files=[{name:'robotfish.bin'}];await nodes.upload.click();
 assert.equal(calls.at(-1).url,'/ota_upload');assert.equal(nodes.upload.disabled,false);
 assert(!calls.some(c=>c.url==='/api/v1/control'));
 assert.equal(timers.length,4);
 console.log('PASS: telemetry rendering, absent/valid IMU, offline preservation, OTA and no Wi-Fi motion requests');
})().catch(e=>{console.error(e);process.exitCode=1;});
