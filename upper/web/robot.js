'use strict';
let robotState=null,robotIdentity=null,robotPending=false,robotError='',robotSeen=0;
let stationMode='mission',stationEpoch=0,modePending=false,identityPending=false,identityEpoch=0;
let robotfishBoundHost='';
let wifiArmed=false,controlConfigPending=false;
let yawPidDirty=false,yawPidSubmitted=null;
let appliedProfile='',robotTimer=0,robotFrame=0;
let controlSuspended=false,recoveryPending=false,recoveryAttempts=0,recoveryAt=0,robotFailures=0;
let autoConnectPending=false,autoConnectAttempted=false,autoConnectError='';
let localRadio={connected:false,mode:3},radioPolling=false,radioEpoch=0;
let radioPortsBusy=false,radioConfigBusy=false;
function dualLink(){return isRobotFish()&&robotIdentity?.lora_protocol===2;}
function isRobotFish(){return robotIdentity?.firmware==='robotfish';}
function loraOnlyControl(){return stationMode!=='simulation'&&localRadio.control_transport==='lora';}
function loraControlReady(){return stationMode!=='simulation'&&localRadio.connected===true&&(!localRadio.host||!robotHost()||localRadio.host===robotHost());}
function manualControlReady(){return stationMode==='simulation'||loraControlReady()||!loraOnlyControl()&&wifiControlReady();}
function wifiControlReady(){const view=currentRobot();return wifiArmed&&!controlSuspended&&isRobotFish()&&view?.wifi_control_allowed&&view?.control?.connected&&view.control.session_confirmed!==false&&!view.control.recovery_required&&view.control.session===controlSession;}
function clearControlInputs(){heldKeys.clear();heldPointers.clear();desiredAxes={surge:0,yaw:0,heave:0};inputVersion++;sentVersion=inputVersion;}
function suspendControl(error){
  clearControlInputs();
  if(wifiArmed&&!controlSuspended){controlSuspended=true;recoveryAttempts=0;recoveryAt=0;diagnosticEvent('control_link',{message:'suspended',code:error?.code});}
  robotText('robot-command-result',error?.message||'连接暂时中断，已清除旧操作，等待验证');
}
function loseControl(error){suspendControl(error);wifiArmed=false;controlSuspended=false;diagnosticEvent('control_link',{message:'session_lost',code:error?.code});}
async function recoverControl(view,epoch){
  if(!wifiArmed||!controlSuspended||recoveryPending||recoveryAttempts>=3||Date.now()<recoveryAt||view.control?.session_confirmed===false)return;
  if(!view.control?.connected||view.control.session!==controlSession)return;
  const session=controlSession;recoveryPending=true;recoveryAttempts++;
  robotText('robot-command-result','连接已恢复，正在确认全停；旧操作不会续发');
  try{
    const result=await queueControl(()=>session!==controlSession||epoch!==stationEpoch?{skipped:true}:service('station/control',{action:'stop',session,sequence:++controlSequence,client_id:controlClientId}));
    if(result.skipped||session!==controlSession||epoch!==stationEpoch)return;
    if(result.accepted!==true||result.session!==session){const error=Error('全停尚未确认，继续等待连接验证');error.retryable=true;error.code='stop_unconfirmed';throw error;}
    clearControlInputs();controlSuspended=false;recoveryAttempts=0;
    robotText('robot-command-result','控制连接已恢复并确认全停，请重新按键操作');diagnosticEvent('control_link',{message:'recovered'});
  }catch(error){
    if(session!==controlSession||epoch!==stationEpoch)return;
    if(!error.retryable)loseControl(error);
    else{recoveryAt=Date.now()+Math.min(3000,500*2**(recoveryAttempts-1));robotText('robot-command-result',recoveryAttempts>=3?'自动验证未完成；链路稳定后点击“全停”重试，无需重新输入密码':error.message);diagnosticEvent('control_link',{message:'recovery_pending',code:error.code});}
  }finally{recoveryPending=false;}
}
function applyDeviceProfile(){
  const radio=loraControlReady(),profile=JSON.stringify([robotIdentity?.firmware,dualLink(),radio,stationMode,localRadio.control_transport]);
  if(profile===appliedProfile)return;
  appliedProfile=profile;
  const single=isRobotFish();document.body.dataset.device=single?'robotfish':'legacy';
  let panel=$('robotfish-maintenance');
  if(single&&!panel){
    panel=document.createElement('section');panel.id='robotfish-maintenance';
    const note=document.createElement('p');note.textContent='RobotFish 单 ESP32-S3：仅 Wi-Fi OTA。没有 STM32、DAP 或板载相机。选择本工程 build/firmware/robotfish.bin；上传后以换槽和健康启动确认结果。';
    panel.append(note,$('firmware-esp32'));$('page-debug').append(panel);
    const steps=$('firmware-esp32').querySelectorAll('.steps li');steps[1].hidden=true;
    const detail=steps[2].querySelector('small');detail.dataset.legacy=detail.textContent;detail.textContent='ESP32-S3、robotfish 项目名与完整性（BIN 未签名）';
    const badge=$('firmware-esp32').querySelector('.badge');badge.dataset.legacy=badge.textContent;badge.textContent='Wi-Fi OTA';
  }else if(!single&&panel){
    const steps=$('firmware-esp32').querySelectorAll('.steps li');steps[1].hidden=false;
    const detail=steps[2].querySelector('small');detail.textContent=detail.dataset.legacy;
    const badge=$('firmware-esp32').querySelector('.badge');badge.textContent=badge.dataset.legacy;
    $('dev-esp32-firmware').append($('firmware-esp32'));panel.remove();
  }
  $('release-transport').querySelector('option[value="usb"]').disabled=single;
  if(single)$('release-transport').value='wifi';
  $('robot-login').textContent=single?'连接机器鱼':'连接设备';
  document.querySelector('label[for="robot-password"]').textContent=single?'自定义控制密码（默认配置无需填写）':'设备管理密钥';
  $('robot-password').placeholder=single?'仅在修改过默认密码时填写一次':'首次连接输入设备密钥';
  $('robot-remember').checked=single;
  $('login-saved').hidden=single;
  for(const id of ['robot-password','robot-remember','forget-key'])$(id).disabled=false;
  const radioChoice=radio||dualLink()||Boolean(localRadio.port),selected=loraOnlyControl()?'lora':radioChoice?'auto':'wifi';
  $('control-source').disabled=(single||radioChoice)&&!radioChoice;
  if(single||radioChoice)$('control-source').value=selected;
  for(const option of $('control-source').options)option.hidden=single||radioChoice?!(radioChoice?['lora','auto']:[selected]).includes(option.value):option.value==='auto';
  $('control-source').querySelector('option[value="lora"]').textContent='仅 LoRa 控制 · Wi-Fi 只读遥测';
  document.querySelector('label[for="control-source"]').textContent=single||radio?'控制通道':'指令来源（仅仿真）';
  $('control-source').querySelector('option[value="wifi"]').textContent=single?'2.4G Wi-Fi':'2.4G 备用手动';
  $('lora-state').parentElement.hidden=false;$('spi-state').parentElement.hidden=single;
  $('lora-config').hidden=stationMode==='simulation';
  $('wifi-state').parentElement.querySelector('span').textContent=single?(dualLink()?'Wi-Fi 遥测 / 备用控制':'2.4G Wi-Fi 控制与遥测'):'2.4G 备用';
  for(const id of ['vision-board-toggle','network-refresh','network-scan','network-connect'])if($(id))$(id).disabled=single;
  $('control-video-source').querySelector('option[value="s3"]').disabled=single;
  if(single&&$('control-video-source').value==='s3')$('control-video-source').value='vision';
}
const ROBOT_STALE_MS=5000;
const MODE_LABELS={mission:'任务 · Wi-Fi遥测',bench:'台架 · HTTP/SPI',simulation:'仿真 · PC模型'};
function robotAddress(value){
  const parts=String(value||'').trim().split(':');
  if(parts.length>2||!validIP(parts[0]||''))return '';
  if(parts.length===2&&(!/^\d+$/.test(parts[1])||+parts[1]<1||+parts[1]>65535))return '';
  return parts[0]+(parts.length===2?':'+Number(parts[1]):'');
}
function robotHost(){
  const typed=$('robot-host').value.trim();
  if(typed)return robotAddress(typed);
  const discovered=Object.keys(probeLinks.http||{}).filter(ip=>probeLinks.http[ip].state==='online');
  return robotAddress($('management-ip').value)||robotAddress($('host-input').value)||(discovered.length===1?discovered[0]:'');
}
function robotText(id,value){const el=$(id);if(el&&el.textContent!==String(value))el.textContent=value;}
function robotNumber(value,suffix='',digits=0){return typeof value==='number'&&Number.isFinite(value)?value.toFixed(digits)+suffix:'—';}
function robotBadge(id,label,tone='unknown'){const el=$(id);if(el){robotText(id,label);if(el.dataset.state!==tone)el.dataset.state=tone;}}
function renderRadioStatus(){
  const simulated=stationMode==='simulation',radio=loraControlReady(),view=currentRobot(),wifi=!simulated&&view?.online===true;
  const connected=localRadio.connected===true,port=localRadio.port||$('lora-port').value||'未选择串口';
  const label=connected?`发送端已打开 · ${port}`:localRadio.error?'发送端断线':'发送端未连接';
  const reception=wifi?(view.control?.lora_healthy===true?'鱼端收包在线 · 执行未确认':'鱼端 LoRa 未收到近期数据'):'鱼端状态未知（无 Wi-Fi 遥测）';
  robotBadge('lora-control-state',simulated?'LoRa · 仿真':`LoRa ${label}`,radio?'online':localRadio.error?'error':'unknown');
  robotBadge('wifi-telemetry-state',simulated?'Wi-Fi · 仿真':wifi?'Wi-Fi 遥测在线':'Wi-Fi 遥测未连接',wifi?'online':'unknown');
  robotBadge('lora-port-state',label,connected?'online':localRadio.error?'error':'unknown');
  robotText('lora-control-detail',simulated?'当前为离线测试':radio?`可通过 LoRa 手动控制 · ${reception}`:localRadio.error||'连接 LoRa 串口后即可手动控制，无需 Wi-Fi。');
  robotText('lora-port-detail',localRadio.error||`${port} · 115200 / 8N1 · 地址 0x${localRadio.address||'0102'} / 信道 0x${localRadio.channel||'20'} · 已发送 ${localRadio.sent||0} 帧${connected?' · '+reception:''}`);
  if(!simulated)robotText('lora-state',connected?`发送端已打开 · ${reception}`:localRadio.error?'发送端断线':'发送端未连接');
  if(!wifi&&radio)robotText('robot-link-detail','LoRa 发送端已打开 · 鱼端状态未知');
  for(const [id,disabled] of [['lora-connect',simulated||radioPortsBusy||radioConfigBusy||!$('lora-port').value],['lora-disconnect',!connected||radioConfigBusy],['lora-port',simulated||radioPortsBusy||radioConfigBusy],['lora-refresh-ports',radioPortsBusy||radioConfigBusy]])if($(id).disabled!==disabled)$(id).disabled=disabled;
  robotText('lora-connect',connected?'重新连接 LoRa':'连接 LoRa');
  if(typeof renderLoraTiming==='function')renderLoraTiming();
}
async function refreshRadioPorts(){
  if(radioPortsBusy||radioConfigBusy||location.protocol==='file:')return;
  radioPortsBusy=true;renderRadioStatus();robotText('lora-ports-detail','正在扫描串口…');
  const select=$('lora-port'),selected=select.value||(localRadio.connected?localRadio.port:'');
  try{
    const result=await service('station/lora/ports');
    const ports=result.ports;
    select.replaceChildren(new Option(ports.length?'请选择 LoRa 发送端串口':'未检测到串口',''),...ports.map(item=>new Option(`${item.port} · ${item.description}`,item.port)));
    select.value=[...select.options].some(option=>option.value===selected)?selected:'';
    robotText('lora-ports-detail',ports.length?`检测到 ${ports.length} 个串口，请选择 LoRa 发送端；插拔后点击“刷新串口”。`:'未检测到串口，请接入 USB 转 TTL 后点击“刷新串口”。');
  }catch(error){select.replaceChildren(new Option('串口扫描失败，请重试',''));robotText('lora-ports-detail',error.message);}
  finally{radioPortsBusy=false;renderRadioStatus();}
}
async function pollRadioStatus(){
  if(radioPolling||location.protocol==='file:')return;
  radioPolling=true;const epoch=radioEpoch,previous=localRadio.connected;
  try{
    const status=await service('station/lora');if(epoch!==radioEpoch)return;
    localRadio=status;
    if(previous&&!status.connected)clearControlInputs();
  }catch(error){
    if(epoch!==radioEpoch)return;
    localRadio={...localRadio,connected:false,error:error.message};
    if(previous)clearControlInputs();
  }finally{radioPolling=false;applyDeviceProfile();requestRobotRender();}
}
function robotMeter(id,value,min,max,centered=false){
  const el=$(id);if(!Number.isFinite(value)){el.style.width='0%';return;}
  const ratio=Math.max(0,Math.min(1,(value-min)/(max-min)));
  el.style.left=centered?(ratio<.5?ratio*100:50)+'%':'0';el.style.width=(centered?Math.abs(ratio-.5):ratio)*100+'%';
}
function currentRobot(){return robotState&&robotSeen&&Date.now()-robotSeen<=ROBOT_STALE_MS?robotState:null;}
let robotRenderedAt=0;
function requestRobotRender(){
  if(robotFrame||document.hidden)return;
  robotFrame=setTimeout(()=>{robotFrame=requestAnimationFrame(()=>{robotFrame=0;robotRenderedAt=performance.now();renderRobot();});},Math.max(0,200-(performance.now()-robotRenderedAt)));
}
function renderImu(){
  const view=currentRobot(),imu=view?.imu||{},imuAge=Number.isFinite(imu.age_ms)?imu.age_ms+Math.max(0,Date.now()-robotSeen):null;
  const valid=!!view?.online&&imu.valid===true&&imuAge!==null&&imuAge<=500;
  for(const axis of ['roll','pitch','yaw'])robotText('imu-'+axis,robotNumber(valid?imu[axis]:null,'°',2));
  robotText('imu-age',imuAge===null?'—':Math.round(imuAge)+' ms');
  robotBadge('imu-state',valid?(imu.filter_active?'姿态在线 · 已滤波':'姿态在线'):imuAge!==null?'数据过期':'等待数据',valid?'online':'warning');
  robotText('imu-detail',`${imu.baud??'—'} baud · 合法帧 ${imu.frames??'—'} · 校验错误 ${imu.checksum_errors??'—'} · 串口错误 ${imu.receive_errors??'—'}${imu.filter_active?` · 二阶低通，每级 ${imu.filter_stage_hz??'—'} Hz`:''}`);
}
function renderRobot(){
  const view=currentRobot(),valid=!!view?.online,battery=view?.battery||{},motion=view?.motion||{},act=valid?view.actuators||{}:{};
  const simulated=stationMode==='simulation',single=isRobotFish(),source=view?.telemetry_source||({mission:'wifi_spi',bench:'http_spi',simulation:'simulation'})[stationMode];
  robotText('source-label',MODE_LABELS[stationMode]);
  robotText('source-description',loraControlReady()?'LoRa 可独立发送控制指令；Wi-Fi 用于遥测和接收确认。':robotError||(single?(dualLink()?'LoRa 主控，Wi-Fi 遥测与备用控制。':'RobotFish 单 ESP32-S3：2.4G Wi-Fi 控制与遥测。'):simulated?'离线测试，未连接实物。':stationMode==='bench'?'台架遥测；运动控制仍走LoRa。':'Wi-Fi回传遥测；连接 LoRa 后可独立手动控制。'));
  const strip=document.querySelector('.source-strip');if(strip.dataset.source!==source)strip.dataset.source=source;
  robotText('source-age',robotSeen?new Date(robotSeen).toLocaleTimeString('zh-CN',{hour12:false}):'未收到数据');
  robotBadge('global-source',simulated?'仿真模式':stationMode==='bench'?'台架只读':'Wi-Fi遥测',simulated?'warning':valid?'online':'unknown');
  const batteryValid=valid&&battery.valid;
  robotText('robot-battery-voltage',batteryValid?robotNumber(battery.voltage,' V',2):'—');
  robotText('robot-battery-age',batteryValid&&Number.isFinite(battery.age_ms)?battery.age_ms+' ms前':'没有有效采样');
  robotBadge('robot-battery-state',batteryValid?battery.condition_label:'未知',batteryValid?(battery.condition==='normal'?'online':'error'):'unknown');
  robotText('robot-action',valid?motion.action_label||'—':'—');
  robotText('robot-mode-sub',valid?`${motion.mode_label||'未知'} · 输入 ${robotNumber(motion.power,'%')}`:'等待有效遥测');
  robotText('owner-value',({none:'等待指令',lora:'LoRa',wifi:'2.4G',vision:'视觉',unknown:'未知'})[view?.owner||'unknown']);
  robotText('robot-link-detail',valid?`收 ${view.link?.received||0} / 发 ${view.link?.sent||0} 帧`:'LoRa › 2.4G › 视觉');
  const mode=view?.safety_mode||'UNKNOWN';
  robotText('robot-safety-verdict',({UNKNOWN:'未知',WAIT_COMMAND:'等待指令',RUNNING:'运行中',SURFACE:'浮力上浮',BATTERY_STOP:'电池停机'})[mode]||mode);
  if(mode==='LEASE_EXPIRED')robotText('robot-safety-verdict','指令租约到期');
  robotText('robot-safety-detail',mode==='UNKNOWN'?'安全原因尚未由固件上报':mode==='SURFACE'?'双桨中立 · 舵机保持':mode==='BATTERY_STOP'?'全部中立 · 等待新指令':mode==='RUNNING'?'目标生效':'等待新的有效目标');
  if(mode==='LEASE_EXPIRED')robotText('robot-safety-detail','双桨中立 · 舵机回中');
  robotText('reason-value',robotError||view?.reason||'等待状态上报');
  robotText('robot-link-verdict',valid?'遥测在线':view?.reason||'没有有效遥测');
  robotBadge('robot-failsafe-state',mode==='UNKNOWN'?'状态未知':mode==='SURFACE'||mode==='BATTERY_STOP'?'保护生效':'正常',mode==='UNKNOWN'?'unknown':mode==='SURFACE'||mode==='BATTERY_STOP'?'warning':'online');
  robotBadge('robot-link-flag',valid?(simulated?'仿真数据':stationMode==='bench'?'台架遥测':'Wi-Fi遥测'):'无有效数据',valid?'online':'unknown');
  for(const [id,tone] of [['robot-hero-battery',batteryValid?(battery.condition==='normal'?'good':'bad'):'warn'],['robot-hero-motion',valid?'good':'warn'],['robot-hero-link',view?.owner&&view.owner!=='unknown'?'good':'warn'],['robot-hero-safety',mode==='BATTERY_STOP'?'bad':mode==='UNKNOWN'||mode==='SURFACE'?'warn':'good']])if($(id).dataset.tone!==tone)$(id).dataset.tone=tone;
  for(const [key,label,unit] of [['left_fin','left-fin','°'],['right_fin','right-fin','°'],['tail1','tail1','°'],['tail2','tail2','°'],['left_esc','left-esc','%'],['right_esc','right-esc','%']]){
    robotText('robot-'+label,robotNumber(act[key],unit));robotMeter('robot-'+label+'-meter',act[key],unit==='°'?0:-100,unit==='°'?180:100,unit==='%');
  }
  robotText('robot-pitch',robotNumber(act.pitch,'°'));
  renderImu();
  renderAttitudeControl(view);
  renderHeadingExperiment(view);
  const route=view?.control?.route==='lora'?'LoRa 主控':'Wi-Fi 备用';
  robotText('robot-age',valid?Math.max(0,Math.floor((Date.now()-robotSeen)/1000))+'秒前':'—');
  if(simulated)robotText('lora-state',view?.links?.lora?'可用 · 仿真':'断开 · 仿真');
  robotText('wifi-state',simulated?(view?.links?.wifi?'可用 · 仿真':'断开 · 仿真'):(valid?'遥测在线':'遥测未连接'));
  robotText('spi-state',simulated?(view?.links?.spi?'可用 · 仿真':'断开 · 仿真'):(valid?'在线':'离线 / 未知'));
  const connected=!loraOnlyControl()&&wifiControlReady(),radio=loraControlReady(),controllable=manualControlReady();
  if(single){robotText('robot-link-detail',dualLink()?`${route} · 已接收 ${view?.link?.received||0} 帧`:`2.4G Wi-Fi · 已接受 ${view?.link?.received||0} 条指令`);robotText('wifi-state',connected?'控制已连接':wifiArmed&&controlSuspended?'连接待验证':valid?'遥测在线 · 控制未连接':'未连接');}
  document.querySelectorAll('[data-axis]').forEach(el=>el.disabled=!controllable||modePending||controlConfigPending||$('control-source').value==='vision');
  $('control-stop').disabled=!(simulated||radio||!loraOnlyControl()&&wifiArmed)||modePending;
  $('robot-control-mode').disabled=!controllable||modePending||controlConfigPending||view?.attitude_control?.enabled===true;
  const shownMode=radio&&!connected?localRadio.mode:view?.control?.mode;
  if(!controlConfigPending&&[1,2,3].includes(shownMode))$('robot-control-mode').value=String(shownMode);
  robotBadge('robot-gate-badge',simulated?'仿真可控制':radio?'LoRa 可控制':connected?(dualLink()?route:'2.4G 可控制'):single?(wifiArmed&&controlSuspended?'连接待验证':'控制未连接'):'连接 LoRa 或 Wi-Fi 后控制',simulated?'warning':radio||connected?'online':'unknown');
  robotText('robot-gate-note',simulated?'离线测试：输出由STM32原分配算法计算。':connected?(view.control.motion_timeout_ms===0?'按住操作，松开发送停止目标；遥控指令保持至新指令或全停，不因3秒没有变化而停止。':'按住操作，松开发送停止目标；当前鱼端仍使用旧版指令超时策略。'):single?(wifiArmed&&controlSuspended?'认证暂时保留；旧按键已清除，验证同一会话并确认全停后再操作。':view?.wifi_control_allowed?'在设备设置输入控制密码并点击“连接控制”。':'当前固件未开放 Wi-Fi 控制，请先升级固件。'):'LoRa发送端未接入，未向实物发送。');
  if(radio)robotText('robot-gate-note','LoRa 已连接，可按住操作、松开发送停止；无需 Wi-Fi。没有遥测时，鱼端执行结果保持未确认。');
  if(!connected&&!radio&&autoConnectPending)robotText('robot-gate-note','正在自动识别并连接机器鱼…');
  else if(!connected&&!radio&&autoConnectError)robotText('robot-gate-note',autoConnectError);
  renderRadioStatus();
  if(!controllable&&(heldKeys.size||heldPointers.size))releaseControl();

  $('robot-sim-panel').hidden=!simulated;
  if(simulated&&view){for(const key of ['lora','wifi','spi','upgrading'])$('sim-'+key).checked=!!view.links[key];}
  renderRobotSession();
  if(typeof renderControl==='function')renderControl();
}
function renderAttitudeControl(view){
  const panel=$('robot-attitude-settings'),toggle=$('robot-attitude-enabled'),feedback=view?.attitude_control;
  panel.hidden=!isRobotFish();
  const supported=robotIdentity?.yaw_pid_supported===true||feedback?.yaw_pid_supported===true;
  const calibrated=feedback?.yaw_calibrated||robotIdentity?.attitude_control_calibrated===true;
  const ready=serviceAttitudeControl&&manualControlReady()&&supported&&!modePending&&!controlConfigPending;
  toggle.disabled=!ready||(!calibrated&&!feedback?.enabled);
  const pid=feedback?.pid;
  if(yawPidSubmitted&&pid&&['kp','ki','kd'].every(key=>typeof pid[key]==='number'&&Math.abs(pid[key]-yawPidSubmitted[key])<0.0005)){
    yawPidSubmitted=null;yawPidDirty=false;
  }
  for(const key of ['kp','ki','kd']){const input=$('robot-yaw-'+key);input.disabled=!ready;if(!yawPidDirty&&typeof pid?.[key]==='number')input.value=String(Number(pid[key].toFixed(3)));}
  $('robot-yaw-apply').disabled=!ready;
  const tuning=Number.isFinite(feedback?.yaw_deadband_deg)?` · 死区 ±${feedback.yaw_deadband_deg}° · 转向增益 ×${feedback.yaw_actuator_gain??'—'}`:'';
  robotText('robot-yaw-pid-state',!supported?'需要烧录支持 yaw PID 的新固件':yawPidSubmitted?`已提交 Kp ${yawPidSubmitted.kp} / Ki ${yawPidSubmitted.ki} / Kd ${yawPidSubmitted.kd}，等待鱼端回读确认`:
    pid&&typeof pid.kp==='number'?`鱼端参数：Kp ${Number(pid.kp.toFixed(3))} · Ki ${Number(pid.ki.toFixed(3))} · Kd ${Number(pid.kd.toFixed(3))} · ${feedback.pid_period_ms||'—'} ms${tuning} · 混合驱动${yawPidDirty?' · 输入尚未应用':''}`:'等待鱼端 PID 参数');
  if(!controlConfigPending&&typeof feedback?.enabled==='boolean')toggle.checked=feedback.enabled;
  const notes={disabled:'已关闭；运动保持手动控制',uncalibrated:'IMU 安装轴向或执行器纠偏方向尚未标定，未参与控制',
    waiting_motion:'已开启，但尚未允许闭环；倒退期间暂停，静止保持需烧录 2.6.1 固件',imu_unavailable:'IMU 姿态或角速度不可用，姿态辅助暂停，保留手动控制',
    imu_stale:'IMU 数据已过期，姿态辅助暂停，保留手动控制',invalid_sample:'IMU 数据无效，姿态辅助暂停',
    manual_pitch:'手动浮沉优先，俯仰辅助暂时让出',active:'鱼端姿态控制就绪'};
  notes.manual_yaw='手动转向优先，松开后重新记录相对航向';notes.heading_unavailable='当前姿态无法判断航向，航向保持暂停';
  let message=!supported?'当前固件未提供 yaw PID，请先更新固件':!calibrated?notes.uncalibrated:!view?'等待鱼端状态':notes[feedback?.reason]||'等待鱼端闭环状态';
  if(feedback?.enabled&&typeof feedback.age_ms==='number'&&feedback.maximum_age_ms>0&&feedback.age_ms+Date.now()-robotSeen>feedback.maximum_age_ms)
    message='姿态控制回传已过期，当前反馈状态尚未确认';
  else if(feedback?.enabled===true&&feedback.yaw_active===true&&feedback.heading_locked===true&&feedback.body_valid){
    const u=feedback.yaw_correction_pct;
    message=`yaw PID 生效 · ${u>0?'向右':u<0?'向左':'保持'}修正 ${robotNumber(Math.abs(u),'%',1)}`;
  }
  if(feedback?.body_valid&&feedback.age_ms+Date.now()-robotSeen<=feedback.maximum_age_ms)
    message+=` · 机体横滚 ${robotNumber(feedback.body_roll_deg,'°',1)} / 俯仰 ${robotNumber(feedback.body_pitch_deg,'°',1)}`;
  if(feedback?.heading_valid)message+=` · yaw ${robotNumber(-feedback.body_yaw_deg,'°',1)}（左转为正）`;
  if(feedback?.heading_locked)message+=` → 目标 ${robotNumber(-feedback.heading_target_deg,'°',1)} · 误差 ${robotNumber(-feedback.yaw_error_deg,'°',1)}`;
  else if(feedback?.enabled&&feedback.manual_yaw)message+=' · 航向保持让出人工转向';
  if(feedback?.tilt_preview_valid&&!feedback.tilt_outputs_connected)message+=` · pitch / roll 仅预览 ${robotNumber(feedback.pitch_preview_pct,'%',1)} / ${robotNumber(feedback.roll_preview_pct,'%',1)}，未接入输出`;
  if(!serviceAttitudeControl)message='本地后台尚未提供姿态辅助接口，需要更新后台后才能切换';
  robotText('robot-attitude-state',message);
}

function renderHeadingExperiment(view){
  const supported=view?.attitude_control?.heading_target_supported===true||robotIdentity?.heading_target_supported===true;
  const reference=view?.heading_reference||{};
  $('heading-send').disabled=headingRequestPending||!supported||!manualControlReady()||controlConfigPending||!reference.valid||!reference.imu_valid;
  $('heading-zero').disabled=headingRequestPending||!view?.attitude_control?.heading_valid||controlConfigPending;
  const number=value=>typeof value==='number'?value.toFixed(1)+'°':'—';
  robotText('heading-experiment-state',!supported?'指定角度需要 S3 2.7.0 固件':!reference.valid?
    '先在视觉页标定鱼头，或用当前 IMU 建立 0°。':
    `当前 ${number(reference.current_deg)} · 鱼端目标 ${number(reference.target_deg)} · ${reference.target_confirmed?'目标已回读确认':'等待目标回读'} · ${reference.source==='VISUAL_HEAD_CALIBRATION'?'鱼头标定零位':'手动 IMU 零位'}`);
}
function renderRobotSession(){
  const host=robotHost();robotText('robot-device-host',host||'—');robotText('robot-session-host',stationMode==='simulation'?'SIMULATION / PC模型':host||'未选择管理设备');
  if(!host){robotBadge('robot-device-badge','自动查找','unknown');robotText('robot-device-detail','先连接机器鱼 Wi-Fi；上位机会自动识别并连接。');}
  else if(!robotIdentity){robotBadge('robot-device-badge','未连接','unknown');robotText('robot-device-detail','找不到设备，或尚未检测；点击“检测”检查此地址。');}
  else{robotBadge('robot-device-badge',robotIdentity.authenticated?'已登录':'设备在线',robotIdentity.authenticated?'online':'unknown');robotText('robot-device-detail',robotIdentity.authenticated?'管理会话已建立；LoRa 主控，丢失后 Wi-Fi 备用。':'设备可达，请输入管理密钥登录，以便 LoRa 丢失后使用 Wi-Fi。');}
  for(const [id,key] of [['robot-version','version'],['robot-slot','running_slot'],['robot-ssid','ssid']])robotText(id,robotIdentity?.[key]||'—');
  $('login-saved').disabled=!robotIdentity?.key_saved;
  if(isRobotFish()){
    const online=currentRobot()?.online===true;
    robotBadge('robot-device-badge',online?(wifiControlReady()?'2.4G 控制已连接':'RobotFish 遥测在线'):'RobotFish 遥测已断线',online?'online':'error');
    robotText('robot-device-detail',online?(dualLink()?'单 ESP32-S3；LoRa 可独立发送，Wi-Fi 用于遥测与备用控制。':'单 ESP32-S3；遥测免登录。输入控制密码并连接后可操控。'):'未收到有效 Wi-Fi 遥测；LoRa 发送端状态单独显示，鱼端状态未知。');
  }
  applyDeviceProfile();
}
async function pollIdentity(){
  if(identityPending||autoConnectPending)return;const host=robotHost(),epoch=identityEpoch;
  if(stationMode==='simulation'){robotIdentity=null;renderRobotSession();return;}
  if(!host){robotIdentity=null;renderRobotSession();return;}
  identityPending=true;
  try{const reply=await service('device/identity?host='+encodeURIComponent(host));if(epoch===identityEpoch){robotIdentity=reply;if(isRobotFish()&&robotfishBoundHost!==host){robotfishBoundHost=host;await switchStationMode(stationMode);}}}
  catch(e){if(epoch===identityEpoch){robotText('robot-login-state',e.message);}}
  finally{identityPending=false;renderRobotSession();}
}
async function autoConnect(force=false){
  if(location.protocol==='file:'||autoConnectPending||modePending||stationMode==='simulation'||(!force&&autoConnectAttempted))return;
  autoConnectPending=true;autoConnectError='';const epoch=identityEpoch;
  robotText('robot-login-state','正在自动查找并连接机器鱼…');
  try{
    if(force)await stopControl();
    let host=robotAddress($('robot-host').value);
    if(!host){
      const discovery=await service('device/discover');if(epoch!==identityEpoch)return;
      if(!discovery.robotfish){robotText('robot-login-state','等待连接机器鱼 Wi-Fi，连上后会自动识别');return;}
      host=discovery.robotfish.host;
      robotIdentity=discovery.robotfish;$('robot-host').value=host;renderRobotSession();
    }
    if(epoch!==identityEpoch)return;
    autoConnectAttempted=true;
    const connect=()=>epoch!==identityEpoch?{skipped:true}:service('device/auto-connect',{host,session:controlSession,force});
    const result=await (loraControlReady()?connect():queueControl(connect));
    if(result.skipped||epoch!==identityEpoch)return;
    if(!loraControlReady()){clearControlInputs();stationEpoch++;}identityEpoch++;
    $('robot-host').value=result.identity.host;robotfishBoundHost=result.identity.host;robotIdentity=result.identity;
    controlSession=result.session;controlSequence=Math.max(controlSequence,result.sequence||0);
    robotState=result.view;stationMode=result.view.mode||stationMode;robotSeen=Date.now();robotError='';
    wifiArmed=result.accepted===true;controlSuspended=!!result.view.control?.recovery_required;recoveryAttempts=0;
    robotText('robot-login-state',result.telemetry_only?'只读遥测已连接；运动指令仅经 LoRa':'机器鱼已自动连接，可以直接操作');
    diagnosticEvent('control_link',{message:'auto_connected'});
    await pollRobot();
  }catch(error){
    if(epoch===identityEpoch){
      robotText('robot-login-state',error.message);
      autoConnectError=error.message;
      if(error.code==='custom_password_required')$('connection-advanced').open=true;
    }
  }finally{autoConnectPending=false;renderRobot();}
}
async function pollRobot(){
  if(location.protocol==='file:'||robotPending)return;
  clearTimeout(robotTimer);robotPending=true;const epoch=stationEpoch,started=performance.now();
  try{const view=await service('station');if(epoch!==stationEpoch)return;robotState=view;stationMode=view.mode||stationMode;$('station-mode').value=stationMode;robotError='';robotSeen=Date.now();robotFailures=0;
    if(wifiArmed&&view.firmware==='robotfish'){
      if(view.control?.session_confirmed!==false&&(!view.control?.connected||view.control.session!==controlSession)){
        if(loraControlReady()){wifiArmed=false;controlSuspended=false;}else loseControl({message:'控制会话已变化，请重新连接控制',code:'control_session_changed'});
      }else if(view.control?.recovery_required){
        if(loraControlReady())controlSuspended=true;else suspendControl({message:'上次指令结果未确认，正在验证全停',code:'recovery_required'});
      }
      if(!loraControlReady())await recoverControl(view,epoch);
    }
  }
  catch(e){if(epoch===stationEpoch){robotFailures++;if(wifiArmed){if(loraControlReady())controlSuspended=true;else suspendControl(e);}robotState=null;robotSeen=0;robotError=e.message;}}
  finally{
    robotPending=false;requestRobotRender();if(typeof updateDataTelemetry==='function')updateDataTelemetry(currentRobot());
    const interval=robotFailures?Math.min(3000,250*2**Math.min(robotFailures-1,4)):document.hidden?1000:isRobotFish()?100:250;
    robotTimer=setTimeout(pollRobot,Math.max(0,interval-(performance.now()-started)));
  }
}
async function switchStationMode(mode){
  if(modePending)return;const stopping=stopControl();modePending=true;await stopping;stationEpoch++;robotState=null;robotSeen=0;renderRobot();
  try{const view=await service('station/mode',{mode,host:robotHost()});stationMode=mode;robotState=view;robotSeen=Date.now();robotError='';log('切换工作模式：'+MODE_LABELS[mode],mode==='simulation'?'warn':'info');}
  catch(e){toast(e.message);log(e.message,'warn');}
  finally{modePending=false;$('station-mode').value=stationMode;renderRobot();await pollRobot();}
}
async function robotLogin(useSaved=false){
  if(!useSaved&&!$('robot-password').value)return autoConnect(true);
  const host=robotHost();if(!host)return toast('请先填写有效设备IP');const password=$('robot-password').value;
  if(!useSaved&&!password)return toast('请输入设备管理密钥');
  const epoch=++identityEpoch;
  try{
    await stopControl();if(epoch!==identityEpoch)return;
    wifiArmed=false;controlSession=crypto.randomUUID();controlSequence=0;sentVersion=inputVersion;
    const session=controlSession;
    const connect=()=>epoch!==identityEpoch?{skipped:true}:service('device/login',{host,password,use_saved:useSaved,remember:$('robot-remember').checked,session});
    const result=await (loraControlReady()?connect():queueControl(connect));
    if(result.skipped||epoch!==identityEpoch)return;
    await pollIdentity();if(epoch!==identityEpoch)return;
    await switchStationMode(stationMode);if(epoch!==identityEpoch)return;
    wifiArmed=result.accepted===true&&result.session===controlSession;autoConnectAttempted=true;controlSuspended=false;recoveryAttempts=0;robotText('robot-login-state',wifiArmed?'控制会话已连接':result.telemetry_only?'只读遥测已连接，控制仅经 LoRa':'管理登录成功');log('已连接设备 '+host,'ok');renderRobot();
  }
  catch(e){if(epoch===identityEpoch){robotText('robot-login-state',e.message);toast(e.message);}}
  finally{$('robot-password').value='';}
}

let controlSession=crypto.randomUUID(),controlSequence=0,controlQueue=Promise.resolve(),desiredAxes={surge:0,yaw:0,heave:0},desiredSource='lora',inputVersion=0,sentVersion=0,targetPending=false,lastRenewal=0;
const controlClientId=crypto.randomUUID();
const heldKeys=new Set(),heldPointers=new Map();
const axisBindings={forward:[1,0,0],back:[-1,0,0],left:[0,-1,0],right:[0,1,0],up:[0,0,1],down:[0,0,-1],'front-left':[1,-1,0],'front-right':[1,1,0],'back-left':[-1,-1,0],'back-right':[-1,1,0]};
const keyboardAxes={w:'forward',s:'back',a:'left',d:'right',r:'up',f:'down'};
let robotHistory=[];
function renderControl(){
 const actual=currentRobot()?.control?.axes;
 robotText('control-axis-value',actual?`已确认：前后 ${actual.surge} · 转向 ${actual.yaw} · 浮沉 ${actual.heave}`:loraControlReady()?`LoRa 指令目标：前后 ${desiredAxes.surge} · 转向 ${desiredAxes.yaw} · 浮沉 ${desiredAxes.heave} · 鱼端执行未确认`:'目标尚未确认');
 document.querySelectorAll('[data-axis]').forEach(button=>button.classList.toggle('pressed',[...heldKeys,...heldPointers.values()].includes(button.dataset.axis)));
}
function queueControl(callback){const result=controlQueue.then(callback);controlQueue=result.catch(()=>{});return result;}
async function configureControl(action,value){
 if(action!=='yaw_pid')window.dispatchEvent(new Event('stationmanual'));
 controlConfigPending=true;renderRobot();
 try{const result=await queueControl(()=>service('station/control',{action,value,session:controlSession,sequence:++controlSequence,client_id:controlClientId}));if(action==='mode'&&(result.queued||result.accepted))localRadio.mode=value;if(action==='yaw_pid'&&(result.queued||result.accepted))yawPidSubmitted={...value};if(action==='source')localRadio.control_transport=result.control_transport;const message=result.queued?'已排队经 LoRa 发送，鱼端执行未确认':result.waiting?'等待通道或参数发送，请稍后重试':result.message;robotText('control-configuration-state',message);log(message,'ok');pollRobot();}
 catch(error){if(isRobotFish()&&!['unsupported_control','attitude_not_calibrated','invalid_pid'].includes(error.code)&&!['heading_zero','heading_target'].includes(action)){if(error.retryable)suspendControl(error);else loseControl(error);}robotText('control-configuration-state',error.message);toast(error.message);$('robot-control-mode').value=String(loraControlReady()?localRadio.mode:currentRobot()?.control?.mode||3);$('control-source').value=loraControlReady()?'auto':isRobotFish()?(dualLink()?'auto':'wifi'):currentRobot()?.control?.source||'lora';}
 finally{controlConfigPending=false;renderRobot();}
}
function updateAxes(sourceOverride){
 window.dispatchEvent(new Event('stationmanual'));
 const total=[0,0,0];for(const key of [...heldKeys,...heldPointers.values()]){const vector=axisBindings[key];if(vector)vector.forEach((value,i)=>total[i]+=value);}
 desiredAxes={surge:Math.sign(total[0])*Number($('robot-control-power').value),yaw:Math.sign(total[1])*Number($('robot-control-strength').value),heave:Math.sign(total[2])*100};desiredSource=sourceOverride||$('control-source').value;inputVersion++;pumpTarget();renderControl();
}
function releaseControl(sourceOverride){heldKeys.clear();heldPointers.clear();updateAxes(sourceOverride);}
async function stopControl(){
 heldKeys.clear();heldPointers.clear();desiredAxes={surge:0,yaw:0,heave:0};inputVersion++;sentVersion=inputVersion;renderControl();
 if(stationMode==='simulation'){sentVersion--;await pumpTarget();return;}
 if(!loraControlReady()&&(loraOnlyControl()||!wifiArmed))return;
 try{const result=await queueControl(()=>service('station/control',{action:'stop',session:controlSession,sequence:++controlSequence,client_id:controlClientId}));if(result.accepted===true&&result.session===controlSession){controlSuspended=false;recoveryAttempts=0;}else if(result.queued&&!loraControlReady())suspendControl({message:'停止帧已排队（LoRa），等待鱼端确认',code:'stop_unconfirmed'});robotText('robot-command-result',result.queued?'LoRa 停止帧已排队，鱼端执行未确认':'已全停');}
 catch(error){if(error.retryable)suspendControl(error);else loseControl(error);robotText('robot-command-result',error.message);log(error.message,'warn');}
}
async function pumpTarget(){
 if(targetPending||modePending||!manualControlReady())return;
 const active=Object.values(desiredAxes).some(value=>value!==0),now=Date.now();
 if(inputVersion===sentVersion&&(!active||now-lastRenewal<150))return;
 targetPending=true;const version=inputVersion,axes={...desiredAxes},source=desiredSource,epoch=stationEpoch;
 try{const result=await queueControl(()=>version!==inputVersion||controlSuspended&&!loraControlReady()?{skipped:true}:service('station/target',{axes,source,session:controlSession,sequence:++controlSequence,client_id:controlClientId}));
   if(result.skipped)return;
   if(epoch===stationEpoch&&version===inputVersion){sentVersion=version;lastRenewal=Date.now();robotText('robot-command-result',result.queued?`目标已排队（LoRa），等待鱼端遥测确认 #${result.id}`:result.waiting?'等待通道切换，下次新目标继续发送':`目标已接受（${result.simulated?'仿真':'Wi-Fi 备用'}） #${result.id}`);}
 }catch(error){if(epoch===stationEpoch){if(error.retryable)suspendControl(error);else loseControl(error);robotText('robot-command-result',error.message);log(error.message,'warn');renderControl();renderRobot();}}
 finally{targetPending=false;if(inputVersion!==sentVersion)pumpTarget();}
}
function renderRobotHistory(){$('robot-history').replaceChildren(...robotHistory.slice(-12).reverse().map(entry=>{const row=document.createElement('div');row.textContent=entry;return row;}));}
for(const connected of [true,false])$('lora-'+(connected?'connect':'disconnect')).onclick=async()=>{
 if(radioConfigBusy)return;
 const port=$('lora-port').value;
 if(connected&&!port){toast('请先扫描并选择 LoRa 发送端串口');return;}
 radioConfigBusy=true;renderRadioStatus();
 try{await stopControl();clearControlInputs();radioEpoch++;localRadio=await queueControl(()=>service('station/lora',{connected,port,address:$('lora-address').value.trim(),channel:$('lora-channel').value.trim()}));radioEpoch++;applyDeviceProfile();renderRobot();await pollRadioStatus();}
 catch(error){robotText('lora-port-detail',error.message);toast(error.message);}
 finally{radioConfigBusy=false;renderRadioStatus();}
};
$('lora-refresh-ports').onclick=refreshRadioPorts;
$('lora-port').onchange=renderRadioStatus;
$('lora-open-settings').onclick=()=>{go('network');$('lora-config').scrollIntoView({behavior:'smooth',block:'start'});};
$('station-mode').onchange=e=>switchStationMode(e.target.value);
$('robot-login').onclick=()=>robotLogin();$('login-saved').onclick=()=>robotLogin(true);$('robot-refresh').onclick=()=>{autoConnectAttempted=false;autoConnect();};
$('forget-key').onclick=async()=>{try{autoConnectAttempted=true;identityEpoch++;await stopControl();wifiArmed=false;await service('device/forget',{host:robotHost()});robotIdentity=null;robotText('robot-login-state','已退出并删除该设备的本机密钥');await pollIdentity();}catch(error){toast(error.message);}};
let hostTimer;$('robot-host').addEventListener('input',()=>{identityEpoch++;autoConnectAttempted=true;stopControl().finally(()=>{wifiArmed=false;robotIdentity=null;clearTimeout(hostTimer);hostTimer=setTimeout(pollIdentity,400);renderRobotSession();});});
$('robot-control-mode').onchange=e=>configureControl('mode',Number(e.target.value));
$('robot-attitude-enabled').onchange=e=>configureControl('attitude',e.target.checked);
let headingRequestPending=false;
$('heading-zero').onclick=()=>configureControl('heading_zero',null);
$('heading-send').onclick=async()=>{
  if(headingRequestPending)return;
  const angle=$('heading-angle'),surge=$('heading-surge');if(!angle.reportValidity()||!surge.reportValidity())return;
  const target={angle_deg:Number(angle.value),surge_pct:Number(surge.value)};
  headingRequestPending=true;renderRobot();
  clearControlInputs();const version=inputVersion,epoch=stationEpoch;
  try{
    await controlQueue;
    if(version!==inputVersion||epoch!==stationEpoch)return;
    const sequence=controlSequence;
    if($('heading-auto-sample').checked&&typeof beginVisionSamples==='function'){
      await beginVisionSamples();
    }
    if(version!==inputVersion||epoch!==stationEpoch||sequence!==controlSequence)return;
    await configureControl('heading_target',target);
  }catch(error){toast(error.message);}
  finally{headingRequestPending=false;renderRobot();}
};
for(const key of ['kp','ki','kd'])$('robot-yaw-'+key).oninput=()=>{yawPidDirty=true;};
$('robot-yaw-apply').onclick=()=>{
 const pid={};for(const key of ['kp','ki','kd']){const input=$('robot-yaw-'+key);if(input.value===''||!input.checkValidity()){input.reportValidity();return;}pid[key]=Number(input.value);}
 configureControl('yaw_pid',pid);
};
$('robot-yaw-history').onclick=async()=>{
 try{const response=await fetch('/api/data/pid/export');if(!response.ok)throw Error((await response.json()).error);downloadBlob(await response.blob(),'robofish-yaw-pid-history.csv');}
 catch(error){toast(error.message);}
};
$('control-source').onchange=e=>{if(heldKeys.size||heldPointers.size)releaseControl(desiredSource);configureControl('source',e.target.value);};
for(const [id,output] of [['robot-control-power','robot-power-value'],['robot-control-strength','robot-strength-value']])$(id).oninput=()=>{robotText(output,$(id).value+'%');if(heldKeys.size||heldPointers.size)updateAxes();};
document.querySelectorAll('[data-axis]').forEach(button=>{
 button.onpointerdown=event=>{event.preventDefault();button.setPointerCapture(event.pointerId);heldPointers.set(event.pointerId,button.dataset.axis);updateAxes();};
 const end=event=>{if(heldPointers.delete(event.pointerId))updateAxes();};button.onpointerup=end;button.onpointercancel=end;button.onlostpointercapture=end;
});
$('control-stop').onclick=()=>{window.dispatchEvent(new Event('stationmanual'));stopControl();robotHistory.push(new Date().toLocaleTimeString()+' 全停');renderRobotHistory();};
$('robot-clear-history').onclick=()=>{robotHistory=[];renderRobotHistory();};
window.addEventListener('keydown',event=>{if(state.page!=='overview'||/INPUT|TEXTAREA|SELECT/.test(event.target.tagName))return;if(event.code==='Space'){event.preventDefault();if(!event.repeat)$('control-stop').click();return;}const key=keyboardAxes[event.key.toLowerCase()];if(key&&!event.repeat&&!modePending&&manualControlReady()&&$('control-source').value!=='vision'){event.preventDefault();heldKeys.add(key);updateAxes();}});
window.addEventListener('keyup',event=>{const key=keyboardAxes[event.key.toLowerCase()];if(key&&heldKeys.delete(key))updateAxes();});
window.addEventListener('blur',()=>{if(heldKeys.size||heldPointers.size)releaseControl();});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&(heldKeys.size||heldPointers.size))releaseControl();});
window.addEventListener('stationpage',event=>{if(event.detail!=='overview'&&(heldKeys.size||heldPointers.size))releaseControl();if(event.detail==='network'){pollIdentity();refreshRadioPorts();}});
for(const key of ['lora','wifi','spi','upgrading'])$('sim-'+key).onchange=async event=>{try{robotState=await service('station/simulate',{changes:{[key]:event.target.checked}});robotSeen=Date.now();renderRobot();}catch(error){toast(error.message);}};
$('robot-sim-fault').onchange=async event=>{try{robotState=await service('station/simulate',{changes:{battery:event.target.value}});robotSeen=Date.now();renderRobot();}catch(error){toast(error.message);}};
setInterval(()=>{if(manualControlReady())pumpTarget();},75);
setInterval(()=>{if(!document.hidden&&state.page==='overview')renderImu();},100);
setInterval(()=>{if(!document.hidden){requestRobotRender();if(state.page==='network')pollIdentity();}},4000);
renderRobot();pollRobot();pollRadioStatus();refreshRadioPorts();
setInterval(()=>pollRadioStatus(),500);
setTimeout(()=>autoConnect(),0);
setInterval(()=>{if(!document.hidden&&!autoConnectAttempted&&!wifiArmed)autoConnect();},4000);
