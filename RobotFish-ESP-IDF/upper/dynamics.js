'use strict';
const $=id=>document.getElementById(id);
const clientId=crypto.randomUUID?.()||'dynamics-'+Date.now()+'-'+Math.random().toString(16).slice(2);
let token='',sequence=0,segmentId='',current={active:false},mode=1,motion='straight',level=20,recipe='balanced';
let staticBiasLevel=40;
let busy=false,held=false,generation=0,driveRequest=null,startRequest=null,polling=false,connected=false;
let vision={running:false},visionBusy=false,visionPolling=false,visionPointerDown=false,visionPointerAt=0,visionPointerQueue=Promise.resolve(),visionPointerSelection=null,headingSynced='',headingSyncError='';
const finite=value=>typeof value==='number'&&Number.isFinite(value);
const format=(value,unit='',digits=2)=>finite(value)?value.toFixed(digits)+(unit?' '+unit:''):'—';
const phaseNames={pre:'动作前 · 等待按住',drive:'驱动中',coast:'滑行 / 余转 · 持续记录',finished:'本段已完成'};
const phaseColors={pre:'#94a5ad',drive:'#087c70',coast:'#b46c25'};
const summaryFields=[['steady_speed_mps','稳定速度','m/s',3],['steady_yaw_rate_dps','稳定角速度','°/s',1],['displacement_m','总位移','m',3],['lateral_drift_m','侧向漂移','m',3],['coast_distance_m','松开后滑行距离','m',3],['residual_turn_deg','松开后余转','°',1],['confirmed_coast_distance_m','输出归零后滑行','m',3],['confirmed_residual_turn_deg','输出归零后余转','°',1],['turn_radius_m','圆弧半径','m',3],['fit_rmse_m','圆弧拟合误差','m',3]];
function notice(message,error=false){$('notice').textContent=message;$('notice').classList.toggle('error',error);}
async function request(path,body,options={}){
  if(body!==undefined&&!token){const response=await fetch('/api/session');const data=await response.json();if(!response.ok)throw Error(data.error||'本地服务不可用');token=data.token;}
  if(options.signal?.aborted)throw new DOMException('操作已取消','AbortError');
  const controller=new AbortController(),cancel=()=>controller.abort(),timeout=setTimeout(cancel,body?.action==='drive'?2500:['vision/start','vision/cameras'].includes(path)?30000:10000);
  options.signal?.addEventListener('abort',cancel,{once:true});
  try{const response=await fetch('/api/'+path,{method:body===undefined?'GET':'POST',headers:{'Content-Type':'application/json','X-AmphiLink-Token':token},body:body===undefined?undefined:JSON.stringify(body),...options,signal:controller.signal});
    const data=await response.json();if(!response.ok){if(response.status===403)token='';throw Error(data.error||'请求未完成');}return data;
  }finally{clearTimeout(timeout);options.signal?.removeEventListener('abort',cancel);}
}
function settings(){
  for(const id of ['tail','bias','surge','diff'])if(!$(id).reportValidity()||!Number.isFinite(Number($(id).value))||!$(id).value.trim())throw Error('请填写有效参数。');
  const result={mode,motion,tail_drive_pct:Number($('tail').value),tail_bias_pct:Number($('bias').value),prop_surge_pct:Number($('surge').value),prop_diff_pct:Number($('diff').value),baseline:$('baseline').checked,pool_width_m:3,pool_height_m:1.6,camera_height_m:2.67};
  if(mode===3&&recipe==='static'&&(result.tail_drive_pct!==0||result.prop_diff_pct!==0||result.baseline))throw Error('静态尾角测量要求尾摆、桨差动均为 0，且关闭旧混合基线。');
  if(mode===3&&recipe==='swim'&&(result.tail_drive_pct!==result.prop_surge_pct||result.prop_surge_pct<0||result.prop_diff_pct!==0||result.baseline))throw Error('摆尾循迹配方要求尾摆与共同推进相同且非负，差动为0。');
  if(result.baseline&&(mode!==3||result.tail_drive_pct!==result.prop_surge_pct||result.prop_surge_pct<0||result.tail_bias_pct!==result.prop_diff_pct))throw Error('旧混合基线要求：尾摆 = 共同推进 ≥ 0，尾偏置 = 差动；请重新选择档位或调整数值。');
  return result;
}
function command(action,extra={},options={}){return request('dynamics/action',{action,client_id:clientId,sequence:++sequence,segment_id:segmentId||undefined,...extra},options);}
function takeState(data){if(data?.state)data=data.state;if(typeof data?.active==='boolean'){current=data;segmentId=data.segment_id||segmentId;render();}}
function applyPreset(){
  const direction=motion==='left'?-1:motion==='right'?1:0;
  let tail=0,bias=0,surge=0,diff=0;
  if(mode===1){tail=level;bias=direction*level;}
  if(mode===2){surge=direction?0:level;diff=direction*level;}
  const staticRecipe=mode===3&&recipe==='static',tailStudy=mode===3&&['static','swim'].includes(recipe);
  if(tailStudy){surge=Number($('surge').value);tail=staticRecipe?0:surge;bias=direction*staticBiasLevel;$('baseline').checked=false;}
  else if(mode===3){const tailShare=recipe==='tail'?.75:recipe==='props'?.25:.5;tail=Math.round(level*tailShare);surge=level-tail;bias=direction*level;diff=direction*level;if($('baseline').checked){tail=level;surge=level;}}
  for(const [id,value] of Object.entries({tail,bias,surge,diff}))$(id).value=String(value);
  $('recipe-wrap').hidden=mode!==3;$('baseline').disabled=mode!==3||tailStudy;
  $('static-wrap').hidden=!tailStudy;$('level-wrap').hidden=tailStudy;
  $('tail').disabled=$('diff').disabled=tailStudy;
  $('surge').min=tailStudy?'0':'-100';
  $('tail-recipe-note').textContent=staticRecipe?'静态对照：尾摆与差动均为0，只改变尾偏。':'摆尾循迹配方：尾摆量＝双桨共同推进量，差动为0；转向只改变摆动中心偏置。';
  if(tailStudy)$('output-details').open=true;
  for(const [selector,key,value] of [['[data-mode]','mode',mode],['[data-motion]','motion',motion],['[data-level]','level',level],['[data-recipe]','recipe',recipe]])document.querySelectorAll(selector).forEach(button=>{const selected=String(button.dataset[key])===String(value);button.classList.toggle('selected',selected);button.setAttribute('aria-pressed',String(selected));});
  updatePreview();
}
function selectTailTest(value){
  if(busy||held||current.active||(current.phase==='finished'&&!current.saved)){notice('先完成并保存当前测段，再切换测试配方。');return;}
  mode=3;recipe=value;motion='straight';staticBiasLevel=0;$('baseline').checked=false;
  if(value==='swim')$('surge').value='10';
  applyPreset();refreshButtons();history.replaceState(null,'','#'+value);
  notice(value==='swim'?'已加载摆尾10%＋双桨10%循迹配方，尚未驱动。':'已打开静态尾角对照：尾摆关闭，尚未驱动。');
}
function updatePreview(){
  for(const [target,id] of [['tail-preview','tail'],['bias-preview','bias'],['surge-preview','surge'],['diff-preview','diff']])$(target).textContent=Number($(id).value)+'%';
  document.querySelectorAll('[data-static-bias]').forEach(button=>{const selected=Number(button.dataset.staticBias)===Math.abs(Number($('bias').value));button.classList.toggle('selected',selected);button.setAttribute('aria-pressed',String(selected));});
}
function resetDisplay(){current={active:false,phase:'pre',recent_samples:[],recent_events:[],summary:{},samples:{vision:0,fish:0}};segmentId='';render();}
function refreshButtons(){
  const savePending=current.phase==='finished'&&!current.saved&&!!segmentId;
  for(const kind of ['swim','static']){$('test-'+kind).disabled=busy||held||current.active||savePending;$('test-'+kind).classList.toggle('selected',mode===3&&recipe===kind);}
  $('start').disabled=busy||current.active||savePending||!connected||visionBusy||!!vision.selection_id;
  $('hold').disabled=busy||!current.active||current.readiness?.ready!==true;
  $('hold').classList.toggle('active',held);$('hold').textContent=held?'正在驱动 · 松开立即全停':'2 · 按住动作 / 松开全停';
  $('finish').disabled=busy||(!current.active&&!savePending)||held;
  $('finish').textContent=savePending?'重新保存本段':'3 · 完成本段并保存';
  $('next').disabled=busy||current.active||savePending;
  $('settings').disabled=busy||held||current.phase==='drive';
  renderVisionControls();
}
async function beginSegment(){
  if(busy||current.active)return;
  let configured;try{configured=settings();}catch(error){notice(error.message,true);return;}
  const turn=++generation;busy=true;resetDisplay();refreshButtons();notice('正在清空上一段记录并准备新测段…');
  const controller=new AbortController();startRequest=controller;
  try{const state=await command('start',{settings:configured,stationary_confirmed:true,heading_confirmed:true},{signal:controller.signal});if(turn!==generation)return;takeState(state);notice(current.readiness?.ready?'本段已开始记录。按住动作，松开后会继续记录滑行。':'本段正在记录，动作暂不可用：'+(current.readiness?.reason||'等待控制准备完成')+'。就绪后请手动按住动作。');}
  catch(error){if(turn===generation)notice(error.name==='AbortError'?'开始操作已取消。':error.message,true);}
  finally{if(startRequest===controller)startRequest=null;if(turn===generation){busy=false;refreshButtons();}poll();}
}
async function driveOnce(turn){
  if(!held||turn!==generation||driveRequest)return;
  const controller=new AbortController();driveRequest=controller;
  try{const data=await command('drive',{settings:settings()},{signal:controller.signal});if(held&&turn===generation)takeState(data);}
  catch(error){if(held&&turn===generation){release('drive_error');notice('驱动请求未确认，已请求全停：'+error.message,true);}}
  finally{if(driveRequest===controller)driveRequest=null;}
}
function press(event){
  if(event?.type==='pointerdown'&&event.button!==0)return;
  if(held||$('hold').disabled)return;
  event?.preventDefault();$('hold').focus({preventScroll:true});if(event?.pointerId!==undefined)$('hold').setPointerCapture(event.pointerId);
  held=true;const turn=++generation;refreshButtons();notice('正在请求驱动。松开即全停，之后继续记录滑行与余转。');driveOnce(turn);
}
function stepTailBias(value){
  if(!held||mode!==3||!['static','swim'].includes(recipe))return;
  if(driveRequest){notice('等待当前命令回执后再做下一次尾偏阶跃。');return;}
  if(value&&motion==='straight'){notice('做尾偏阶跃前先选择左转或右转。');return;}
  $('bias').value=String((motion==='left'?-1:1)*value);staticBiasLevel=value;
  updatePreview();notice(`保持共同推进 ${$('surge').value}%，只将尾偏置改为 ${$('bias').value}%；松开动作仍全停。`);
  driveOnce(generation);
}
function release(reason='release',force=false){
  const pending=held||!!driveRequest||!!startRequest;
  if(!pending&&!force)return Promise.resolve();
  const turn=++generation;held=false;busy=false;driveRequest?.abort();driveRequest=null;startRequest?.abort();startRequest=null;refreshButtons();
  notice('已请求全停，继续记录滑行与余转。等鱼静止后完成本段。');
  return command(force?'stop':'release',{reason},{keepalive:true}).then(data=>{if(turn===generation)takeState(data);}).catch(error=>{if(turn===generation)notice('全停尚未确认：'+error.message+'。请检查连接并再次点击全停。',true);}).finally(()=>poll());
}
async function finishSegment(){
  if(busy||(!current.active&&!(current.phase==='finished'&&!current.saved&&segmentId))||held)return;
  busy=true;refreshButtons();notice('正在停止输出、计算本段结果并保存到桌面…');
  try{takeState(await command('finish'));const stopUnconfirmed=current.stop_confirmed===false;notice(current.saved?(stopUnconfirmed?'本段数据已保存到桌面，但全停尚未确认。请点击全停重试。':'本段已自动保存到桌面。可以查看结果，或加载下一组参数。'):'保存状态尚未确认，请查看提示后重试。',!current.saved||stopUnconfirmed);}
  catch(error){notice('保存未完成：'+error.message+'。本段记录保留，请重试。',true);}
  finally{busy=false;refreshButtons();poll();}
}
function nextGroup(){
  if(current.active||busy)return;
  if(mode===3&&['static','swim'].includes(recipe)){
    if(motion==='straight'){motion='left';if(!staticBiasLevel)staticBiasLevel=40;}
    else if(motion==='left')motion='right';
    else if(staticBiasLevel<100){staticBiasLevel=[40,60,80,100].find(value=>value>staticBiasLevel);motion='left';}
    else{staticBiasLevel=0;motion='straight';}
  }else if(motion==='straight')motion='left';else if(motion==='left')motion='right';else{motion='straight';if(level<40)level+=10;else{level=20;mode=mode===3?1:mode+1;}}
  $('baseline').checked=false;applyPreset();refreshButtons();notice('已加载下一组参数，未开始记录或驱动。鱼静止、鱼头箭头正确后，点击开始本段。');
}
function lastValue(samples,key){for(let i=samples.length-1;i>=0;i--)if(finite(samples[i][key]))return samples[i][key];return null;}
function render(){
  const samples=current.recent_samples||[],summary=current.summary||{},counts=current.samples||{},ready=current.readiness;
  $('phase').textContent=phaseNames[current.phase]||'未开始';$('phase').className='badge'+(current.active?' good':'');
  $('connection').textContent=ready?.ready?'Wi-Fi / 视觉已就绪':ready?.reason||(connected?'本地服务已连接':'本地服务未连接');$('connection').className='badge'+(ready?.ready?' good':connected?'':' bad');
  $('elapsed').textContent=format(current.elapsed_s||0,'s',1);$('segment').textContent=current.segment_id||'等待开始';
  $('speed').textContent=format(lastValue(samples,'speed_1s_mps'),'m/s',3);$('yaw-rate').textContent=format(lastValue(samples,'yaw_rate_dps'),'°/s',1);
  $('counts').textContent=((counts.vision||0)+(counts.fish||0))+' / '+(current.events||0);$('sample-details').textContent='视觉 '+(counts.vision||0)+' · 鱼端 '+(counts.fish||0);
  const events=current.recent_events||[];const ack=events.find(event=>/drive.*(ack|success)|drive_applied/.test(event.kind||''));
  const ackTime=summary.drive_ack_time_s??ack?.time_s;$('write-status').textContent='命令回执：'+(finite(ackTime)?format(ackTime,'s',2)+' 确认':'尚无确认');
  $('motion-status').textContent='实际起动：'+(finite(summary.motion_start_time_s)?format(summary.motion_start_time_s,'s',2)+'（位移 / 转角阈值估计）':'等待完成测段后估计');
  $('results').replaceChildren(...summaryFields.map(([key,label,unit,digits])=>{const item=document.createElement('div');item.className='result';const name=document.createElement('small');name.textContent=label;const value=document.createElement('strong');value.textContent=key==='turn_radius_m'&&!finite(summary[key])?'数据不足':format(summary[key],unit,digits);item.append(name,value);return item;}));
  $('result-state').textContent=current.saved?'本段结果已保存':current.active?'测量中 · 完成后计算':'完成本段后计算';$('saved').hidden=!current.saved;$('saved-path').textContent=current.path||'';
  $('fit-note').textContent=summary.fit_reason||(summary.fit_quality?'拟合质量：'+summary.fit_quality:'短弧不足以可靠拟合时，半径显示“数据不足”；速度、转角、位移和原始数据仍会保存。');
  const labels={start:'开始本段',segment_start:'开始本段',segment_started:'开始本段',drive:'请求驱动',drive_request:'请求驱动',drive_requested:'请求驱动',drive_phase:'进入驱动记录阶段',drive_ack:'命令回执确认',drive_error:'驱动请求失败',release:'松开动作',release_phase:'松开动作，记录滑行与余转',release_requested:'请求全停',release_ack:'停止命令回执确认',stop:'全停',stop_request:'请求全停',stop_ack:'停止命令回执确认',stop_error:'全停未确认',finish:'完成测段',segment_finish_requested:'请求完成并保存本段',finish_control_status:'完成测段时的控制状态已记录',shutdown_control_status:'关闭页面时的控制状态已记录',settings:'参数变更',motion_start:'实际起动（估计）',motion_start_estimated:'实际起动（视觉估计）',output_zero_observed:'鱼端遥测确认输出归零',source_error:'采样源出现错误',source_gap:'采样出现断流',source_restored:'采样已恢复'};
  $('event-list').replaceChildren(...(events.length?events.slice(-30).map(event=>{const li=document.createElement('li');li.textContent=format(event.time_s,'s',2)+' · '+(labels[event.kind]||'测量事件已记录');return li;}):[Object.assign(document.createElement('li'),{textContent:'尚无事件'})]));
  if(current.control_error)notice(current.control_error,true);
  else if(current.error)notice(current.error,true);
  else if(current.active&&current.phase==='pre'&&!busy&&!held)notice(ready?.ready?'本段已开始记录。现在可以按住动作；松开后继续记录滑行。':'本段正在记录，动作暂不可用：'+(ready?.reason||'等待控制准备完成')+'。就绪后请手动按住动作。');
  refreshButtons();drawTrajectory(samples,events);drawSeries('kinematics',samples,[{key:'speed_1s_mps',color:'#087c70',axis:0},{key:'yaw_rate_dps',color:'#b46c25',axis:1}]);drawSeries('actuators',samples,[{key:'left_pct',color:'#3277b5'},{key:'right_pct',color:'#a754a4'},{key:'tail1_deg',color:'#b46c25'},{key:'tail2_deg',color:'#087c70'}]);
}
function canvasFrame(id){
  const canvas=$(id),width=Math.max(100,canvas.clientWidth),height=Number(canvas.dataset.plotHeight||canvas.getAttribute('height')),ratio=Math.min(2,window.devicePixelRatio||1);
  canvas.dataset.plotHeight=String(height);canvas.style.height=height+'px';
  if(canvas.width!==Math.round(width*ratio)||canvas.height!==Math.round(height*ratio)){canvas.width=Math.round(width*ratio);canvas.height=Math.round(height*ratio);}
  const ctx=canvas.getContext('2d');ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,width,height);ctx.font='10px Segoe UI, Microsoft YaHei';ctx.lineWidth=1;return {ctx,width,height};
}
function emptyPlot(ctx,width,height,message){ctx.fillStyle='#84959f';ctx.textAlign='center';ctx.fillText(message,width/2,height/2);ctx.textAlign='left';}
function drawTrajectory(samples,events){
  const {ctx,width,height}=canvasFrame('trajectory'),points=samples.filter(row=>finite(row.x_m)&&finite(row.y_m));
  const poolWidth=Number(current.settings?.pool_width_m)||3,poolHeight=Number(current.settings?.pool_height_m)||1.6;
  const pad=30,scale=Math.min((width-pad*2)/poolWidth,(height-pad*2)/poolHeight),left=(width-poolWidth*scale)/2,top=(height-poolHeight*scale)/2;
  const x=value=>left+value*scale,y=value=>top+value*scale;
  ctx.strokeStyle='#e1e9ed';ctx.fillStyle='#899aa4';for(let value=0;value<=poolWidth+.001;value+=.5){ctx.beginPath();ctx.moveTo(x(value),top);ctx.lineTo(x(value),y(poolHeight));ctx.stroke();ctx.fillText(value.toFixed(1),x(value)-6,y(poolHeight)+15);}for(let value=0;value<=poolHeight+.001;value+=.5){ctx.beginPath();ctx.moveTo(left,y(value));ctx.lineTo(x(poolWidth),y(value));ctx.stroke();ctx.fillText(value.toFixed(1),left-25,y(value)+3);}ctx.fillText('m · x → / y ↓',left,top-9);ctx.strokeStyle='#b9cbd2';ctx.strokeRect(left,top,poolWidth*scale,poolHeight*scale);
  if(!points.length){emptyPlot(ctx,width,height,'等待米制视觉位置');return;}
  ctx.save();ctx.beginPath();ctx.rect(left,top,poolWidth*scale,poolHeight*scale);ctx.clip();
  for(let i=1;i<points.length;i++){const a=points[i-1],b=points[i];if(b.time_s-a.time_s>1)continue;ctx.strokeStyle=phaseColors[b.phase]||'#087c70';ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(x(a.x_m),y(a.y_m));ctx.lineTo(x(b.x_m),y(b.y_m));ctx.stroke();}
  const stride=Math.max(1,Math.ceil(points.length/16));for(let i=0;i<points.length;i+=stride){const p=points[i];if(!finite(p.yaw_deg))continue;const angle=p.yaw_deg*Math.PI/180,px=x(p.x_m),py=y(p.y_m),dx=Math.cos(angle),dy=-Math.sin(angle);ctx.strokeStyle=phaseColors[p.phase]||'#087c70';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(px,py);ctx.lineTo(px+dx*12,py+dy*12);ctx.lineTo(px+dx*8-dy*3,py+dy*8+dx*3);ctx.moveTo(px+dx*12,py+dy*12);ctx.lineTo(px+dx*8+dy*3,py+dy*8-dx*3);ctx.stroke();}
  const marks=events.filter(event=>/drive_ack|release|stop|finish/.test(event.kind||''));for(const event of marks){const point=points.reduce((best,p)=>Math.abs(p.time_s-event.time_s)<Math.abs(best.time_s-event.time_s)?p:best,points[0]);if(Math.abs(point.time_s-event.time_s)>1)continue;ctx.fillStyle=/drive/.test(event.kind)?'#087c70':'#b46c25';ctx.beginPath();ctx.arc(x(point.x_m),y(point.y_m),4,0,Math.PI*2);ctx.fill();}
  const last=points.at(-1);ctx.fillStyle='#183b48';ctx.beginPath();ctx.arc(x(last.x_m),y(last.y_m),4,0,Math.PI*2);ctx.fill();ctx.restore();
}
function drawSeries(id,samples,series){
  const {ctx,width,height}=canvasFrame(id),valid=samples.filter(row=>finite(row.time_s));
  if(!valid.length){emptyPlot(ctx,width,height,'等待连续采样');return;}
  const end=Math.max(...valid.map(row=>row.time_s)),start=Math.max(0,end-60),rows=valid.filter(row=>row.time_s>=start),span=Math.max(5,end-start),left=35,right=series.some(item=>item.axis===1)?39:12,top=15,bottom=25,plotWidth=width-left-right,plotHeight=height-top-bottom;
  const ranges=[0,1].map(axis=>{const fields=series.filter(item=>(item.axis||0)===axis).map(item=>item.key),values=rows.flatMap(row=>fields.map(key=>row[key]).filter(finite));const min=Math.min(0,...values),max=Math.max(axis===0&&id==='kinematics'?.1:1,...values);return {min,max};});
  const x=time=>left+(time-start)/span*plotWidth,y=(value,axis)=>top+(ranges[axis].max-value)/(ranges[axis].max-ranges[axis].min)*plotHeight;
  ctx.fillStyle='#84959f';ctx.strokeStyle='#e1e9ed';for(let step=0;step<=4;step++){const py=top+plotHeight*step/4;ctx.beginPath();ctx.moveTo(left,py);ctx.lineTo(width-right,py);ctx.stroke();const label=ranges[0].max-(ranges[0].max-ranges[0].min)*step/4;ctx.textAlign='right';ctx.fillText(label.toFixed(id==='kinematics'?2:0),left-5,py+3);if(right>12){ctx.textAlign='left';ctx.fillText((ranges[1].max-(ranges[1].max-ranges[1].min)*step/4).toFixed(0),width-right+5,py+3);}}
  ctx.textAlign='left';ctx.fillText(start.toFixed(1)+' s',left,height-6);ctx.textAlign='right';ctx.fillText((start+span).toFixed(1)+' s',width-right,height-6);ctx.textAlign='left';
  for(const line of series){ctx.strokeStyle=line.color;ctx.lineWidth=1.5;ctx.beginPath();let prior=null;for(const row of rows){if(!finite(row[line.key]))continue;const px=x(row.time_s),py=y(row[line.key],line.axis||0);if(prior===null||row.time_s-prior>1)ctx.moveTo(px,py);else ctx.lineTo(px,py);prior=row.time_s;}ctx.stroke();}
}
function cameraNotice(message,error=false){$('camera-notice').textContent=message;$('camera-notice').classList.toggle('error',error);}
function renderVisionControls(){
  const locked=visionBusy||busy||current.active;
  $('camera-start').disabled=locked||(vision.running&&vision.preview_enabled!==false);
  $('camera-start').textContent=vision.running&&vision.preview_enabled===false?'显示画面':'启动画面';
  $('camera-stop').disabled=locked||!vision.running;
  for(const id of ['camera-head','camera-fish','camera-pool'])$(id).disabled=locked||!vision.running||!vision.ready;
  const interaction=vision.result?.interaction;
  $('camera-head').textContent=interaction==='HEAD_DIRECTION'?'取消鱼头选择':'鱼头方向';
  $('camera-fish').textContent=interaction==='MARKER_ROI'?'取消框选':'框选鱼';
  $('camera-pool').textContent=interaction==='POOL_CALIB'?'取消四角选择':'重新选择四角';
  $('camera-image').classList.toggle('selecting',!!vision.selection_id);
}
function updateCameraImage(){
  const image=$('camera-image');
  if(document.hidden||!vision.running||!vision.ready||!vision.result_ready||vision.preview_enabled===false){image.removeAttribute('src');delete image.dataset.stream;image.dataset.pointerReady='false';image.hidden=true;$('camera-empty').hidden=false;$('camera-empty').textContent=vision.running?(vision.preview_enabled===false?'画面已隐藏，点击“显示画面”恢复。':'正在等待视觉画面…'):'点击“启动画面”使用摄像头。已有视觉会自动显示。';return;}
  const frozen=!!vision.selection_id,url=`/api/vision/${frozen?'frame':'stream'}?kind=result&session=${vision.process_id}&space=${vision.display_space||'image'}&selection=${vision.selection_id||''}${frozen?'&revision='+vision.preview_revision:''}`;
  if(image.dataset.stream!==url){image.dataset.pointerReady='false';image.dataset.stream=url;image.src=url;}
  image.hidden=false;$('camera-empty').hidden=true;
}
async function pollVision(){
  if(visionPolling||document.hidden)return;visionPolling=true;
  try{
    vision=await request('vision/state');renderVisionControls();updateCameraImage();
    $('camera-state').textContent=vision.running?(vision.ready?'实时画面':'相机初始化中'):'画面未启动';
    $('camera-state').className='badge'+(vision.ready?' good':'');
    const initial=vision.result?.initial_heading||{},yaw=vision.result?.body_yaw||{},optical=vision.result?.optical||{};
    $('camera-yaw').textContent='当前朝向 '+format(yaw.usable?yaw.yaw_deg:optical.valid?optical.relative_deg:null,'°',1)+' · 鱼头标定为 0°';
    if(vision.running&&/^\d+$/.test(String(vision.source))&&initial.calibrated&&initial.imu_aligned&&initial.calibration_id&&initial.calibration_id!==headingSynced){
      headingSynced=initial.calibration_id;headingSyncError='';
      try{await request('station/heading-reference',{calibration_id:initial.calibration_id});}
      catch(error){headingSyncError='鱼头视觉零位已建立，但 IMU 零位同步失败：'+error.message+'。请重新点击“鱼头方向”标定。';}
    }
    if(headingSyncError)cameraNotice(headingSyncError,true);
    else if(vision.last_task?.state==='error')cameraNotice(vision.last_task.status||'视觉操作失败',true);
    else if(vision.result?.interaction)cameraNotice(vision.result.status||'请在画面上完成选择。');
    else if(vision.running)cameraNotice(initial.calibrated?'鱼头标定已建立当前 0°；箭头正确、鱼静止后即可开始测量。':'鱼头箭头不正确时，点击“鱼头方向”并在画面中点鱼头，自动建立当前 0°。');
    refreshButtons();
  }catch(error){cameraNotice(error.message,true);}
  finally{visionPolling=false;}
}
async function startCamera(){
  if(visionBusy||current.active)return;visionBusy=true;refreshButtons();cameraNotice('正在打开摄像头画面…');
  try{
    vision=await request('vision/state');
    if(vision.running){if(vision.preview_enabled===false)await request('vision/action',{event:{task:'PREVIEW'}});}
    else{
      const config=await request('vision/config'),available=await request('vision/cameras');
      const cameras=available.cameras||[];if(!cameras.length)throw Error('未找到可用摄像头，请检查连接后重试“启动画面”。');
      let preferred='';try{preferred=localStorage.getItem('robofish-vision-camera')||'';}catch(_){}
      const camera=cameras.find(item=>String(item.index)===preferred)||cameras[0],model=config.default_model||config.models?.[0]?.path||'';
      if(config.model_required!==false&&!model)throw Error('未找到鱼体识别模型，请在主界面配置模型。');
      vision=await request('vision/start',{source:Number(camera.index),model});
    }
    await pollVision();
  }catch(error){cameraNotice(error.message,true);}
  finally{visionBusy=false;refreshButtons();}
}
async function stopCamera(){
  if(visionBusy||current.active)return;visionBusy=true;refreshButtons();
  try{await visionPointerQueue;vision=await request('vision/stop',{});headingSynced='';headingSyncError='';updateCameraImage();cameraNotice('画面已停止。');}
  catch(error){cameraNotice(error.message,true);}
  finally{visionBusy=false;refreshButtons();pollVision();}
}
async function cameraTask(task){
  if(visionBusy||current.active||!vision.ready)return;visionBusy=true;refreshButtons();
  try{
    await visionPointerQueue;
    if(vision.result?.tracking?.automatic)await request('vision/action',{event:{task:'AUTO_STOP'}});
    const event=task==='POOL_SIZE'?(vision.result?.interaction==='POOL_CALIB'?{kind:'right',x:0,y:0}:{task,width_m:3,height_m:1.6}):{task};
    await request('vision/action',{event});await pollVision();
  }catch(error){cameraNotice(error.message,true);}
  finally{visionBusy=false;refreshButtons();}
}
function cameraPoint(event,clip){
  const image=$('camera-image'),box=image.getBoundingClientRect(),ratio=Math.min(box.width/image.naturalWidth,box.height/image.naturalHeight),width=image.naturalWidth*ratio,height=image.naturalHeight*ratio;
  const x=(event.clientX-box.left-(box.width-width)/2)/width,y=(event.clientY-box.top-(box.height-height)/2)/height;
  if(!Number.isFinite(x)||!Number.isFinite(y))return null;
  return clip?{x:Math.max(0,Math.min(1,x)),y:Math.max(0,Math.min(1,y))}:x>=0&&x<=1&&y>=0&&y<=1?{x,y}:null;
}
function sendCameraPointer(event,kind){
  const image=$('camera-image'),selection=vision.selection_id||'';
  if(current.active||!selection)return false;
  const dragging=visionPointerDown&&visionPointerSelection?.selection_id===selection;
  if(!dragging&&(image.dataset.pointerReady!=='true'||image.dataset.selectionId!==selection)){if(kind==='down')cameraNotice('标定画面正在加载，图像显示后再选择。');return false;}
  const point=cameraPoint(event,dragging||kind==='right');if(!point)return false;
  const context=dragging?visionPointerSelection:{selection_id:selection,space:image.dataset.pointerSpace||'image'};
  if(kind==='down')visionPointerSelection=context;
  const input={kind,...point,...context};
  visionPointerQueue=visionPointerQueue.then(()=>request('vision/action',{event:input})).then(()=>{if(kind==='up'||kind==='right')pollVision();}).catch(error=>cameraNotice(error.message,true));
  return true;
}
async function poll(){
  if(polling||document.hidden)return;polling=true;const turn=generation;
  try{const data=await request('dynamics/state');connected=true;if(turn===generation)takeState(data);}
  catch(error){connected=false;$('connection').textContent=error.message;$('connection').className='badge bad';if(held)release('connection_lost');refreshButtons();}
  finally{polling=false;}
}
document.querySelectorAll('[data-mode]').forEach(button=>button.onclick=()=>{mode=Number(button.dataset.mode);if(mode!==3)$('baseline').checked=false;applyPreset();});
document.querySelectorAll('[data-motion]').forEach(button=>button.onclick=()=>{motion=button.dataset.motion;applyPreset();});
document.querySelectorAll('[data-level]').forEach(button=>button.onclick=()=>{level=Number(button.dataset.level);applyPreset();});
document.querySelectorAll('[data-recipe]').forEach(button=>button.onclick=()=>{if($('settings').disabled)return;recipe=button.dataset.recipe;if(recipe==='static')mode=3;applyPreset();});
document.querySelectorAll('[data-static-bias]').forEach(button=>button.onclick=()=>{if($('settings').disabled||mode!==3||!['static','swim'].includes(recipe))return;if(motion==='straight'&&Number(button.dataset.staticBias)>0){notice('先选择左转或右转，再加载尾偏置；共同推进保持不变。');return;}staticBiasLevel=Number(button.dataset.staticBias);applyPreset();});
for(const id of ['tail','bias','surge','diff'])$(id).addEventListener('input',()=>{if(id==='surge'&&mode===3&&recipe==='swim')$('tail').value=$('surge').value;if(id==='bias'&&mode===3&&['static','swim'].includes(recipe)&&Number.isFinite(Number($('bias').value)))staticBiasLevel=Math.min(100,Math.abs(Number($('bias').value)));updatePreview();});
$('test-swim').onclick=()=>selectTailTest('swim');$('test-static').onclick=()=>selectTailTest('static');
$('baseline').onchange=applyPreset;
$('camera-start').onclick=startCamera;$('camera-stop').onclick=stopCamera;$('camera-head').onclick=()=>cameraTask('HEAD_DIRECTION');$('camera-fish').onclick=()=>cameraTask('MARKER_ROI');$('camera-pool').onclick=()=>cameraTask('POOL_SIZE');
$('camera-image').onload=()=>{const image=$('camera-image');if(!image.dataset.stream)return;const url=new URL(image.dataset.stream,location.href);image.dataset.pointerSpace=url.searchParams.get('space')||'image';image.dataset.selectionId=url.searchParams.get('selection')||'';image.dataset.pointerReady='true';};
$('camera-image').onerror=()=>{const image=$('camera-image');image.dataset.pointerReady='false';delete image.dataset.stream;cameraNotice('画面暂未加载，正在重试。');};
$('camera-image').onpointerdown=event=>{event.preventDefault();if(event.button===2){sendCameraPointer(event,'right');return;}if(event.button===0&&sendCameraPointer(event,'down')){event.target.setPointerCapture(event.pointerId);visionPointerDown=true;}};
$('camera-image').onpointermove=event=>{if(visionPointerDown&&Date.now()-visionPointerAt>40){visionPointerAt=Date.now();sendCameraPointer(event,'move');}};
$('camera-image').onpointerup=event=>{if(visionPointerDown)sendCameraPointer(event,'up');visionPointerDown=false;visionPointerSelection=null;};
$('camera-image').onpointercancel=$('camera-image').onlostpointercapture=event=>{if(visionPointerDown)sendCameraPointer(event,'right');visionPointerDown=false;visionPointerSelection=null;};$('camera-image').oncontextmenu=event=>event.preventDefault();
$('start').onclick=beginSegment;$('finish').onclick=finishSegment;$('next').onclick=nextGroup;$('stop').onclick=()=>release('emergency_stop',true);
$('hold').addEventListener('pointerdown',press);$('hold').addEventListener('contextmenu',event=>event.preventDefault());
window.addEventListener('pointerup',()=>release('pointerup'));window.addEventListener('pointercancel',()=>release('pointercancel'));$('hold').addEventListener('lostpointercapture',()=>release('capture_lost'));
$('hold').addEventListener('keydown',event=>{if(event.code==='Enter'&&!event.repeat)press(event);});$('hold').addEventListener('keyup',event=>{if(event.code==='Enter')release('key_release');});
window.addEventListener('keydown',event=>{
  if(held&&mode===3&&['static','swim'].includes(recipe)&&['KeyZ','KeyX'].includes(event.code)&&!/INPUT|TEXTAREA|SELECT/.test(event.target.tagName)){
    event.preventDefault();if(!event.repeat)stepTailBias(event.code==='KeyZ'?0:60);return;
  }
  if(event.code==='Space'){event.preventDefault();if(!event.repeat)release('space_stop',true);}
  if(event.code==='Escape')release('escape_stop',true);
});
window.addEventListener('blur',()=>release('window_blur'));
document.addEventListener('visibilitychange',()=>{if(document.hidden)release('page_hidden');else{poll();pollVision();}updateCameraImage();});
window.addEventListener('pagehide',()=>release('page_exit',held||!!startRequest));
window.addEventListener('resize',()=>render());
if(['#static','#swim'].includes(location.hash))selectTailTest(location.hash.slice(1));else applyPreset();
render();poll();pollVision();setInterval(()=>{poll();pollVision();},500);
