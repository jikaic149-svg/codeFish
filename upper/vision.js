'use strict';
let visionState={running:false,ready:false},visionPolling=false,visionStarting=false,visionConfiguration=null;
let controlVideoActive=false,pointerDown=false,lastPointerAt=0,boardCameraActive=false,lastCalibrationTask='';
let syncedHeadingCalibration='';
function visionSaved(key,fallback){try{return localStorage.getItem('robofish-vision-'+key)||fallback;}catch(_){return fallback;}}
function saveVision(key,value){try{localStorage.setItem('robofish-vision-'+key,value);}catch(_){}}
let visionLayout=visionSaved('layout','control');
function clearVisionImage(image,empty){$(image).removeAttribute('src');delete $(image).dataset.stream;$(image).hidden=true;$(empty).hidden=false;}
function visionNotice(message){robotText('vision-task-state',message);log(message,'info');}
function boardHost(){return robotAddress($('vision-board-host').value)||(!$('vision-board-host').value.trim()?robotHost():'');}
function boardStreamURL(host){if(isRobotFish())throw Error('此 RobotFish 未提供相机');const [address,port]=host.split(':');if(Number(port||80)>=65535)throw Error('相机HTTP端口需小于65535');return `http://${address}:${Number(port||80)+1}/stream`;}
function applyVisionLayout(value){
  visionLayout=['board','control','dual'].includes(value)?value:'control';saveVision('layout',visionLayout);
  $('vision-views').dataset.layout=visionLayout;
  $('vision-board-card').hidden=visionLayout==='control';$('vision-control-card').hidden=visionLayout==='board';
  document.querySelectorAll('[data-vision-layout]').forEach(button=>{const chosen=button.dataset.visionLayout===visionLayout;button.classList.toggle('selected',chosen);button.setAttribute('aria-pressed',String(chosen));});
  renderVisionControls();updateVisionImages();
}
function renderVisionControls(){
  if(isRobotFish()&&visionLayout!=='control'){applyVisionLayout('control');return;}
  $('vision-start').hidden=visionLayout==='board';$('vision-stop').hidden=visionLayout==='board'&&!visionState.running;
  $('vision-start').disabled=visionStarting||visionState.running||(visionConfiguration?.model_required!==false&&!$('vision-model').value);
  $('vision-stop').disabled=!visionState.running||visionStarting;
  for(const id of ['vision-camera-index','vision-source','vision-model'])$(id).disabled=visionState.running||visionStarting;
  if(visionConfiguration?.model_required===false)$('vision-model').disabled=true;
  document.querySelectorAll('[data-vision-task]').forEach(button=>{
    button.hidden=visionConfiguration?.supported_tasks?!visionConfiguration.supported_tasks.includes(button.dataset.visionTask):button.hidden;
    button.disabled=!visionState.running||!visionState.ready;
    if(button.dataset.visionTask==='SAMPLE_START')button.disabled||=visionState.samples?.active===true||visionState.result?.eof===true;
    if(button.dataset.visionTask==='SAMPLE_STOP')button.disabled||=!visionState.samples?.active;
    if(button.dataset.visionTask==='PREVIEW'){button.textContent=visionState.preview_enabled===false?'打开原图预览':'关闭原图预览';button.setAttribute('aria-pressed',String(visionState.preview_enabled!==false));}
  });
  for(const id of ['vision-speed-apply','vision-pool-apply'])$(id).disabled=!visionState.running||!visionState.ready;
  const samples=visionState.samples||{};
  robotText('vision-recording-state',samples.error?'采样未完整保存：'+samples.error:samples.active?`正在采样 · ${samples.rows||0} 行 · 独立视觉角度、轨迹、IMU 与输出指令`:samples.saved?`已保存 ${samples.rows||0} 行 · ${samples.file}`:'未采样；停止采样后可下载 CSV 和标定元数据。');
  $('vision-sample-files').hidden=!samples.saved||samples.active===true;
  $('vision-board-toggle').textContent=boardCameraActive?'关闭板载相机':'打开板载相机';
}
function syncVisionParameters(){
  if(!visionState.running)return;
  if(/^\d+$/.test(visionState.source)){$('vision-source').value='camera';$('vision-camera-index').value=visionState.source;}
  else $('vision-source').value='sample';
  const actual=String(visionState.model||'').replaceAll('\\','/').toLowerCase();
  const model=[...$('vision-model').options].find(option=>actual===option.value.toLowerCase()||actual.endsWith('/'+option.value.toLowerCase()));
  if(model)$('vision-model').value=model.value;
}
async function loadVisionConfiguration(){
  try{
    visionConfiguration=await service('vision/config');
    document.querySelector('label[for="vision-model"]').textContent=visionConfiguration.model_required===false?'视觉算法':'鱼体识别模型（.pt）';
    document.querySelector('label[for="vision-source"]').textContent='视觉观测输入';
    document.querySelector('.vision-input-preview').hidden=visionConfiguration.observation_only===true;
    $('vision-calibration-turn').closest('article').hidden=visionConfiguration.observation_only===true;
    $('vision-speed-target').closest('label').hidden=visionConfiguration.observation_only===true;
    document.querySelector('#vision-settings>summary').textContent=visionConfiguration.model_required===false?'相机与标定配置':'相机、模型与标定配置';
    const select=$('vision-model'),previous=select.value||visionConfiguration.default_model||visionSaved('model','');
    select.replaceChildren();
    if(visionConfiguration.model_required===false)select.add(new Option(visionConfiguration.algorithm||'OpenCV（无需模型）',''));
    for(const model of visionConfiguration.models){const option=new Option(`${model.name} · ${(model.bytes/1024/1024).toFixed(1)} MiB`,model.path);select.add(option);}
    select.value=visionConfiguration.models.some(item=>item.path===previous)?previous:visionConfiguration.default_model;
    for(const profile of visionConfiguration.calibration){
      const value=profile.values||{};
      let text=profile.error?'文件无效：'+profile.error:profile.saved?'已保存标定':'尚未标定';
      if(profile.key==='head')text='标定场地后点击鱼头，建立独立视觉 0°；实时相机将同步 IMU 控制零位，每次会话重新标定。';
      if(profile.key==='tail')text='使用 YOLO 自动识别鱼体；需要时框选完整机器鱼。';
      if(profile.saved&&!profile.error){
        if(profile.key==='pool')text+=` · ${value.physical_width_m??'—'} × ${value.physical_height_m??'—'} m · ${value.frame_width??'—'}×${value.frame_height??'—'}`;
        else if(Number.isFinite(value.hue_center))text+=` · 色相 ${value.hue_center.toFixed(1)} ± ${Number(value.hue_tolerance||0).toFixed(1)}`;
      }
      robotText('vision-calibration-'+profile.key,text);
    }
    robotText('vision-model-state',visionState.running?'正在使用：'+(visionState.model||visionConfiguration.algorithm):'使用最新鱼体模型；视觉只观测和采样，目标角度从操控页发送，由 S3 本地闭环。');
    syncVisionParameters();renderVisionControls();
  }catch(error){robotText('vision-model-state',error.message);}
}
async function startVision(){
  if(visionStarting)return;visionStarting=true;renderVisionControls();robotText('vision-service-state','正在启动视觉观测…');
  try{
    if(!$('vision-model').value)await loadVisionConfiguration();
    const source=$('vision-source').value==='sample'?'sample':Number($('vision-camera-index').value);
    visionState=await service('vision/start',{source,model:$('vision-model').value});
    saveVision('source',$('vision-source').value);saveVision('camera',$('vision-camera-index').value);saveVision('model',$('vision-model').value);
    await pollVision();
  }catch(error){visionNotice(error.message);toast(error.message);}
  finally{visionStarting=false;renderVisionControls();}
}
async function visionTask(task){
  if(['MARKER_ROI','HEAD_DIRECTION','POOL_CALIB','TURN_CALIB'].includes(task)&&visionLayout==='board')applyVisionLayout('control');
  try{const result=await service('vision/action',{event:{task}});visionNotice(`视觉操作已排队：${task} · ${result.id.slice(0,8)}`);await pollVision();}
  catch(error){visionNotice(error.message);toast(error.message);renderVisionControls();}
}
async function beginVisionSamples(){
  visionState=await service('vision/state');
  if(!visionState.running)return;
  if(!visionState.ready)throw Error('视觉尚未准备好，请稍后发送或取消自动采样');
  if(visionState.samples?.active)return;
  const task=await service('vision/action',{event:{task:'SAMPLE_START'}});
  for(let attempt=0;attempt<30;attempt++){
    visionState=await service('vision/state');
    if(visionState.samples?.active)return;
    if(visionState.last_task?.id===task.id&&visionState.last_task.state==='error')throw Error(visionState.last_task.status);
    await new Promise(resolve=>setTimeout(resolve,100));
  }
  throw Error('视觉尚未确认开始采样，请检查视觉状态');
}
function updateBoardImage(){
  if(!boardCameraActive||state.page!=='vision'||visionLayout==='control'){clearVisionImage('vision-board','vision-board-empty');return;}
  const image=$('vision-board');if(Number(image.dataset.retryAfter||0)>Date.now())return;
  const host=boardHost();if(!host){robotText('vision-board-state','请在相机配置中填写有效板载相机地址');return;}
  try{
    const url=boardStreamURL(host);
    if(image.dataset.stream!==url){image.dataset.stream=url;image.src=url;robotText('vision-board-state','正在连接板载相机 · '+host);}
    if(image.naturalWidth){image.hidden=false;$('vision-board-empty').hidden=true;robotText('vision-board-state',`板载相机 · ${host} · ${image.naturalWidth}×${image.naturalHeight}`);}
  }catch(error){robotText('vision-board-state',error.message);}
}
function updateVisionImages(){
  updateBoardImage();
  $('vision-result').closest('.camera-stage').classList.toggle('preview-paused',visionState.preview_enabled===false);
  if(!visionState.running||!visionState.ready||visionState.preview_enabled===false){
    for(const [image,empty] of [['vision-raw','vision-raw-empty'],['vision-result','vision-result-empty']])clearVisionImage(image,empty);
    if($('control-video-source').value==='vision')clearVisionImage('control-camera','control-camera-empty');
    robotText('vision-result-empty',visionState.running&&visionState.preview_enabled===false?'预览已关闭，后台继续识别；采样状态见下方，标定时自动打开。':'启动视觉后显示原图预览');
    return;
  }
  function show(image,kind,empty){
    if(kind==='result'&&!visionState.result_ready){clearVisionImage(image,empty);return;}
    if(Number($(image).dataset.retryAfter||0)>Date.now())return;
    const url=`/api/vision/stream?kind=${kind}&session=${visionState.process_id}`;
    if($(image).dataset.stream!==url){$(image).src=url;$(image).dataset.stream=url;}
    $(image).hidden=false;$(empty).hidden=true;
  }
  if(state.page==='vision'&&visionLayout!=='board'){
    show('vision-result','result','vision-result-empty');
    if(document.querySelector('.vision-input-preview').open)show('vision-raw','raw','vision-raw-empty');else clearVisionImage('vision-raw','vision-raw-empty');
  }else for(const [image,empty] of [['vision-raw','vision-raw-empty'],['vision-result','vision-result-empty']])clearVisionImage(image,empty);
  if(state.page==='overview'&&controlVideoActive&&$('control-video-source').value==='vision')show('control-camera','raw','control-camera-empty');
}
async function pollVision(){
  if(visionPolling)return;visionPolling=true;
  try{
    visionState=await service('vision/state');
    robotText('vision-service-state',!visionState.running?'视觉观测未运行':visionState.result?.eof?'录像回放结束':visionState.ready?`视觉观测运行中 · 图像 ${visionState.frame_age_ms??'—'} ms前${visionState.preview_enabled===false?' · 后台识别':''}`:'相机或模型初始化中');
    robotText('vision-config-path',visionState.source?'当前视觉输入：'+visionState.source+(visionState.width&&visionState.height?` · ${visionState.width}×${visionState.height}`:''):'外部视觉使用所选摄像头或录像回放；板载相机单独打开。');
    if(visionState.running)robotText('vision-model-state','正在使用：'+(visionState.model||visionConfiguration?.algorithm||'OpenCV'));
    if(visionState.last_task?.state==='processed'){
      robotText('vision-task-state',`已处理：${visionState.last_task.task} · ${visionState.last_task.status}${visionState.last_task.task==='START'&&!visionState.last_task.tracking?'（循迹未启动，请检查标定、目标和控制链路）':''}`);
      if(visionState.last_task.id!==lastCalibrationTask&&['MARKER_ROI','HEAD_DIRECTION','POOL_CALIB','TURN_CALIB'].includes(visionState.last_task.task)){lastCalibrationTask=visionState.last_task.id;loadVisionConfiguration();}
    }
    if(visionState.last_task?.state==='error')robotText('vision-task-state','操作失败：'+visionState.last_task.status);
    const initial=visionState.result?.initial_heading||{};
    if(visionState.running&&/^\d+$/.test(visionState.source)&&initial.calibrated&&initial.imu_aligned&&initial.calibration_id!==syncedHeadingCalibration){
      syncedHeadingCalibration=initial.calibration_id;
      try{await service('station/heading-reference',{calibration_id:initial.calibration_id});robotText('vision-calibration-head','鱼头 0° 已同步到操控页；未发送运动指令。');}
      catch(error){robotText('vision-calibration-head','视觉零位已建立；IMU 同步失败：'+error.message);}
    }
    if(!visionState.running&&visionState.logs?.length)robotText('vision-task-state',visionState.logs.slice(-3).join(' '));
    if(state.page==='vision')robotText('vision-result-data',JSON.stringify(visionState.result||{},null,2));
    const result=visionState.result||{},optical=result.optical||{},yaw=result.body_yaw||{},position=result.position||{},perf=result.performance||{};
    const number=(value,digits=1)=>typeof value==='number'?value.toFixed(digits):'—';
    robotText('vision-yaw-state',`独立视觉角度 ${optical.valid?number(optical.relative_deg)+'°':'无有效值（'+(optical.reason||'等待标定')+'）'} · IMU ${yaw.usable?number(yaw.yaw_deg)+'°':'未对齐'}${result.replay?' · 录像不与当前 IMU 配对':''}`);
    robotText('vision-speed-state',`轨迹坐标 X ${number(position.x_m,3)} / Y ${number(position.y_m,3)} m · 视觉角度误差 ${number(result.vision_error_deg)}°${result.reference_aligned?'':'（等待同一零位和时间对齐）'}`);
    robotText('vision-performance-state',`设备 ${perf.device===0?'CUDA GPU 0':perf.device||'—'} · 检测 ${number(perf.detect_ms)} / 光学角度 ${number(perf.optical_ms)} / 预览 ${number(perf.preview_ms)} ms · 预览最高 3 fps · 丢弃过期采集帧 ${perf.dropped_capture_frames||0}`);
    const pool=visionState.result?.pool||{},test=pool.test||visionState.perspective_test||{};
    $('vision-perspective-files').hidden=!(pool.perspective||test.image_file);
    $('vision-perspective-files').querySelector('a').hidden=!test.image_file;
    if(pool.perspective)robotText('vision-calibration-pool',`四点透视已生效 · ${pool.width_m} × ${pool.height_m} m（设定值）· 下次自动复用`);
    if(pool.width_m&&pool.height_m&&document.activeElement!==$('vision-pool-width')&&document.activeElement!==$('vision-pool-height')){
      $('vision-pool-width').value=String(pool.width_m);$('vision-pool-height').value=String(pool.height_m);
    }
    if($('control-video-source').value==='vision')robotText('control-video-state',!visionState.running?'外部视觉未启动':visionState.ready?(visionState.source.endsWith('.mp4')?'录像回放；画面不代表控制效果。':'实时外部视觉画面'):'等待摄像头画面…');
    syncVisionParameters();renderVisionControls();updateVisionImages();
  }catch(error){robotText('vision-service-state',error.message);}
  finally{visionPolling=false;}
}
function pointInImage(event,image){
  const box=image.getBoundingClientRect(),ratio=Math.min(box.width/image.naturalWidth,box.height/image.naturalHeight);
  const width=image.naturalWidth*ratio,height=image.naturalHeight*ratio;
  const x=(event.clientX-box.left-(box.width-width)/2)/width,y=(event.clientY-box.top-(box.height-height)/2)/height;
  return Number.isFinite(x)&&Number.isFinite(y)&&x>=0&&x<=1&&y>=0&&y<=1?{x,y}:null;
}
function sendVisionPointer(event,kind){
  const point=pointInImage(event,$('vision-result'));if(!point)return;
  service('vision/action',{event:{kind,...point}}).catch(error=>robotText('vision-task-state',error.message));
}
$('vision-result').onpointerdown=event=>{event.preventDefault();event.target.setPointerCapture(event.pointerId);pointerDown=true;sendVisionPointer(event,event.button===2?'right':'down');};
$('vision-result').onpointermove=event=>{if(pointerDown&&Date.now()-lastPointerAt>40){lastPointerAt=Date.now();sendVisionPointer(event,'move');}};
$('vision-result').onpointerup=event=>{sendVisionPointer(event,'up');pointerDown=false;};
$('vision-result').onpointercancel=()=>{pointerDown=false;};$('vision-result').oncontextmenu=event=>event.preventDefault();
$('vision-start').onclick=startVision;$('vision-stop').onclick=async()=>{try{visionState=await service('vision/stop',{});await pollVision();}catch(error){toast(error.message);}};
$('vision-speed-apply').onclick=async()=>{
  const input=$('vision-speed-target');if(!input.reportValidity())return;
  try{await service('vision/action',{event:{task:'SPEED',value:Number(input.value)}});visionNotice('巡航目标已更新；转弯和接近终点时自动减速。');}catch(error){toast(error.message);}
};
$('vision-pool-apply').onclick=async()=>{
  const width=$('vision-pool-width'),height=$('vision-pool-height');if(!width.reportValidity()||!height.reportValidity())return;
  try{await service('vision/action',{event:{task:'POOL_SIZE',width_m:Number(width.value),height_m:Number(height.value)}});visionNotice('尺寸已应用，请在画面中依次点击四角。');}catch(error){toast(error.message);}
};
document.querySelectorAll('[data-vision-task]').forEach(button=>button.onclick=()=>visionTask(button.dataset.visionTask));
document.querySelectorAll('[data-vision-layout]').forEach(button=>button.onclick=()=>applyVisionLayout(button.dataset.visionLayout));
$('vision-config-link').onclick=event=>{event.preventDefault();$('vision-settings').open=true;$('vision-settings').scrollIntoView({block:'start',behavior:'smooth'});loadVisionConfiguration();};
$('vision-open-output').onclick=async()=>{try{const result=await service('vision/open-output',{});toast('已打开采样与测试图目录：'+result.path);}catch(error){toast(error.message);}};
$('vision-board-toggle').onclick=()=>{
  if(!boardCameraActive&&!boardHost()){robotText('vision-board-state','请先填写板载相机IP');$('vision-settings').open=true;$('vision-board-host').focus();return;}
  boardCameraActive=!boardCameraActive;if(boardCameraActive)saveVision('board-host',$('vision-board-host').value);
  else robotText('vision-board-state','板载相机已关闭');renderVisionControls();updateVisionImages();
};
$('vision-board-host').value=visionSaved('board-host','');
$('vision-source').value=visionSaved('source','camera');$('vision-camera-index').value=visionSaved('camera','0');
$('vision-board-host').onchange=()=>{boardCameraActive=false;clearVisionImage('vision-board','vision-board-empty');renderVisionControls();saveVision('board-host',$('vision-board-host').value);};
for(const [id,key] of [['vision-source','source'],['vision-camera-index','camera'],['vision-model','model']])$(id).onchange=()=>saveVision(key,$(id).value);
document.querySelector('.vision-input-preview').ontoggle=updateVisionImages;
$('vision-board').onerror=()=>{clearVisionImage('vision-board','vision-board-empty');$('vision-board').dataset.retryAfter=Date.now()+2500;robotText('vision-board-state','板载画面暂不可用，将重试；请检查相机地址和设备网络。');};
$('control-video-start').onclick=async()=>{
  controlVideoActive=!controlVideoActive;$('control-video-start').textContent=controlVideoActive?'关闭画面':'开启画面';
  if(!controlVideoActive){clearVisionImage('control-camera','control-camera-empty');return;}
  if($('control-video-source').value==='vision'){if(!visionState.running)await startVision();else updateVisionImages();}
  else{
    const host=boardHost();if(!host){controlVideoActive=false;return toast('请先在视觉配置或设备设置填写板载相机地址');}
    try{$('control-camera').src=boardStreamURL(host);$('control-camera').hidden=false;$('control-camera-empty').hidden=true;robotText('control-video-state','板载相机 · '+host);}
    catch(error){controlVideoActive=false;toast(error.message);}
  }
};
$('control-video-source').onchange=()=>{controlVideoActive=false;clearVisionImage('control-camera','control-camera-empty');$('control-video-start').textContent='开启画面';};
for(const [image,empty] of [['vision-raw','vision-raw-empty'],['vision-result','vision-result-empty'],['control-camera','control-camera-empty']])$(image).onerror=()=>{clearVisionImage(image,empty);$(image).dataset.retryAfter=Date.now()+1000;robotText(image==='control-camera'?'control-video-state':'vision-task-state','画面暂不可用，等待视频源恢复');};
window.addEventListener('stationpage',event=>{if(event.detail!=='overview')clearVisionImage('control-camera','control-camera-empty');if(event.detail==='vision'||event.detail==='overview')pollVision();else updateVisionImages();});
setInterval(()=>{if(state.page==='vision'||(state.page==='overview'&&controlVideoActive))pollVision();},300);
applyVisionLayout(visionLayout);loadVisionConfiguration();pollVision();
