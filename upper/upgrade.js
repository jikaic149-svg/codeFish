'use strict';
let releaseTicket=null;
let releaseFormat=null;
$('upgrade-probe').textContent='先选择升级包';
$('release-stage').textContent='等待选择 RobotFish .bin 或旧探针 .amphipkg';
$('release-file').onchange=async event=>{
  releaseTicket=null;$('upgrade-probe').disabled=true;
  releaseFormat=null;
  const file=event.target.files[0];
  $('upgrade-probe').textContent='先选择有效升级包';
  if(!file){$('release-status').textContent='尚未选择文件，升级还未开始。';return;}
  const bin=file.name.toLowerCase().endsWith('.bin'),pkg=file.name.toLowerCase().endsWith('.amphipkg');
  if((!bin&&!pkg)||file.size>(bin?0x330000:2200000)||!file.size||(isRobotFish()&&!bin)){
    $('release-status').textContent='请选择 RobotFish 应用 .bin（最多 0x330000 字节）；旧探针使用 .amphipkg。不能选择整片镜像或 ZIP。';
    return;
  }
  $('release-file').disabled=true;$('upgrade-probe').textContent='正在校验升级包…';
  $('release-stage').textContent='正在校验签名与镜像';$('release-progress').removeAttribute('value');$('release-percent').textContent='进行中';
  $('release-status').textContent=bin?'正在检查 ESP32-S3、robotfish 项目名和镜像完整性（无签名）…':'正在验证旧探针签名、型号和镜像完整性…';
  try{
    if(!serviceToken)await service('state');
    const bytes=new Uint8Array(await file.arrayBuffer());let raw='';
    for(let i=0;i<bytes.length;i+=8192)raw+=String.fromCharCode(...bytes.subarray(i,i+8192));
    const result=await service('probe-package',{name:file.name,data:btoa(raw)});
    releaseTicket=result.ticket;
    releaseFormat=bin?'bin':'amphipkg';
    $('release-status').textContent=`${bin?'应用镜像检查通过（无签名）':'签名校验通过'} · ${result.manifest.hardware} · v${result.manifest.version} · ${(result.manifest.size/1024).toFixed(0)} KiB`;
    if(bin)$('release-transport').value='wifi';
    $('upgrade-probe').disabled=false;
    $('upgrade-probe').textContent='开始升级 ESP32';
    $('release-stage').textContent='校验通过，等待你点击开始升级';$('release-progress').value=0;$('release-percent').textContent='待升级';
  }catch(e){
    $('release-status').textContent='校验失败：'+e.message;
    $('upgrade-probe').textContent='重新选择升级包';
    $('release-stage').textContent='校验未通过';$('release-progress').value=0;$('release-percent').textContent='未开始';
  }finally{$('release-file').disabled=false;}
};
$('upgrade-probe').onclick=async()=>{
  if(!releaseTicket||liveBusy)return;
  if(liveState.session_active)return toast('请先结束 STM32 调试会话');
  if(!confirm(releaseFormat==='bin'?'将向所选 RobotFish 上传应用 BIN 并重启。先停止运动，再确认设备地址正确。继续？':'将升级旧 ESP32 探针并重启，保留配网数据。继续？'))return;
  liveBusy=true;liveEpoch++;$('upgrade-probe').disabled=true;$('release-file').disabled=true;
  $('upgrade-probe').textContent='升级进行中…';
  $('release-stage').textContent='正在提交升级请求';$('release-progress').removeAttribute('value');$('release-percent').textContent='进行中';
  $('release-status').textContent='升级进行中，请勿断电；下方显示当前阶段与实际进度';renderConnection();
  try{
    const r=await service('probe-upgrade',{ticket:releaseTicket,transport:$('release-transport').value,
      host:releaseFormat==='bin'?robotHost():$('host-input').value.trim(),password:$('release-password').value});
    $('release-status').textContent=`升级完成 · v${r.version} · 已确认新版本启动`;
    $('release-stage').textContent='版本、换槽与健康启动已确认';$('release-progress').value=100;$('release-percent').textContent='完成 100%';
    releaseTicket=null;
  }catch(e){
    $('release-status').textContent='升级未确认完成：'+e.message;
    $('release-stage').textContent='本次请求未确认完成，请检查设备状态';
    $('release-progress').value=0;$('release-percent').textContent='未确认';
  }
  finally{$('release-password').value='';liveBusy=false;$('release-file').disabled=false;$('upgrade-probe').disabled=!releaseTicket;
    $('upgrade-probe').textContent=releaseTicket?'重新检查状态后重试':'先选择升级包';renderConnection();}
};
