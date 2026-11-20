'use strict';
let recorded={rows:[],recording:false,count:0,dropped:0},dataPending=false,chartFrozen=null,chartHover=-1,lastCaptureError='';
let rawTelemetry={latest:{},count:0,lost:0};
let dataTableVersion='',chartFrame=0,chartLegendGroup='';
let liveRows=[],liveSource='',lastLiveSample=0,capturePending=false,captureEpoch=0;
const hiddenChartSeries=new Set();
const terminalRows=[],terminalSeen={PC:'',FISH:''};
let terminalEnabled=true,terminalDirty=true;
let terminalRenderedRows=[],terminalRenderedSource='';
function terminalNumber(value,digits=2){return Number.isFinite(value)?value.toFixed(digits):'—';}
function appendTerminal(source,key,text){
  if(!terminalEnabled||terminalSeen[source]===key)return;
  terminalSeen[source]=key;
  terminalRows.push({source,text:`${new Date().toISOString()} [${source}] ${text}`});
  if(terminalRows.length>1000)terminalRows.shift();terminalDirty=true;
}
function appendVisionTerminal(value,error=''){
  const result=value?.result||{},frame=value?.frame_age_ms,pose=result.optical||{},point=result.position||{},guide=result.guidance||{};
  if(!value?.running||!value.ready){appendTerminal('PC','offline:'+error,error||'视觉未启动或尚无画面');return;}
  const tracking=result.tracking||{},ackAge=tracking.target_ack_age_ms??(tracking.last_target_at?Date.now()-tracking.last_target_at*1000:null);
  const ackDelayed=tracking.automatic===true&&Number.isFinite(ackAge)&&ackAge>1000;
  const check=result.validation||{};
  const validationText=check.enabled?' check_1s_mps='+terminalNumber(check.speed_1s_mps,3)+
    ' common_pct='+terminalNumber(check.common_pct)+' differential_pct='+terminalNumber(check.differential_pct)+
    ' tail_drive_pct='+terminalNumber(check.tail_drive_pct)+' curve_limit_mps='+terminalNumber(check.curve_speed_limit_mps,3)+
    ' curve_distance_m='+terminalNumber(check.curve_distance_m,3)+' curve_status='+(check.curve_speed_status||'disabled'):'';
  appendTerminal('PC',`${value.process_id}:${result.frame}:${tracking.active}:${tracking.sent_targets}:${tracking.stop_reason}:${tracking.pause_count}:${ackDelayed}`,
    `${result.replay?'REPLAY':'CAMERA'} frame=${result.frame??'—'} age_ms=${terminalNumber(frame,0)} `+
    `position_valid=${point.valid===true} x_m=${terminalNumber(point.valid?point.x_m:null,3)} y_m=${terminalNumber(point.valid?point.y_m:null,3)} `+
    `v_mps=${terminalNumber(result.velocity?.speed_mps,3)} vision_valid=${pose.valid===true} yaw_left_deg=${terminalNumber(pose.valid?pose.relative_deg:null)} `+
    `imu_left_deg=${terminalNumber(result.body_yaw?.usable?result.body_yaw.yaw_deg:null)} `+
    `los_world_deg=${terminalNumber(Number.isFinite(guide.psi_ref_rad)?guide.psi_ref_rad*180/Math.PI:null)} `+
    `heading_error_deg=${terminalNumber(guide.heading_error_deg)} cross_m=${terminalNumber(guide.signed_cross_track_error_m,3)} `+
    `v_along_mps=${terminalNumber(guide.v_along_mps,3)} v_cross_mps=${terminalNumber(guide.v_cross_mps,3)} sideslip_deg=${terminalNumber(guide.sideslip_deg)} `+
    `curvature_per_m=${terminalNumber(guide.path_curvature_per_m,3)} curve_ff_deg=${terminalNumber(guide.curve_heading_feedforward_deg)} reference_limited=${guide.reference_rate_limited===true} `+
    `path_shape=${guide.path_geometry?.status||'original'} achieved_radius_m=${terminalNumber(guide.path_geometry?.achieved_min_radius_m,3)} `+
    `path_progress_m=${terminalNumber(guide.along_m,3)} path_segment=${guide.seg_index??'—'} `+
    `speed_target_mps=${terminalNumber(guide.speed_target_mps,3)} power_pct=${terminalNumber(result.speed_control?.power_percent)} `+
    `feedforward_pct=${terminalNumber(result.speed_control?.feedforward_percent)} feedforward_range=${result.speed_control?.feedforward_range||'—'} feedforward_profile=${result.speed_control?.feedforward_profile_id||'—'} `+
    `tracking=${result.tracking?.active===true} automatic=${result.tracking?.automatic===true} transport=${result.tracking?.transport||'—'} `+
    `sent_targets=${tracking.sent_targets??'—'} target_ack_age_ms=${terminalNumber(ackAge,0)} ack_delayed=${ackDelayed} `+
    `command_rtt_ms=${terminalNumber(result.tracking?.command_rtt_ms)} feedback_state=${result.speed_control?.state||'—'} `+
    `observation_age_ms=${terminalNumber(result.speed_control?.observation_age_ms,0)} receipt_age_ms=${terminalNumber(result.speed_control?.receipt_age_ms,0)} `+
    `paused=${tracking.paused===true} pause_reason=${JSON.stringify(tracking.pause_reason||'')} pause_age_ms=${terminalNumber(tracking.pause_age_ms,0)} `+
    `pause_count=${tracking.pause_count??0} last_pause_reason=${JSON.stringify(tracking.last_pause_reason||'')} last_pause_at=${terminalNumber(tracking.last_pause_at,3)} `+
    `stop_reason=${JSON.stringify(result.tracking?.stop_reason||'')} status=${JSON.stringify(result.status||'')}`+validationText);
}
function appendFishTerminal(view){
  if(!view?.online){appendTerminal('FISH','offline:'+robotError,robotError||'未收到有效鱼端回传');return;}
  const raw=view.diagnostics||{},imu=view.imu||{},pid=view.attitude_control||{},reference=view.heading_reference||{},act=view.actuators||{},radio=raw.lora||{};
  const key=`${view.host}:${raw.uptime_ms??view.checked_at}:${pid.age_ms}`;
  appendTerminal('FISH',key,
    `${stationMode==='simulation'?'SIMULATION ':''}uptime_ms=${raw.uptime_ms??'—'} `+
    `imu_valid=${imu.valid===true} imu_age_ms=${terminalNumber(imu.age_ms,0)} roll_deg=${terminalNumber(imu.valid?imu.roll:null)} pitch_deg=${terminalNumber(imu.valid?imu.pitch:null)} `+
    `yaw_left_deg=${terminalNumber(reference.valid&&reference.imu_valid?reference.current_deg:null)} target_left_deg=${terminalNumber(reference.valid?reference.target_deg:null)} `+
    `body_yaw_cw_deg=${terminalNumber(pid.body_yaw_deg)} error_cw_deg=${terminalNumber(pid.yaw_error_deg)} u_right_pct=${terminalNumber(pid.yaw_correction_pct)} turn_right_pct=${terminalNumber(pid.yaw_turn_pct)} `+
    `P_pct=${terminalNumber(pid.yaw_p)} I_pct=${terminalNumber(pid.yaw_i)} D_pct=${terminalNumber(pid.yaw_d)} `+
    `Kp=${terminalNumber(pid.pid?.kp,3)} Ki=${terminalNumber(pid.pid?.ki,3)} Kd=${terminalNumber(pid.pid?.kd,3)} deadband_deg=${terminalNumber(pid.yaw_deadband_deg)} turn_gain=${terminalNumber(pid.yaw_actuator_gain)} `+
    `allocation_profile=${pid.heading_allocation_profile||'—'} common_pct=${terminalNumber(raw.allocation?.written_common_pct)} differential_pct=${terminalNumber(raw.allocation?.written_differential_pct)} tail_drive_pct=${terminalNumber(raw.allocation?.tail_drive_pct)} tail_bias_pct=${terminalNumber(raw.allocation?.tail_bias_pct)} `+
    `tail_amp_gain=${terminalNumber(raw.allocation?.tail_amplitude_gain)} tail_amp_deg=${terminalNumber(raw.allocation?.tail_amplitude_deg)} tail_freq_hz=${terminalNumber(raw.allocation?.tail_frequency_hz)} `+
    `effective_bias_deg=${terminalNumber(raw.allocation?.effective_tail_bias_deg)} effective_amp_deg=${terminalNumber(raw.allocation?.effective_tail_amplitude_deg)} `+
    `esc_left_pct=${terminalNumber(act.left_esc,0)} esc_right_pct=${terminalNumber(act.right_esc,0)} tail1_deg=${terminalNumber(act.tail1)} tail2_deg=${terminalNumber(act.tail2)} `+
    `fin_left_deg=${terminalNumber(act.left_fin)} fin_right_deg=${terminalNumber(act.right_fin)} route=${view.control?.route||'—'} `+
    `wifi_seq=${raw.wifi?.sequence??'—'} heading_active=${pid.heading_target_active===true} `+
    `lora_seq=${radio.last_sequence??'—'} lora_age_ms=${terminalNumber(view.control?.lora_age_ms,0)} state=${view.safety_mode||'UNKNOWN'}`);
}
function terminalSelection(){const source=$('trace-source').value;return terminalRows.filter(row=>source==='all'||row.source===source);}
function terminalText(){return terminalSelection().map(row=>row.text).join('\n');}
function renderTerminal(){
  if(state.page!=='data'||document.hidden)return;
  robotText('trace-state',terminalEnabled?'持续输出':'输出已暂停');
  robotText('trace-count',`${terminalRows.length} / 1000 行 · ${currentRobot()?.online?'鱼端遥测在线':'鱼端无有效回传'}`);
  if(!terminalDirty)return;
  const output=$('trace-terminal'),rows=terminalSelection(),source=$('trace-source').value;
  const first=terminalRenderedRows.indexOf(rows[0]);
  const overlap=first<0?[]:terminalRenderedRows.slice(first);
  const reusable=source===terminalRenderedSource&&first>=0&&overlap.length<=rows.length&&overlap.every((row,index)=>row===rows[index]);
  let existing=0;
  if(reusable){for(let index=0;index<first;index++)output.firstChild.remove();existing=overlap.length;}
  else output.replaceChildren();
  if(!rows.length)output.textContent='等待 PC 视觉观测或鱼端遥测…';
  else{
    const added=document.createDocumentFragment();
    for(const row of rows.slice(existing)){const line=document.createElement('span');line.className='trace-line';line.textContent=row.text;added.append(line);}
    output.append(added);
  }
  terminalRenderedRows=rows;terminalRenderedSource=source;
  if($('trace-scroll').checked)output.scrollTop=output.scrollHeight;terminalDirty=false;
}
function requestChart(){if(!chartFrame)chartFrame=requestAnimationFrame(()=>{chartFrame=0;drawChart();});}
const PLOTS={imu:[['roll_deg','Roll 横滚','#178b7d'],['pitch_deg','Pitch 俯仰','#637bc5'],['yaw_deg','Yaw 航向','#b27c3c']],battery:[['battery_v','电池电压','#178b7d']],esc:[['left_esc_pct','左螺旋桨','#178b7d'],['right_esc_pct','右螺旋桨','#637bc5']],servo:[['left_fin_deg','左胸鳍','#178b7d'],['right_fin_deg','右胸鳍','#637bc5'],['tail1_deg','尾舵1','#b27c3c'],['tail2_deg','尾舵2','#b56d94']],latency:[['latency_ms','通信往返','#178b7d']]};
const UNITS={imu:'°',battery:'V',esc:'%',servo:'°',latency:'ms'};
function chartRows(){return (chartFrozen||($('chart-source').value==='live'?liveRows:recorded.rows)||[]).slice(-Number($('chart-range').value));}
function plotSeries(rows,key){
  let previous=null;
  return rows.map(row=>{
    const raw=row[key];if(!Number.isFinite(raw))return null;
    const value=$('chart-continuous').checked&&['roll_deg','yaw_deg'].includes(key)&&previous!==null?
      previous+((raw-previous+180)%360+360)%360-180:raw;
    previous=value;return value;
  });
}
function updateDataTelemetry(view){
  appendFishTerminal(view);
  const source=stationMode+':'+(view?.host||robotHost());
  if(source!==liveSource){liveRows=[];liveSource=source;lastLiveSample=0;chartFrozen=null;chartHover=-1;$('chart-freeze').textContent='冻结图表';}
  const now=Date.now();if(now-lastLiveSample<80)return;lastLiveSample=now;
  const valid=view?.online===true,imu=view?.imu||{},act=view?.actuators||{},battery=view?.battery||{};
  const age=Number.isFinite(imu.age_ms)?imu.age_ms+Math.max(0,now-robotSeen):null;
  const row={time:new Date(now).toISOString(),source:view?.telemetry_source||liveSource,valid,
    imu_valid:valid&&imu.valid===true&&age!==null&&age<=500,imu_age_ms:age,
    battery_v:valid&&battery.valid?battery.voltage:null,
    latency_ms:valid?(view.telemetry_rtt_ms??(Number.isFinite(view.link?.round_trip_us)?view.link.round_trip_us/1000:null)):null};
  for(const axis of ['roll','pitch','yaw'])row[axis+'_deg']=row.imu_valid?imu[axis]:null;
  for(const [key,suffix] of [['tail1','_deg'],['tail2','_deg'],['left_fin','_deg'],['right_fin','_deg'],['left_esc','_pct'],['right_esc','_pct']])row[key+suffix]=valid?(act[key]??null):null;
  liveRows.push(row);if(liveRows.length>600)liveRows.shift();
  if(state.page==='data'){renderDataLive();requestChart();}
}
function renderDataLive(){
  renderLoraTiming();
  const view=currentRobot(),row=liveRows.at(-1),online=view?.online===true;
  const single=isRobotFish()||view?.firmware==='robotfish';$('raw-telemetry-panel').hidden=single||loraControlReady()||stationMode==='simulation';
  const latencyLabel=single?'Wi-Fi 遥测往返':'遥测通信往返';
  robotText('data-latency-label',latencyLabel);
  robotText('data-latency-column',single?'Wi-Fi 往返 ms':'遥测往返 ms');
  robotText('data-latency-note',single?'电脑通过 Wi-Fi 读取鱼端状态的 HTTP 往返耗时，包含设备处理；LoRa 指令到达和执行时延需单独测量。':'当前设备上报的遥测链路往返时间，不表示 LoRa 指令的空口耗时。');
  const latencyOption=$('chart-group').querySelector('option[value="latency"]');
  if(latencyOption.textContent!==latencyLabel+' · ms')latencyOption.textContent=latencyLabel+' · ms';
  if(PLOTS.latency[0][1]!==latencyLabel){PLOTS.latency[0][1]=latencyLabel;chartLegendGroup='';}
  robotBadge('data-live-state',online?(row?.imu_valid?'实时遥测在线':'遥测在线 · IMU不可用'):'等待设备遥测',online?'online':'warning');
  for(const axis of ['roll','pitch','yaw'])robotText('data-live-'+axis,robotNumber(online&&row?.imu_valid?row[axis+'_deg']:null,'°',2));
  robotText('data-live-latency',robotNumber(online?row?.latency_ms:null,' ms',1));
  const imu=view?.imu||{};
  robotText('data-live-detail',online?`${imu.model||'当前设备'} · 数据年龄 ${robotNumber(row?.imu_age_ms,' ms')} · 合法帧 ${imu.frames??'—'} · 校验错误 ${imu.checksum_errors??'—'} · 串口错误 ${imu.receive_errors??'—'}`:(robotError||'连接设备后自动显示实时曲线，无需先开始记录。'));
  const hasBattery=[...liveRows,...(recorded.rows||[])].some(item=>Number.isFinite(item.battery_v));
  const batteryOption=$('chart-group').querySelector('option[value="battery"]');batteryOption.disabled=single&&!hasBattery;
  robotText('data-battery-note',single&&!hasBattery?'当前机器鱼未接入电池采样，电压和剩余电量不可用。':'只显示设备实际上报的数据；没有测量值的通道保持空白。');
  if(batteryOption.disabled&&$('chart-group').value==='battery')$('chart-group').value='imu';
}
function renderLoraTiming(){
  const timing=localRadio.last_command_timing,connected=localRadio.connected===true;
  for(const [id,key] of [['data-lora-queue','queue_ms'],['data-lora-write','write_ms'],['data-lora-host','host_ms']])robotText(id,robotNumber(timing?.[key],' ms',2));
  robotBadge('data-lora-state',connected?(timing?'发送端已测量':'已连接 · 等待指令'):'未连接',connected?'online':'unknown');
  const action=({target:'目标变化',stop:'全停',mode:'模式设置',attitude_on:'开启辅助',attitude_off:'关闭辅助'})[timing?.action]||'指令';
  robotText('data-lora-detail',timing?`最近${action} #${timing.sequence} · ${new Date(timing.completed_at*1000).toLocaleTimeString('zh-CN',{hour12:false})} · 只测发送端，不含空口和鱼端执行。`:'通过 LoRa 发送操作后显示本机排队与 USB 串口写入耗时，无需 Wi-Fi。');
}
function renderCapture(){
  const sourceLabel=({simulation:'仿真数据',bench:'台架遥测',mission:'Wi-Fi遥测'})[stationMode];
  document.querySelectorAll('.capture-count').forEach(el=>el.textContent=recorded.count.toLocaleString());
  document.querySelectorAll('.capture-status').forEach(el=>{el.textContent=recorded.recording?'正在记录':'未记录';el.dataset.state=recorded.recording?'online':'unknown';});
  robotText('capture-capacity',`${(recorded.capacity||30000).toLocaleString()} / ${recorded.dropped||0}`);
  const sources=[...new Set((recorded.rows||[]).map(row=>row.source))];
  robotText('capture-source',sources.length?sources.map(s=>({simulation:'仿真',http_spi:'台架',wifi_spi:'Wi-Fi',robotfish_http:'2.4G Wi-Fi'})[s]||s).join(' / '):'—');
  robotText('capture-detail',recorded.error||(recorded.recording?'后台每秒记录一条，切换页面仍继续采集。':'实时预览无需记录；点击右上角“开始记录”保存实验数据。'));
  robotText('capture-overview-state',`${recorded.recording?'正在记录':'未记录'} · ${sourceLabel} · ${recorded.count.toLocaleString()}条`);
  robotText('capture-overview-detail','每秒记录 IMU 三轴姿态、六路输出、通信延迟及安全状态；电压仅在设备提供测量值时记录。关闭服务前请导出CSV/JSON。');
  $('capture-overview-state').dataset.state=recorded.recording?'online':'unknown';
  $('capture-toggle').disabled=capturePending||modePending||(stationMode!=='simulation'&&!robotHost());$('capture-toggle').textContent=recorded.recording?'停止记录':`开始记录${sourceLabel}`;$('capture-clear').disabled=capturePending||recorded.recording||(!recorded.count&&!rawTelemetry.count);
  for(const id of ['data-export-csv','data-export-json'])$(id).disabled=!recorded.count;
  if(state.page==='data'){renderDataTable();renderDataLive();}
}
function renderDataTable(){
  const version=JSON.stringify([recorded.count,recorded.rows.at(-1)]);
  if(dataTableVersion===version)return;dataTableVersion=version;
  const body=$('data-rows');body.replaceChildren();
  if(!recorded.rows.length){const row=body.insertRow();const cell=row.insertCell();cell.colSpan=11;cell.textContent='尚无记录；实时曲线已在上方显示，开始记录后可导出。';return;}
  for(const row of recorded.rows.slice(-12).reverse()){
    const tr=body.insertRow();
    for(const value of [row.time.replace('T',' ').replace('+00:00','Z'),row.source,row.valid?'是':'否',robotNumber(row.roll_deg,'',2),robotNumber(row.pitch_deg,'',2),robotNumber(row.yaw_deg,'',2),robotNumber(row.battery_v,'',3),robotNumber(row.left_esc_pct),robotNumber(row.right_esc_pct),robotNumber(row.latency_ms,'',1),row.safety])tr.insertCell().textContent=value;
  }
}
async function pollData(){
  if(dataPending||location.protocol==='file:'||document.hidden)return;dataPending=true;
  const epoch=captureEpoch;
  try{const [saved,raw]=await Promise.allSettled([service('data'),service('telemetry')]);
    if(saved.status==='fulfilled'&&epoch===captureEpoch)recorded=saved.value;
    if(raw.status==='fulfilled')rawTelemetry=raw.value;
    if(state.page==='data')renderRawTelemetry();if(recorded.error&&recorded.error!==lastCaptureError)log('数据记录：'+recorded.error,'warn');lastCaptureError=recorded.error||'';renderCapture();if(state.page==='data')requestChart();
    if(saved.status==='rejected')robotText('capture-detail',saved.reason.message);
  }
  catch(e){robotText('capture-detail',e.message);}
  finally{dataPending=false;}
}
function renderRawTelemetry(){
  const data=rawTelemetry;
  robotText('raw-telemetry-state',data.error||(data.online?'遥测在线':'等待原始采样'));
  robotText('raw-telemetry-count',`已记录 ${data.count||0} 条 · 游标丢失 ${data.lost||0} 条 · 缓存覆盖 ${data.overwritten||0} 条 · 源序号缺口 ${data.source_gaps||0} 条`);
  const rows=Object.values(data.latest||{}).map(item=>{
    const row=document.createElement('tr');
    const label=({1:'电压 ADC',2:'IMU 原始帧',3:'串口原始帧'})[item.kind]||'通道 '+item.kind;
    const payload=item.adc?`${item.data}  （ADC ${item.adc.battery_counts} / VREF ${item.adc.reference_counts}）`:item.data;
    for(const value of [label,item.source_sequence,item.sampled_at_ms+' ms',item.valid?'有效':'无效',payload]){
      const cell=document.createElement('td');cell.textContent=String(value);row.append(cell);
    }
    return row;
  });
  $('raw-telemetry-rows').replaceChildren(...rows);
  for(const format of ['csv','json'])$('raw-export-'+format).disabled=!data.count;
}
async function captureAction(action){
  if(capturePending)return;capturePending=true;captureEpoch++;renderCapture();
  try{recorded=await service('data/record',{action});if(action==='clear'){chartFrozen=null;$('chart-freeze').textContent='冻结图表';}renderCapture();drawChart();log(({start:'开始后台数据记录',stop:'停止数据记录',clear:'已清空本次内存数据'})[action],'info');}
  catch(e){toast(e.message);log(e.message,'warn');}
  finally{capturePending=false;renderCapture();}
}
function drawChart(){
  if(state.page!=='data')return;
  const canvas=$('data-chart'),box=canvas.getBoundingClientRect();if(box.width<20)return;
  const ratio=window.devicePixelRatio||1,width=box.width,height=box.height;
  const pixelWidth=Math.round(width*ratio),pixelHeight=Math.round(height*ratio);
  if(canvas.width!==pixelWidth)canvas.width=pixelWidth;if(canvas.height!==pixelHeight)canvas.height=pixelHeight;
  const context=canvas.getContext('2d');context.setTransform(ratio,0,0,ratio,0,0);
  const css=getComputedStyle(document.body),ink=css.getPropertyValue('--muted'),line=css.getPropertyValue('--line');
  context.fillStyle=css.getPropertyValue('--card');context.fillRect(0,0,width,height);
  const group=$('chart-group').value,series=PLOTS[group].filter(([key])=>!hiddenChartSeries.has(key)),rows=chartRows();
  const plotted=series.map(([key])=>plotSeries(rows,key));
  if(chartLegendGroup!==group){chartLegendGroup=group;$('chart-legend').replaceChildren(...PLOTS[group].map(([key,label,color])=>{
    const button=document.createElement('button'),dot=document.createElement('i');button.type='button';button.className='chart-series';button.dataset.series=key;
    dot.style.background=color;button.append(dot,document.createTextNode(label));button.setAttribute('aria-pressed',String(!hiddenChartSeries.has(key)));
    button.onclick=()=>{if(hiddenChartSeries.has(key))hiddenChartSeries.delete(key);else hiddenChartSeries.add(key);button.setAttribute('aria-pressed',String(!hiddenChartSeries.has(key)));requestChart();};return button;
  }));}
  const values=plotted.flat().filter(Number.isFinite);
  $('chart-empty').hidden=values.length>0;$('chart-export').disabled=!values.length;
  robotText('chart-empty',!series.length?'点击图例选择要查看的曲线。':$('chart-source').value==='recorded'?'尚无该通道的记录，点击右上角“开始记录”采集实验数据。':group==='battery'?'当前设备未提供电池电压测量。':'等待该通道的有效遥测；连接设备后自动显示，无需开始记录。');
  let low=values.length?Math.min(...values):0,high=values.length?Math.max(...values):1;
  const padding=high===low?Math.max(1,Math.abs(low)*.05):(high-low)*.15;low-=padding;high+=padding;
  const times=rows.map(row=>Date.parse(row.time)),span=times.length>1?times.at(-1)-times[0]:0;
  const plot={x:55,y:20,w:width-77,h:height-57},x=i=>plot.x+(span>0?(times[i]-times[0])/span:.5)*plot.w,y=v=>plot.y+(high-v)/(high-low)*plot.h;
  context.font='10px "Cascadia Code",Consolas,monospace';context.fillStyle=ink;context.textAlign='right';
  for(let i=0;i<=4;i++){const yy=plot.y+plot.h*i/4;context.strokeStyle=line;context.lineWidth=1;context.beginPath();context.moveTo(plot.x,yy);context.lineTo(width-22,yy);context.stroke();context.fillText((high-(high-low)*i/4).toFixed(group==='battery'?2:1),plot.x-10,yy+3);}
  context.textAlign='left';context.fillText(UNITS[group],plot.x,10);
  if(rows.length){context.fillText(rows[0].time.slice(11,19)+' UTC',plot.x,height-10);context.textAlign='right';context.fillText(rows.at(-1).time.slice(11,19)+' UTC',width-22,height-10);}
  series.forEach(([key,,color],seriesIndex)=>{
    const data=plotted[seriesIndex];
    context.strokeStyle=color;context.fillStyle=color;context.lineWidth=2;context.beginPath();let active=false;
    data.forEach((value,i)=>{if(!Number.isFinite(value)){active=false;return;}const xx=x(i),yy=y(value);if(active)context.lineTo(xx,yy);else context.moveTo(xx,yy);active=true;});context.stroke();
    data.forEach((value,i)=>{if(Number.isFinite(value)&&(rows.length<15||i===chartHover)){context.beginPath();context.arc(x(i),y(value),2.7,0,Math.PI*2);context.fill();}});
  });
  if(chartHover>=0&&chartHover<rows.length){context.setLineDash([3,4]);context.strokeStyle=ink;context.beginPath();context.moveTo(x(chartHover),plot.y);context.lineTo(x(chartHover),plot.y+plot.h);context.stroke();context.setLineDash([]);const row=rows[chartHover];robotText('chart-note',row.time+' · '+series.map(([key,label])=>label+' '+robotNumber(row[key],UNITS[group],2)).join(' / '));}
  else robotText('chart-note',chartFrozen?'图表已冻结，实时遥测和后台记录继续更新。':`${$('chart-source').value==='live'?'实时预览 · 最近600条遥测':'记录回放 · 每秒一条'}；无效样本留空。连续角度仅影响曲线，导出保留传感器原始值。`);
}
$('capture-toggle').onclick=()=>captureAction(recorded.recording?'stop':'start');
$('station-exit').onclick=async()=>{
  const button=$('station-exit');button.disabled=true;
  try{
    const [saved,raw]=await Promise.all([service('data'),service('telemetry')]);
    const unsaved=saved.recording||saved.count||raw.count;
    if(unsaved&&!confirm(`当前有 ${saved.count||0} 条状态记录、${raw.count||0} 条原始记录${saved.recording?'，正在采集':''}。退出会丢失未导出的数据。已导出或确认放弃并退出？`))return;
    const result=await service('shutdown',{discard_records:!!unsaved});
    if(!result.exiting)throw Error('退出尚未确认');
    const html='<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>上位机已退出</title><body style="font:18px system-ui;padding:10vh 8vw"><h1>上位机已退出</h1><p>本机后台进程已结束，可以关闭此页面并重新打开启动器。</p>'+(result.stop_confirmed===false?'<p>设备全停未确认，请检查机器鱼。</p>':'')+'</body></html>';
    location.replace(URL.createObjectURL(new Blob([html],{type:'text/html;charset=utf-8'})));
  }catch(error){toast(error.message);}
  finally{button.disabled=false;}
};
$('capture-clear').onclick=()=>{if(confirm('清空本次内存记录？需要保留时请先导出CSV或JSON。'))captureAction('clear');};
$('chart-group').onchange=$('chart-range').onchange=$('chart-continuous').onchange=()=>{chartHover=-1;drawChart();};
$('chart-source').onchange=()=>{chartFrozen=null;chartHover=-1;$('chart-freeze').textContent='冻结图表';drawChart();};
$('chart-freeze').onclick=()=>{chartFrozen=chartFrozen?null:chartRows().slice();$('chart-freeze').textContent=chartFrozen?'恢复实时':'冻结图表';drawChart();};
$('data-chart').onpointermove=event=>{
  const box=$('data-chart').getBoundingClientRect(),rows=chartRows();if(!rows.length)return;
  const fraction=Math.max(0,Math.min(1,(event.clientX-box.left-55)/(box.width-77)));
  const time=Date.parse(rows[0].time)+fraction*(Date.parse(rows.at(-1).time)-Date.parse(rows[0].time));
  let hover=0;for(let i=1;i<rows.length;i++)if(Math.abs(Date.parse(rows[i].time)-time)<Math.abs(Date.parse(rows[hover].time)-time))hover=i;
  if(hover!==chartHover){chartHover=hover;requestChart();}
};
$('data-chart').onpointerleave=()=>{chartHover=-1;requestChart();};
$('chart-export').onclick=()=>{drawChart();$('data-chart').toBlob(blob=>{if(blob)downloadBlob(blob,'robofish-'+$('chart-group').value+'.png');});};
for(const format of ['csv','json'])$('data-export-'+format).onclick=()=>{const link=document.createElement('a');link.href='/api/data/export?format='+format;link.download='robofish-data.'+format;link.click();log('导出数据 '+format.toUpperCase()+'，'+recorded.count+'条样本');};
for(const format of ['csv','json'])$('raw-export-'+format).onclick=()=>{const link=document.createElement('a');link.href='/api/data/raw/export?format='+format;link.download='robofish-raw.'+format;link.click();};
for(const format of ['csv','json'])$('data-synchronous-'+format).onclick=async()=>{
  try{const response=await fetch('/api/data/synchronous/export?format='+format);
    if(!response.ok){const data=await response.json();throw Error(data.error||'同步导出失败');}
    downloadBlob(await response.blob(),'robofish-servo-imu.'+format);
  }catch(error){toast(error.message);}
};
new ResizeObserver(requestChart).observe($('data-chart').parentElement);
window.addEventListener('stationtheme',requestChart);
window.addEventListener('stationpage',event=>{if(event.detail==='data'){renderCapture();renderRawTelemetry();pollData();requestChart();}if(event.detail==='journal')renderJournal();});
function renderJournal(){
  if($('log-drawer-body').hidden)return;
  const search=$('journal-search').value.toLowerCase(),level=$('journal-level').value;
  const rows=state.logs.filter(row=>(level==='all'||row.level===level)&&row.message.toLowerCase().includes(search));
  $('journal-rows').replaceChildren(...rows.slice(-500).reverse().map(row=>logRow(row,true)));
  $('journal-count').textContent=`${rows.length}条`+(rows.length>500?' · 显示最近500条':'');
}
$('journal-level').onchange=$('journal-search').oninput=renderJournal;
$('journal-export-csv').onclick=()=>exportLogs('csv');
$('trace-pause').onclick=()=>{terminalEnabled=!terminalEnabled;$('trace-pause').textContent=terminalEnabled?'暂停输出':'继续输出';terminalDirty=true;renderTerminal();};
$('trace-clear').onclick=()=>{terminalRows.length=0;terminalSeen.PC=terminalSeen.FISH='';terminalDirty=true;renderTerminal();};
$('trace-source').onchange=()=>{terminalDirty=true;renderTerminal();};
$('trace-scroll').onchange=()=>{terminalDirty=true;renderTerminal();};
$('trace-export').onclick=()=>downloadBlob(new Blob(['\ufeff'+terminalText()+'\n'],{type:'text/plain;charset=utf-8'}),'robotfish-trace-'+new Date().toISOString().replace(/[:.]/g,'-')+'.txt');
window.addEventListener('stationpage',event=>{if(event.detail==='data'){terminalDirty=true;appendFishTerminal(currentRobot());renderTerminal();}});
setInterval(renderTerminal,250);

renderJournal();renderCapture();pollData();setInterval(pollData,1000);
