"""Analyze an exported swim CSV and replay the real C++ yaw controller (no hardware)."""
import argparse
import base64
import csv
from datetime import datetime
import html
import io
import json
import os
from pathlib import Path
import subprocess

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CPP = r'''
#include <cstdio>
#include "config/AttitudeConfig.h"
int main() {
  auto settings=robofish::config::attitude::makeSettings();
  robofish::attitude::HeadingState state;
  robofish::attitude::AngleFilter filter;
  unsigned now,generation,age; float roll,pitch,yaw,surge,manualYaw;
  std::fprintf(stderr,"%g %g %g %u %g",settings.yawKp,settings.yawKi,settings.yawKd,settings.yawPeriodMs,robofish::config::attitude::kAngleFilterStageHz);
  std::printf("target_left_deg,error_left_deg,correction_right_pct,p_right_pct,i_right_pct,d_right_pct,active,filtered_roll_deg,filtered_pitch_deg,filtered_yaw_deg\n");
  while(std::scanf("%u %u %u %f %f %f %f %f",&now,&generation,&age,&roll,&pitch,&yaw,&surge,&manualYaw)==8) {
    robofish::attitude::Sample sample{{roll,pitch,yaw},{0,0,0},now-age,now-age,true,true,true};
    sample=robofish::attitude::filterAngles(sample,filter,robofish::config::attitude::kAngleFilterStageHz,settings.maximumAgeMs);
    auto result=robofish::attitude::evaluate(settings,sample,now,true,surge>0,false);
    robofish::attitude::holdHeading(settings,result,state,manualYaw!=0,generation,now);
    if(result.headingLocked) std::printf("%.6f,%.6f",-result.headingTargetDegrees,-result.yawErrorDegrees);
    else std::printf(",");
    std::printf(",%.6f,%.6f,%.6f,%.6f,%d,%.6f,%.6f,%.6f\n",result.yawCorrectionPercent,result.yawP,result.yawI,result.yawD,result.yawActive,sample.anglesDegrees.x,sample.anglesDegrees.y,sample.anglesDegrees.z);
  }
}
'''


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv',type=Path)
    parser.add_argument('--output',type=Path,default=ROOT/'build/analysis/yaw-pid-20261024')
    parser.add_argument('--compiler',default=os.environ.get('ROBOFISH_CXX','C:/msys64/ucrt64/bin/g++.exe'))
    args=parser.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    rows=list(csv.DictReader(args.csv.open(encoding='utf-8-sig')))
    stamps=np.array([datetime.fromisoformat(row['time_utc']).timestamp() for row in rows]);t=stamps-stamps[0]
    def column(key):return np.array([float(row[key]) if row.get(key) else np.nan for row in rows])
    uptime=column('device_uptime_ms');surge=column('surge_target_pct');steer=column('yaw_target_pct')
    reboot=np.flatnonzero(np.diff(uptime)<0)+1
    breaks=np.flatnonzero((np.diff(t)>.5)|(np.diff(uptime)<0))+1
    raw_yaw=column('sensor_yaw_deg');unwrapped=np.full(len(rows),np.nan)
    for group in np.split(np.arange(len(rows)),reboot):
        unwrapped[group]=np.rad2deg(np.unwrap(np.deg2rad(raw_yaw[group])))
    # The replay samples the controller only where a CSV row exists. It does not
    # reconstruct missing 100ms device samples, rates, or an unmeasured plant.
    source=out/'replay.cpp';source.write_text(CPP,encoding='utf-8')
    executable=out/'replay.exe';env=dict(os.environ)
    env['PATH']=str(Path(args.compiler).parent)+os.pathsep+env.get('PATH','')
    subprocess.run([args.compiler,'-std=c++17','-O1','-Wall','-Wextra','-Werror','-I'+str(ROOT/'main'),str(source),'-o',str(executable)],check=True,env=env)
    lines=[];generation=0
    for i,row in enumerate(rows):
        if i in reboot:generation+=1
        lines.append(f"{int(uptime[i])} {generation} {int(float(row['imu_device_age_ms']))} "
                     f"{row['sensor_roll_deg']} {row['sensor_pitch_deg']} {raw_yaw[i]} {surge[i]} {steer[i]}")
    run=subprocess.run([str(executable)],input='\n'.join(lines),capture_output=True,text=True,check=True,env=env)
    replay=run.stdout;kp,ki,kd,period,hz=run.stderr.split()
    replay_rows=list(csv.DictReader(io.StringIO(replay)))
    filtered_yaw=np.array([float(row['filtered_yaw_deg']) for row in replay_rows]);filtered_continuous=np.full(len(rows),np.nan)
    for group in np.split(np.arange(len(rows)),reboot):filtered_continuous[group]=np.rad2deg(np.unwrap(np.deg2rad(filtered_yaw[group])))
    with (out/'controller-replay.csv').open('w',encoding='utf-8-sig',newline='') as handle:
        writer=csv.DictWriter(handle,fieldnames=['time_s','device_uptime_ms',*replay_rows[0]])
        writer.writeheader();writer.writerows(dict(time_s=round(t[i],3),device_uptime_ms=int(uptime[i]),**row) for i,row in enumerate(replay_rows))
    spans=[];start=0
    for end in range(1,len(rows)+1):
        if end<len(rows) and (surge[end],steer[end],rows[end]['mode'])==(surge[start],steer[start],rows[start]['mode']):continue
        if surge[start]>0:
            indexes=np.arange(start,end);y=unwrapped[indexes]
            spans.append(dict(start_s=round(t[start],2),duration_s=round(t[end-1]-t[start],2),mode=rows[start]['mode'],surge_pct=surge[start],yaw_command_pct=steer[start],
                              yaw_change_deg=round(y[-1]-y[0],2),yaw_slope_dps=round(float(np.polyfit(t[indexes]-t[start],y,1)[0]),3) if len(indexes)>1 else None))
        start=end
    invalid=[dict(time_s=round(t[i],3),rtt_ms=float(row['wifi_http_rtt_ms']),device_age_ms=float(row['imu_device_age_ms'])) for i,row in enumerate(rows) if row['imu_valid']!='True']
    summary=dict(rows=len(rows),duration_s=round(float(t[-1]),3),static_duration_s=round(float(t[np.flatnonzero(surge>0)[0]]),3),
                 reboot_gaps=[dict(start_s=round(float(t[i-1]),3),end_s=round(float(t[i]),3),gap_s=round(float(t[i]-t[i-1]),3)) for i in reboot],
                 invalid_display_samples=invalid,motion_segments=spans)
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    font=FontProperties(fname='C:/Windows/Fonts/msyh.ttc')
    plt.rcParams['font.family']=font.get_name();plt.rcParams['axes.unicode_minus']=False
    def plot_broken(axis,values,*pos,**kw):
        tx=t.copy();values=np.asarray(values,dtype=float).copy();tx[breaks]=np.nan
        return axis.plot(tx,values,*pos,**kw)
    fig,axes=plt.subplots(4,1,figsize=(13,11),sharex=True,layout='constrained')
    plot_broken(axes[0],unwrapped,color='#146cb4',label='实测 yaw（连续展开，左转为正）')
    axes[0].set_ylabel('yaw / °');axes[0].legend(loc='upper left')
    for key,color in [('surge_target_pct','#19855a'),('yaw_target_pct','#b7790a')]:plot_broken(axes[1],column(key),color=color,label='前进目标 / %' if key.startswith('surge') else '手动转向 / %（右转为正）')
    axes[1].set_ylabel('命令 / %');axes[1].legend(loc='upper left')
    for key,color,label in [('tail1_command_deg','#8d55b5','前尾舵机命令'),('tail2_command_deg','#d47716','后尾舵机命令')]:plot_broken(axes[2],column(key),color=color,label=label,linewidth=.9)
    axes[2].set_ylabel('舵机 / °');axes[2].legend(loc='upper left')
    for key,color,label in [('left_esc_pct','#288969','左桨'),('right_esc_pct','#bf4857','右桨')]:plot_broken(axes[3],column(key),color=color,label=label)
    axes[3].set_ylabel('双桨 / %');axes[3].set_xlabel('距记录开始 / 秒');axes[3].legend(loc='upper left')
    for axis in axes:
        axis.grid(alpha=.2)
        for gap in summary['reboot_gaps']:axis.axvspan(gap['start_s'],gap['end_s'],color='#cbd5e1',alpha=.6)
    fig.suptitle('水中自由游动实测：航向、指令与执行器命令（阴影为重启期间记录中断）')
    fig.savefig(out/'measured.png',dpi=150);fig.savefig(out/'measured.svg');plt.close(fig)
    correction=np.array([float(row['correction_right_pct']) for row in replay_rows])
    target=np.array([float(row['target_left_deg']) if row['target_left_deg'] else np.nan for row in replay_rows])
    target=filtered_continuous+(target-filtered_yaw+180)%360-180
    error=np.array([float(row['error_left_deg']) if row['error_left_deg'] else np.nan for row in replay_rows])
    fig,axes=plt.subplots(3,1,figsize=(13,8),sharex=True,layout='constrained')
    plot_broken(axes[0],unwrapped,color='#146cb4',label='实测 yaw（连续展开）',alpha=.55)
    plot_broken(axes[0],filtered_continuous,color='#148269',label='软件二阶滤波回放')
    plot_broken(axes[0],target,color='#dc8c1b',label='回放控制器记录的目标',linestyle='--');axes[0].set_ylabel('角度 / °');axes[0].legend()
    plot_broken(axes[1],error,color='#8953b0');axes[1].set_ylabel('最短角度误差 / °')
    plot_broken(axes[2],correction,color='#148269');axes[2].set_ylabel('修正 / %\n右正、左负');axes[2].set_xlabel('距记录开始 / 秒')
    for axis in axes:axis.grid(alpha=.2)
    fig.suptitle('真实 C++ PID 离线回放：保留原始实测运动，只展示控制器输出；不是闭环水测结果')
    fig.savefig(out/'pid-replay.png',dpi=150);fig.savefig(out/'pid-replay.svg');plt.close(fig)
    table=''.join('<tr>'+''.join(f'<td>{html.escape(str(segment[key]))}</td>' for key in ('start_s','duration_s','mode','surge_pct','yaw_command_pct','yaw_change_deg','yaw_slope_dps'))+'</tr>' for segment in spans)
    fig,axes=plt.subplots(3,1,figsize=(13,8),sharex=True,layout='constrained')
    for axis,key,raw,label,circular in zip(axes,('filtered_roll_deg','filtered_pitch_deg','filtered_yaw_deg'),
            (column('sensor_roll_deg'),column('sensor_pitch_deg'),raw_yaw),('Roll（右翻为正）','Pitch（抬头减小）','Yaw（左转增大）'),(True,False,True)):
        smooth=np.array([float(row[key]) for row in replay_rows])
        if circular:
            for group in np.split(np.arange(len(rows)),reboot):
                raw[group]=np.rad2deg(np.unwrap(np.deg2rad(raw[group])))
                smooth[group]=np.rad2deg(np.unwrap(np.deg2rad(smooth[group])))
        plot_broken(axis,raw,color='#8294ad',label='传感器原始值',linewidth=.9)
        plot_broken(axis,smooth,color='#148269',label='二阶低通回放',linewidth=1)
        axis.set_ylabel(label+' / °');axis.grid(alpha=.2);axis.legend(loc='upper left')
    axes[-1].set_xlabel('距记录开始 / 秒');fig.suptitle(f'三轴滤波对照：两级低通，每级 {hz} Hz（按 CSV 时间回放）')
    fig.savefig(out/'filter-comparison.png',dpi=150);fig.savefig(out/'filter-comparison.svg');plt.close(fig)
    pictures=''.join(f'<img alt="{name}" src="data:image/png;base64,{base64.b64encode((out/name).read_bytes()).decode()}">' for name in ('measured.png','filter-comparison.png','pid-replay.png'))
    gap_note='；'.join(f"{gap['start_s']}～{gap['end_s']} 秒中断 {gap['gap_s']} 秒，恢复后 MCU 运行时间和帧计数重计" for gap in summary['reboot_gaps']) or '没有发现运行时间倒退'
    stale_note=f"{len(invalid)} 条显示无效样本"+(f"，遥测往返范围 {min(row['rtt_ms'] for row in invalid):.2f}～{max(row['rtt_ms'] for row in invalid):.2f} ms" if invalid else '')
    report=f'''<!doctype html><html lang="zh"><meta charset="utf-8"><title>yaw 数据与 PID 回放</title><style>body{{font:16px/1.7 "Microsoft YaHei",sans-serif;max-width:1250px;margin:30px auto;padding:20px;color:#172536}}img{{width:100%;height:auto}}table{{border-collapse:collapse;width:100%}}td,th{{padding:8px;border:1px solid #cbd5e1}}.note{{padding:15px;background:#edf4fa}}</style>
    <h1>yaw 数据分析与闭环实现</h1><p>{len(rows)} 条，{t[-1]:.3f} 秒；前 {summary['static_duration_s']} 秒为首段静止。左转 yaw 增大，上浮 pitch 减小，roll 增大向右翻滚。</p>
    <p class="note">闭环默认关闭，开启后在开始前进时记录当前航向；手动转向优先，松开后重新记录。{period} ms 更新，舵机与双桨混合驱动，无角度死区。默认 Kp={kp}、Ki={ki}、Kd={kd}，软件二阶低通每级 {hz} Hz（直接读取 AttitudeConfig.h），保留原有 ±20% 修正限幅及执行器限速。当前数据不能验证这些初值的闭环稳定性。</p>
    <p>{gap_note}。这说明发生过重启，原因不能由 CSV 判定。另有 {stale_note}，需要结合设备 IMU 标记与计数区分通信过期和传感器中断。本次 3 条设备端 IMU 都仍有效。</p>
    <table><tr><th>开始 / s</th><th>记录跨度 / s</th><th>模式</th><th>前进 / %</th><th>右转命令 / %</th><th>yaw 净变化 / °</th><th>yaw 拟合变化率 / °/s</th></tr>{table}</table>
    <p>后两段转向的 yaw 变化方向符合用户确认的极性；直行也存在航向偏移。水中自由游动同时受到惯性、水流与姿态影响，不能把变化率直接当作舵机独立增益。</p>
    {pictures}<p class="note">回放直接编译并调用固件相同的 C++ 控制器；只在 CSV 现有采样点更新，未补造缺失设备样本。CSV 没有角速度列，回放填零角速度以满足输入结构，yaw 的 D 项来自角度误差差分。原始运动轨迹不受回放输出影响，所以它展示“控制器会发出什么修正”，不能证明实鱼已返回目标或量化闭环改善。</p>
    <p>同目录 controller-replay.csv 为逐样本控制器回放，summary.json 为分段统计；上位机的新同步导出将包含实鱼 PID 参数、开关、目标、误差、P/I/D 及执行器命令，供下一轮真实闭环比较。</p></html>'''
    (out/'report.html').write_text(report,encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False,indent=2));print(out/'report.html')


if __name__=='__main__':main()
