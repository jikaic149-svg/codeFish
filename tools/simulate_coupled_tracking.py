"""Offline Windows SIL. Run with the project's numpy Python; inputs/outputs are local only."""
import argparse
import csv
import ctypes as ct
import functools
import itertools
import math
import os
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import subprocess

import numpy as np

from simulate_path_planning import CONTROL, DT, POOL, make_guide
from simulate_heading_planning import TRACE_DIR
from path_guidance import PathGuidance, VelocityEstimator, wrap_to_pi
from vision_speed import VisualSpeedLoop

ROOT = Path(__file__).resolve().parents[1]
DLL = ROOT / 'build/coupled-tracking-host.dll'
CC = Path(os.environ.get('ROBOFISH_CXX', 'C:/msys64/ucrt64/bin/g++.exe'))
CASES = ('straight', 'yaw75', 'right25', 'right40', 'right62', 'left40', 's_bend', 'crossing',
         'straight_push+', 'straight_push-', 'arc_push+', 'arc_push-', 'camera_jump')
CORNER_CASES = ('s_mirror', 'corner_l', 'corner_l_mirror')
CORNER_PROBES = ('corner_chord', 'corner_blend', 'corner_point')
# 'current' is the frozen pre-fix policy used by the original comparison CSV.
VARIANTS = ('production', 'current', 'angle_only', 'no_projection_guard', 'forward_first', 'zero_to_turn',
            'balanced', 'share_travel', 'prop_heavy', 'tail_heavy', 'adaptive',
            'tangent_damped', 'tangent_no_damping', 'tangent_no_lead')
RAD = math.pi / 180.
FAST = CONTROL['guidance']['cruise_speed_mps']
PREVIEW = .24


def build():
    DLL.parent.mkdir(exist_ok=True)
    files = ['tools/coupled_tracking_host.cpp', 'main/Mode/Motion/MotionMode.cpp',
             *[f'main/Device/Actuator/{a}Motion.cpp' for a in ('Tail', 'Propulsion', 'Pectoral')]]
    subprocess.run([str(CC), '-std=c++17', '-O2', '-shared', '-Wall', '-Wextra', '-Werror',
                    '-Imain', *files, '-o', str(DLL.relative_to(ROOT))], cwd=ROOT, check=True,
                   env={**os.environ, 'PATH': str(CC.parent)+os.pathsep+os.environ['PATH']})


@functools.cache
def native():
    handle = os.add_dll_directory(str(CC.parent))
    lib = ct.CDLL(str(DLL))
    lib._deps = handle
    lib.reset_run.restype = None
    lib.actuate.argtypes = [ct.c_uint, *([ct.c_double]*5), ct.c_int, ct.c_int]
    lib.actuate.restype = ct.POINTER(ct.c_double)
    return lib


def rows(file):
    with file.open(encoding='utf-8-sig', newline='') as inp:
        return list(csv.DictReader(inp))


def references(dynamics):
    # d = tau*(u0-u1) AND dt = tau*log(u0/u1) for an exponential. Neither
    # alone is a full stopping test; use the span of estimates across the three drives.
    coast = [r for r in rows(dynamics/'analysis/out/kinematics.csv')
             if r['mode']=='3' and r['motion']=='straight']
    taus = []
    for r in coast:
        a,b,t,d = map(float, (r[k] for k in ('coast_v0','coast_v1','coast_t_s','coast_dist_m')))
        taus.extend([d/(a-b), t/math.log(a/b)])
    # Fit rate to the signed geometric EFFECTIVE bias, not target angle a second time.
    # The older trace has no effective degrees: use requested bias*24/100 as a proxy.
    fits = []
    for file in sorted(TRACE_DIR.glob('*fish_active_unique.csv')):
        use = [r for r in rows(file) if r['imu_valid']==r['heading_active']=='True'
               and r['allocation_profile']=='hybrid_gait_tail_bias']
        if not use:
            continue
        t,y,b = np.array([(float(r['uptime_ms'])*.001,float(r['yaw_left_deg']),
                           float(r.get('effective_bias_deg') or (-.24*float(r['tail_bias_pct']))))
                          for r in use]).T
        dt = np.diff(t)
        mask = (dt>.035)&(dt<.45)
        rate = ((np.diff(y)+180)%360-180)/np.where(mask,dt,1)
        ok = mask[1:]&mask[:-1]
        old, new, bias, step = rate[:-1][ok],rate[1:][ok], b[1:-1][ok]/24,dt[1:][ok]
        fits.append((file.name[:12],(old,new,bias,step)))
    assert len(fits)==2 and len(coast)==3
    best=[]
    for name,(old,new,bias,step) in fits:
        def score(pair):
            cap,tau=pair
            pred=old+(-np.expm1(-step/tau))*(cap*bias-old)
            return np.mean(np.minimum(abs(new-pred),45.))
        fit=min(itertools.product(range(6,43,2),(.25,.40,.55,.8,1.1,1.5)),key=score)
        other_name,(o,n,b,s)=next(f for f in fits if f[0]!=name)
        pred=o+(-np.expm1(-s/fit[1]))*(fit[0]*b-o)
        print(f'{name}: n={len(new)} bias->yaw cap/tau={fit}; held-out {other_name} '
              f'median rate error {np.median(abs(n-pred)):.2f} vs persistence {np.median(abs(n-o)):.2f} deg/s')
        best.append(fit)
    print('Historic straight partial-coast tau estimates (s):', np.round(taus,2))
    return sorted(best), (min(taus),max(taus))


def route(name):
    if name.startswith(('right','left','arc_push')):
        r=.48 if name.startswith('arc') else float(name[-2:])*.01
        th=np.linspace(0.,math.pi,97)
        points=np.vstack(([.40,.17],np.column_stack((1.5+r*np.sin(th),.17+r*(1.-np.cos(th)))),
                          [.37,.17+2*r]))
        if name.startswith('left'): points[:,1]=1.6-points[:,1]
        return points,0.
    if name=='crossing':
        # Two diagonal straights cross; tangents match the intervening 231-degree arc.
        a=math.atan(.48); th=np.linspace(a,-math.pi-a,57); r=.39
        arc=np.column_stack((2.03-r*(np.sin(th)-math.sin(a)),1.12+r*(np.cos(th)-math.cos(a))))
        return np.vstack(([.34,1.12-(2.03-.34)*.48],arc,[.34,arc[-1,1]+(2.03-.34)*.48])), -a
    if name in ('s_bend','s_mirror'):
        points=np.array([(.35,.30),(1.20,.30),(1.20,1.23),(2.63,1.23)])
    elif name in ('corner_l','corner_l_mirror'):
        points=np.array([(.35,.26),(1.55,.26),(1.55,1.32)])
    else:
        points=np.array([(.35,.80),(2.45,.80)])
    if name.endswith('_mirror'): points[:,1]=1.6-points[:,1]
    return points, (75*RAD if name=='yaw75' else 0.)


def bounded(guide, p, step):
    # Search the connected arc interval only; allow small backwards corrections.
    # Clip each candidate TO the interval before scoring, including the old projection.
    lo=max(0.,guide.progress_m-step*.5); hi=min(guide.cumulative[-1],guide.progress_m+step)
    i=max(0,int(np.searchsorted(guide.cumulative,lo,'right'))-1)
    j=min(len(guide.path)-1,int(np.searchsorted(guide.cumulative,hi,'right')))
    v=guide.path[i+1:j+1]-guide.path[i:j]
    lens=np.diff(guide.cumulative[i:j+1]); arc=guide.cumulative[i:j]
    s=np.clip(arc+np.sum((p-guide.path[i:j])*v,axis=1)/lens,
              np.maximum(lo,arc),np.minimum(hi,arc+lens))
    d=guide.path[i:j]+((s-arc)/lens)[:,None]*v-p
    k=int(np.argmin(np.sum(d*d,axis=1)))
    guide.progress_m=float(s[k]); guide.segment_index=i+k


def branch_probe():
    p=np.array((1.05,.835))
    out=[]
    for guarded in (False,True):
        guide=make_guide(); guide.desired_turn_radius_m=0.
        guide.start(((.25,.80),(1.15,.80),(1.15,.85),(.25,.85)), (1.05,.8),100.,(1,0))
        old=guide.progress_m
        if guarded: bounded(guide,p,.014)
        else: guide._advance_progress(p)
        out.append(100*(guide.progress_m-old))
    assert out[0]>20. and abs(out[1])<1e-5, out
    print('Adjacent-return probe, 5 cm between legs, 3.5 cm cross offset: '
          'legacy / connected arc progress change (cm):',np.round(out,2))


def truth_projection(p,guide,seg,lensq,last):
    # Independent scoring association: true state and connected ~18 cm arc neighbourhood,
    # not the controller's s. Unrelated arm of the crossing may be spatially closer.
    i=max(0,last-6); j=min(len(seg),last+7); d=p-guide.path[i:j]
    w=np.clip(np.sum(d*seg[i:j],axis=1)/lensq[i:j],0.,1.)
    res=d-w[:,None]*seg[i:j]; k=int(np.argmin(np.sum(res**2,axis=1))); index=i+k
    norm=math.sqrt(lensq[index]); dx,dy=seg[index]/norm
    tangent=math.atan2(-dy,dx); err=dx*res[k,1]-dy*res[k,0]
    return index,guide.cumulative[index]+w[k]*norm,math.hypot(*res[k]),tangent,err


def run(job):
    case,variant,cap,tau,coast,tail_share,turn_effect,configured_geometry=job
    lib=native(); lib.reset_run()
    pts,yaw=route(case); pos=pts[0].copy()
    production=variant=='production' or variant in CORNER_PROBES
    guide=PathGuidance.from_config(CONTROL) if production else make_guide()
    if variant in ('corner_point','corner_blend'):
        chord_reference=guide._chord_reference
        def probe_heading(*args):
            chord,lead,correction=chord_reference(*args)
            d=guide._point_at(guide.progress_m+guide.tangent_tracking['capture_length_m'])-guide._accepted_position
            point=math.atan2(-d[1],d[0]) if np.linalg.norm(d)>=.004 else chord
            return wrap_to_pi(chord+(.5 if variant=='corner_blend' else 1.)*wrap_to_pi(point-chord)),lead,correction
        guide._chord_reference=probe_heading
    # Paired geometry for all policies. Raw 0.60 m request tested separately.
    if not configured_geometry:
        guide.desired_turn_radius_m=.30 if case=='s_bend' else 0.
    guard=not production and variant not in ('current','angle_only','no_projection_guard')
    if guard:
        guide._advance_progress=lambda p: bounded(guide,p,budget)
        # Explicit start at waypoint 0; this is NOT a mid-route re-acquisition rule.
        guide._initial_progress=lambda p,h: None
    budget=.014
    unit=lambda a:np.array((math.cos(a),-math.sin(a)))
    guide.start(pts,pos,99.7 if production else 100.,unit(yaw))
    seg=np.diff(guide.path,axis=0); lensq=np.sum(seg*seg,axis=1)
    geom=guide.path_geometry['status']; L=float(guide.cumulative[-1])
    mid=.5*(guide.cumulative[1:]+guide.cumulative[:-1])
    psi=np.unwrap(np.arctan2(-seg[:,1],seg[:,0]))
    assert configured_geometry or geom!='unachievable', (case,geom)
    loop=VisualSpeedLoop(**CONTROL['speed'],max_age_s=CONTROL['feedback_max_age_s'])
    speeds=[0.,*[v for v,_ in loop.feedforward_samples]]
    cmds=[0.,*[c for _,c in loop.feedforward_samples]]
    vel=VelocityEstimator(); last_obs=pos.copy(); accepted_at=99.7
    history=[]; log=[]; last_ref=yaw; u=r=0.
    rejected=boundary=0; jumped=False; prev_s=0.; index=0; onset=None
    dwell=0; recovered=None; recovered_s=None; release_s=None; recovery_at=None; opposite=0.
    push=case.startswith(('straight_push','arc_push')); sign=1 if case.endswith('+') else -1
    curved=case.startswith(('right','left','arc_push'))
    radius=.48 if case.startswith('arc_push') else float(case[-2:])*.01 if curved else 0.
    normal=np.zeros(2); truth_s=0.
    split={'prop_heavy':.7,'tail_heavy':1.3}.get(variant,1.)
    for k in range(math.ceil((L/.044+19)/DT)):
        t=k*DT; now=100.+t; observed_at=now-.3
        index,truth_s,distance,tangent,signed=truth_projection(pos,guide,seg,lensq,index)
        yaw_error=abs(wrap_to_pi(yaw-tangent))/RAD
        if truth_s>=L-.43: break  # Finish process scoring BEFORE the approach or arrival layer.
        if push and onset is None and truth_s >= (1.22 if curved else .53):
            onset=k
            normal=sign*np.array((math.sin(tangent), math.cos(tangent)))
        forcing=onset is not None and 0<=k-onset<12
        if onset is not None and k-onset>=12:
            if release_s is None: release_s=truth_s
            dwell=dwell+1 if distance<=.04 and yaw_error<=12 else 0
            if dwell>=10 and recovered is None:
                recovered=(k-onset-12)*DT
                recovered_s=truth_s-release_s
                recovery_at=truth_s
            opposite=max(opposite,-sign*signed)
        history.append(pos.copy()); raw=history[max(k-3,0)].copy()
        # Same sub-millimetric repeatable ripple. A bad FIX does not displace the fish.
        raw+=.0006*np.array((math.sin(1.9*t),math.sin(2.7*t)))
        if case=='camera_jump' and not jumped and raw[0]>.92:
            raw[0]+=.32; raw[1]+=.12; jumped=True
        good=(guide.accepts_position(raw,observed_at) if production else
              not guard or np.linalg.norm(raw-last_obs)<=.014+.19*(observed_at-accepted_at))
        if good:
            vector=vel.update(raw,observed_at)
            last_obs=raw; accepted_at=observed_at
        else:
            rejected+=int(k>0); vector=vel.velocity
        sp=float(np.linalg.norm(vector)) if vector is not None else 0.
        budget=0. if not good else DT*(min(.18,sp)+.04)+.001
        if production:
            g=guide.update(raw,observed_at,speed_mps=sp,body_heading=unit(yaw),velocity=vector)
            assert good==g['position_accepted'] and g['position_sample_time']==accepted_at
        elif budget or not guard:
            g=guide.update(last_obs,observed_at+0.3,speed_mps=sp,body_heading=unit(yaw),velocity=vector)
        else:
            g=guide.last_result
        ds=g['along_m']-prev_s; prev_s=g['along_m']
        preview=PREVIEW
        if variant=='adaptive':
            preview=max(.14,min(.32,.32-.035*abs(g['path_curvature_per_m'])-.50*g['cross_track_m']))
        aim=guide._point_at(g['along_m']+preview)-last_obs
        raw_ref=math.atan2(-aim[1],aim[0]) if np.linalg.norm(aim)>.004 else last_ref
        if variant.startswith('tangent_'):
            s=g['along_m']
            psi0=float(np.interp(s,mid,psi))
            kappa=(np.interp(s+.13,mid,psi)-np.interp(s+.01,mid,psi))/.12
            nx,ny=math.sin(psi0),math.cos(psi0)
            dx,dy=last_obs-g['closest_point']
            e=dx*nx+dy*ny
            lateral=0. if vector is None else vector[0]*nx+vector[1]*ny
            # e>0 is camera-right of the oriented path; CCW correction decreases it.
            correction=math.atan2(e+(0. if variant=='tangent_no_damping' else .85*lateral),.26)
            lead=0. if variant=='tangent_no_lead' else np.clip(.15*kappa,-.48,.48)
            raw_ref=wrap_to_pi(psi0+lead+np.clip(correction,-.9,.9))
        ref=(g['psi_ref_rad'] if production or variant=='current' else
             wrap_to_pi(last_ref+np.clip(wrap_to_pi(raw_ref-last_ref),-35*RAD*DT,35*RAD*DT)))
        last_ref=ref
        if production or variant in ('current','angle_only'):
            target=g['speed_target_mps']
        else:
            align=max(0.,math.cos(wrap_to_pi(raw_ref-yaw)))
            curvature=abs(g['path_curvature_per_m'])
            curve_cap=min(FAST, .17/max(.01,curvature)) # budget rad/s, not measured radius
            if variant.startswith('tangent_'):
                future=np.interp(s+np.array([.01,.13,.25,.37]),mid,psi)
                severity=max(abs(np.diff(future)))/.12
                curve_cap=min(FAST,.17/max(.01,severity))
            target=curve_cap
            if variant=='zero_to_turn':
                if abs(wrap_to_pi(raw_ref-yaw))>35*RAD: target=0.
            elif variant!='forward_first':
                target=curve_cap*(.45+.55*align**2)
        power=loop.update(float(target),None if production and vector is None else sp,
                          accepted_at,received_at=accepted_at+.3,now=now)
        boundary|=int(any(pos<0.) or any(pos>POOL))
        clipped=0
        for sub in range(5):
            prop,bias,amp,hz,wanted,diff=lib.actuate(1000+k*100+sub*20,ref/RAD,yaw/RAD,r/RAD,
                                                    power,split,variant=='share_travel',False)[:6]
            assert abs(diff)<1e-5 and abs(bias)+amp<=32.001
            clipped+=int(amp<wanted-.05)
            # Equivalent straight-drive indices: f~sqrt(drive), A~sqrt(drive).
            tail=(amp*hz)/(37.5*1.5)*100.
            b=bias/24.
            # Unknown fractions refer to equivalent surge, NOT force/power readings.
            eq=((1-tail_share)*prop+tail_share*tail)*(1+turn_effect*b*b)
            ss=float(np.interp(eq,cmds,speeds))
            u+=(ss-u)*(-math.expm1(-.02/(.9 if ss>=u else coast)))
            # Bias needs oncoming flow or tail motion. Zero command at rest cannot spin.
            flow=.7*u/.08+.3*(amp*hz)/(37.5*1.5*.11)
            r+=(cap*RAD*b*min(1.5,flow)*(1. if b>=0 else .86)-r)*(-math.expm1(-.02/tau))
            yaw=wrap_to_pi(yaw+.02*(r-(sign*15*RAD/1.2 if forcing else 0.)))
            slip=min(.18,max(-.18,.30*r))
            pos=pos+.02*(u*unit(yaw-slip)+(.15*normal if forcing else 0.))
        log.append((t,distance,u,ds,truth_s,clipped/5.,power,yaw_error,g['along_m']-truth_s))
    a=np.array(log)
    assert np.isfinite(a).all()
    def quant(mask,col):
        if not np.any(mask): return None
        return float(np.quantile(a[mask,col],.95))
    complete=truth_s>=L-.43
    arc=(a[:,4]>=1.1)&(a[:,4]<=1.1+radius*math.pi) if curved else np.zeros(len(a),bool)
    # Equal spacing along true progress: stalling does not win with lots of near-identical samples.
    markers=np.arange(.03,min(max(a[:,4]),L-.43),.025)
    ids=np.searchsorted(np.maximum.accumulate(a[:,4]),markers)
    p95=float(np.quantile(a[ids,1],.95)) if len(ids) else None
    arc95=quant(arc,1)
    return dict(route=case,variant=variant,yaw_cap=cap,yaw_tau_s=tau,coast_s=coast,
                tail_surge_share_assumed=tail_share,turn_surge_assumed=turn_effect,geometry=geom,
                process_complete=int(complete),exit_pool=int(boundary),
                p95_path_cm=100*p95 if p95 is not None else None,peak_path_cm=100*max(a[:,1]),
                arc_p95_cm=100*arc95 if arc95 is not None else None,arc_yaw_p95_deg=quant(arc,7),
                push_delivered=int(onset is not None),recovery_s=recovered,recovery_travel_m=recovered_s,
                recovered_in_arc=(int(recovery_at is not None and recovery_at<1.1+radius*math.pi)
                                  if curved and push else None),
                opposite_peak_cm=100*opposite if push else None,
                progress_ahead_peak_cm=100*max(a[:,8]),arc_median_speed_mps=np.median(a[arc,2]) if arc.any() else None,
                elapsed_s=a[-1,0],max_progress_step_cm=100*max(a[:,3]),rejects=rejected,
                amp_clipped_pct=100*np.mean(a[:,5]),fault_injected=int(jumped),last_speed=a[-1,2])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dynamics',type=Path,default=Path.home()/'Desktop/RobotFish-Dynamics')
    parser.add_argument('--quick',action='store_true')
    parser.add_argument('--workers',type=int,default=6)
    parser.add_argument('--variants',nargs='+',choices=VARIANTS+CORNER_PROBES,default=VARIANTS)
    parser.add_argument('--cases',nargs='+',choices=CASES+CORNER_CASES,default=CASES)
    parser.add_argument('--configured-geometry',action='store_true',
                        help='Use the deployed rounding request even when it cannot fit the drawn path')
    parser.add_argument('--csv',type=Path)
    opts=parser.parse_args()
    if opts.csv and opts.csv.exists(): parser.error('output already exists')
    build()
    branch_probe()
    fits,coasts=references(opts.dynamics)
    mix=itertools.product(fits,coasts,(.2,.65),(-.25,.1))
    plants=[(*f,c,w,h) for f,c,w,h in mix]
    if opts.quick: plants=plants[:1]
    jobs=[(case,v,*p,opts.configured_geometry) for case in opts.cases for v in opts.variants for p in plants
          if v in ('production','current','balanced','adaptive')+CORNER_PROBES or v.startswith('tangent_')
          or case in ('right25','right40','arc_push+','straight_push+','yaw75','camera_jump')]
    with ProcessPoolExecutor(opts.workers) as pool: results=list(pool.map(run,jobs,chunksize=4))
    for variant in opts.variants:
        rs=[r for r in results if r['variant']==variant]
        print(f'{variant:20} completed {sum(r["process_complete"] for r in rs):3}/{len(rs):3} | '
              f'outside {sum(r["exit_pool"] for r in rs)}')
    if opts.csv:
        with opts.csv.open('x',encoding='utf-8',newline='') as out:
            writer=csv.DictWriter(out,fieldnames=results[0]);writer.writeheader();writer.writerows(results)
    assert all(r['rejects']==1 for r in results if r['route']=='camera_jump' and r['variant'] in ('balanced','production'))
    assert all(r['fault_injected'] for r in results if r['route']=='camera_jump')
    assert all(r['push_delivered'] for r in results if 'push' in r['route'])
    assert all(not r['process_complete'] for r in results if r['variant']=='zero_to_turn' and r['route']=='yaw75')
    assert len(results)==len(jobs)


if __name__=='__main__':
    main()
