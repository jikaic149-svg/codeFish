"""Add separately labelled, display-only short-gap interpolation to a head-feedback CSV."""
import argparse
import csv
from pathlib import Path

import numpy as np


def timestamp(row):
    return float(row.get('t') or row.get('video_s') or row['camera_sample_time'])


def put(row,status,values):
    row.update(review_state=status,review_speed_mps=values[0],review_x_px=values[1],review_y_px=values[2])


def review(rows):
    previous=None
    for i,row in enumerate(rows):
        stamp=timestamp(row)
        state=row.get('head_state') or row['head_speed_state']
        key=tuple(row.get(k) for k in ('head_speed_epoch','track_id','source','pool_calibration_id','head_calibration_id'))
        raw=(row['speed_mps'],row['head_x_px'],row['head_y_px'])
        row.update(review_state='GAP',review_speed_mps='',review_x_px='',review_y_px='')
        if (not key[0] or row.get('speed_source','white_head_forward')!='white_head_forward'
                or not np.isfinite(stamp)):
            previous=None
            continue
        if previous and (key!=previous[2] or not 0<stamp-previous[1]<=.250001 or
                         int(row['frame'])<=int(rows[i-1]['frame']) or
                         stamp<=timestamp(rows[i-1])):
            previous=None
        if state in ('MISSING','REJECTED','HELD'):continue
        if state!='TRACKING' or not all(v not in ('',None) for v in raw):
            previous=None
            continue
        values=np.array(raw,dtype=float)
        if not np.isfinite(values).all():
            previous=None
            continue
        put(row,'MEASURED',values)
        if previous is not None:
            start,t0,_,v0=previous
            for missing in rows[start+1:i]:
                t=timestamp(missing)
                put(missing,'INTERPOLATED',v0+(values-v0)*((t-t0)/(stamp-t0)))
        previous=(i,stamp,key,values)
    return rows


def plot(rows,path,baseline=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    ts=np.array([timestamp(r) for r in rows])
    ts-=ts[0]
    speed=np.array([float(r['speed_mps'] or 'nan') for r in rows])
    display=np.array([float(r['review_speed_mps']) if r['review_state']!='GAP' else np.nan for r in rows])
    fill=np.array([r['review_state']=='INTERPOLATED' for r in rows])
    signed=np.array([float(r.get('head_signed_mps') or 'nan') for r in rows])
    # Show the longest invalid interval at a readable scale.
    edges=np.diff(np.r_[False,~np.isfinite(display),False].astype(int))
    gaps=list(zip(np.where(edges==1)[0],np.where(edges==-1)[0]))
    first,last=max(gaps,key=lambda pair:ts[pair[1]-1]-ts[pair[0]]) if gaps else (0,1)
    fig,axes=plt.subplots(2,1,figsize=(11,5.3),layout='constrained')
    for ax in axes:
        if baseline is not None:
            ax.plot(ts,baseline,color='#929bad',lw=.85,alpha=.8,label='Before (same frames)')
        ax.plot(ts,display,color='#db7620',ls='--',lw=1)
        ax.plot(ts,speed,color='#006e78',lw=1.35,label='Control: gated + filtered')
        if np.any(signed<0):
            ax.plot(ts,np.where(signed<0,signed,np.nan),color='#645383',lw=.85,ls=':',label='Measured backward motion')
        ax.scatter(ts[fill],display[fill],s=10,color='#db7620',label='Review only: interpolated',zorder=3)
        ax.set_ylabel('Forward speed (m/s)')
        ax.grid(alpha=.18)
    axes[0].set_title('White head | measured forward speed and short gaps',loc='left')
    axes[0].legend(frameon=False,ncol=2,fontsize=9)
    axes[1].set_title('Detail around longest remaining gap',loc='left',fontsize=10)
    axes[1].set_xlim(ts[first]-1.5,ts[last-1]+1.5)
    axes[1].set_xlabel('Recording time (s)')
    fig.savefig(path,dpi=155)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv',type=Path)
    parser.add_argument('--baseline',type=Path,help='optional earlier replay of exactly the same frames')
    args=parser.parse_args()
    with args.csv.open(encoding='utf-8-sig',newline='') as stream:
        reader=csv.DictReader(stream)
        if not reader.fieldnames or 'head_speed_epoch' not in reader.fieldnames:
            parser.error('Need a new white-head recording/replay with head_speed_epoch')
        rows=review(list(reader))
    if not rows:parser.error('Empty capture')
    out=args.csv.with_name(args.csv.stem+'-review.csv')
    with out.open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    baseline=None
    if args.baseline:
        with args.baseline.open(encoding='utf-8-sig',newline='') as f:before=list(csv.DictReader(f))
        if len(before)!=len(rows) or any(b['frame']!=r['frame'] or abs(timestamp(b)-timestamp(r))>1e-6 for b,r in zip(before,rows)):
            parser.error('Baseline does not have the same frame IDs and sample times')
        baseline=np.array([float(r['speed_mps'] or 'nan') for r in before])
    plot(rows,out.with_suffix('.png'),baseline)
    for state in ('MEASURED','INTERPOLATED','GAP'):
        print(state,sum(r['review_state']==state for r in rows))


if __name__=='__main__':main()
