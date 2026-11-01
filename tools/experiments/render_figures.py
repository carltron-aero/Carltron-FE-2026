"""Render the selected measurement figures from the included numerical exports.

Offline only: no camera, network, robot imports or actuator interfaces.
"""
from pathlib import Path
import argparse
import hashlib
import json
import re

DATA = Path(__file__).resolve().parent/'data'
DEFAULT_OUTPUT = Path(__file__).resolve().parents[2]/'docs/testing/images'
BLUE, ORANGE, GREEN, RED = '#24669b', '#cf792b', '#368376', '#b34f56'


def reject_nonfinite(value):
    raise ValueError('Non-finite JSON number: ' + value)


def load(name):
    return json.loads((DATA/name).read_text(encoding='utf-8'), parse_constant=reject_nonfinite)


def verify_data():
    manifest = load('manifest.json')
    declared = manifest['files']
    if not isinstance(declared, dict) or not declared:
        raise ValueError('The data manifest must list at least one file')
    for name, expected in declared.items():
        if not re.fullmatch(r'[a-z0-9][a-z0-9-]*\.json', name) or name == 'manifest.json':
            raise ValueError('Invalid data filename')
        if not isinstance(expected, str) or not re.fullmatch(r'[0-9a-f]{64}', expected):
            raise ValueError('Invalid SHA-256 hash: ' + name)
        actual = hashlib.sha256((DATA/name).read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError('Data hash differs: '+name)
        load(name)
    actual_files = {path.name for path in DATA.iterdir() if path.is_file()} - {'manifest.json'}
    if set(declared) != actual_files:
        raise ValueError('Data files and manifest differ: ' + ', '.join(sorted(set(declared) ^ actual_files)))
    return manifest


def render(output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,
        'axes.spines.top':False,'axes.spines.right':False,'axes.titleweight':'bold',
        'savefig.facecolor':'white'})
    output.mkdir(parents=True, exist_ok=True)
    files = []

    def finish(fig, name, axes):
        for ax in axes:
            ax.grid(axis='y', alpha=.18)
            ax.set_axisbelow(True)
        fig.savefig(output/name, dpi=170)
        plt.close(fig)
        files.append(name)

    ab = load('encoder-ab.json')['results']
    labels = [f"{r['stage']}: {r['condition']}" for r in ab]
    colors = [ORANGE if r['condition']=='on' else BLUE for r in ab]
    fig, ax = plt.subplots(1, 2, figsize=(10.5,3.8),layout='constrained')
    ax[0].bar(labels,[r['metrics']['host_read_hz'] for r in ab],color=colors)
    ax[0].axhline(150,color='#555',ls=':',label='150-Hz request')
    ax[0].set(title='OTOS read rate at 80% duty',ylabel='Successful host reads per second',ylim=(0,170))
    ax[0].legend(fontsize=8)
    x=list(range(4))
    ax[1].bar([i-.18 for i in x],[r['metrics']['native_ioctl_wall_ms']['p95'] for r in ab],.36,label='Native ioctl wall time',color=GREEN)
    ax[1].bar([i+.18 for i in x],[r['metrics']['native_return_to_python_ms']['p95'] for r in ab],.36,label='Return to Python',color=ORANGE)
    ax[1].set(xticks=x,xticklabels=labels,title='I²C call and Python return timing',ylabel='95th-percentile duration [ms]')
    ax[1].legend(fontsize=8)
    finish(fig,'encoder-ab.png',ax)

    heights = sorted(load('mounting-heights.json')['datasets'],key=lambda d:d['height_mm'])
    fig,ax=plt.subplots(1,2,figsize=(10.5,4.0),layout='constrained')
    for d,color in zip(heights,[ORANGE,BLUE,GREEN]):
        bins=d['matched_55_95_powered_speed_bins']; xx=[sum(r['range_mps'])/2 for r in bins]
        ax[0].plot(xx,[r['raw_capture_ratio'] for r in bins],'o-',color=color,label=f"{d['height_mm']} mm")
    ax[0].axhline(1,color='#555',ls=':')
    ax[0].set(title='OTOS displacement by speed',xlabel='Optical interval speed bin [m/s]',ylabel='OTOS distance / optical distance',ylim=(.4,1.16))
    ax[0].legend()
    for i,d in enumerate(heights):
        r=d['matched_55_95']
        ax[1].bar(i-.16,r['median_absolute_raw_endpoint_error_mm'],.32,color=BLUE)
        ax[1].bar(i+.16,r['p95_absolute_raw_endpoint_error_mm'],.32,color=ORANGE)
    ax[1].set(xticks=range(3),xticklabels=[f"{d['height_mm']} mm" for d in heights],ylabel='Absolute along-track endpoint difference [mm]',title='Endpoint comparison at 55–95% duty')
    ax[1].legend(handles=[Patch(color=BLUE,label='Median'),Patch(color=ORANGE,label='95th percentile')])
    finish(fig,'mounting-heights.png',ax)

    fig,axes=plt.subplots(3,2,figsize=(10.5,8.0),layout='constrained',sharex=True)
    for pair,d in zip(axes,heights):
        e=d['example']
        for rows,color,label in [(e['native'],BLUE,'OTOS'),(e['optical'],ORANGE,'Optical Motion Tracking')]:
            speeds=[r for r in rows if r.get('reported_speed_mps',r.get('fitted_speed_mps')) is not None]
            pair[0].plot([r['relative_time_s'] for r in speeds],[r.get('reported_speed_mps',r.get('fitted_speed_mps')) for r in speeds],color=color,label=label)
            pair[1].plot([r['relative_time_s'] for r in rows],[r['local_along_m'] for r in rows],color=color,label=label)
        for a in pair:
            a.axvline(e['drive_s'],color='#777',ls=':',label='Brake decision')
            a.set_xlim(-.1,1.7)
        pair[0].set(title=f"{d['height_mm']} mm · 85% · first repeat",ylabel='Forward speed [m/s]')
        pair[1].set(title=f"{d['height_mm']} mm · displacement",ylabel='Distance from start [m]')
    axes[0,0].legend(fontsize=8)
    for a in axes[-1]:a.set_xlabel('Time since actual drive start [s]')
    finish(fig,'mounting-traces.png',list(axes.flat))

    breakaway=load('breakaway.json')
    fig,ax=plt.subplots(1,2,figsize=(10.5,4),layout='constrained')
    for kind,color in [('start',BLUE),('hold',ORANGE)]:
        rows=[r for r in breakaway['start_hold'] if r['trial']==kind]
        ax[0].plot([float(r['duty_percent']) for r in rows],[float(r['optical_speed_mps'])*100 for r in rows],'o-',color=color,label=kind.capitalize())
    ax[0].set(title='Start and hold measurements',xlabel='Target duty [%]',ylabel='Optical fitted forward speed [cm/s]')
    ax[0].legend()
    rows=breakaway['lower_hold']
    for field,label,color in [('optical_speed_mps','1.2–1.8 s after prime',BLUE),('late_optical_speed_mps','2.8–3.6 s after prime',ORANGE)]:
        ax[1].plot([float(r['duty_percent']) for r in rows],[float(r[field])*100 for r in rows],'o-',label=label,color=color)
    ax[1].axhline(10,color='#777',ls=':',label='10 cm/s')
    ax[1].set(title='Speed after reducing to hold duty',xlabel='Hold duty [%]',ylabel='Optical fitted forward speed [cm/s]')
    ax[1].legend(fontsize=8)
    finish(fig,'start-hold.png',ax)

    motors=load('raised-motors.json')['rows']
    fig,ax=plt.subplots(figsize=(8.5,4.5),layout='constrained')
    for condition,label,color in [('old_installed','Original motor',BLUE),('pololu_installed','Pololu #4841',ORANGE)]:
        rows=[r for r in motors if r['condition']==condition and r['direction']=='forward']
        for clean in (True,False):
            chosen=[r for r in rows if r['consistent_rotation']==clean]
            ax.scatter([r['duty_percent'] for r in chosen],[r['provisional_gearbox_rpm_magnitude'] for r in chosen],
                marker='o',s=45,facecolors=color if clean else 'none',edgecolors=color,
                label=label+(' · clean quadrature' if clean else ' · source faults'))
    ax.set(title='Installed motors, wheels lifted: forward',xlabel='Applied duty [%]',ylabel='Encoder-derived gearbox output speed [rpm]')
    ax.legend(fontsize=8)
    finish(fig,'raised-motors.png',[ax])

    groups=load('braking.json')['groups']
    fig,ax=plt.subplots(1,2,figsize=(10.5,3.8),layout='constrained')
    for profile,color,label in [('full',BLUE,'Full brake'),('pulse60',ORANGE,'60-ms pulsed phase, then full')]:
        rows=[r for r in groups if r['profile']==profile]
        ax[0].plot([r['median']['optical_prebrake_speed_mps'] for r in rows],[100*r['median']['optical_brake_distance_m'] for r in rows],'o-',color=color,label=label)
        ax[1].plot([r['median']['optical_prebrake_speed_mps'] for r in rows],[r['median']['optical_peak_yaw_after_decision_deg'] for r in rows],'o-',color=color,label=label)
    ax[0].set(xlabel='Optical speed before brake [m/s]',ylabel='Decision-to-stop distance [cm]',title='Braking distance')
    ax[1].set(xlabel='Optical speed before brake [m/s]',ylabel='Peak heading change [deg]',title='Heading change during braking')
    ax[0].legend(fontsize=8);finish(fig,'braking.png',ax)

    learning=load('camera-learning.json')
    rows=[learning['methods'][key] for key in ['original','density-control','fullrate-spatial-holdout']]+[learning['classical']]
    labels=['Small CNN','Denser CNN','Broader kernel','Geometry\naccepted subset']
    fig,ax=plt.subplots(1,2,figsize=(10.5,4.2),layout='constrained')
    for i,r in enumerate(rows):
        ax[0].bar(i-.16,r['planar_p50_p95_max_mm'][0],.32,color=BLUE)
        ax[0].bar(i+.16,r['planar_p50_p95_max_mm'][1],.32,color=ORANGE)
        ax[1].bar(i,100*r['fraction_within10_all'],color=GREEN)
        ax[1].text(i,100*r['fraction_within10_all']+.5,f"{round(r['fraction_within10_all']*162)}/162",ha='center',fontsize=9)
    for a in ax:a.set(xticks=range(4),xticklabels=labels)
    ax[0].set(title='Same 162 held-out straight images',ylabel='Planar disagreement [mm]')
    ax[0].legend(handles=[Patch(color=BLUE,label='Median'),Patch(color=ORANGE,label='95th percentile')])
    ax[1].set(title='Frames within 10 mm',ylabel='All eligible frames within 10 mm [%]',ylim=(0,15))
    finish(fig,'camera-learning.png',ax)

    pilots=load('curve-pilots.json')['runs']
    first=pilots[0]
    fig,ax=plt.subplots(1,2,figsize=(10.5,4),layout='constrained')
    laps=first['lap_metrics'];xx=[r['lap'] for r in laps]
    ax[0].plot(xx,[r['target_mps'] for r in laps],'o--',color='#777',label='Requested')
    ax[0].plot(xx,[r['optical_steady_median_mps'] for r in laps],'o-',color=GREEN,label='Optical steady median')
    ax[0].set(xticks=xx,xlabel='Lap',ylabel='Speed [m/s]',title='Target and measured speed');ax[0].legend(fontsize=8)
    for prefix,label,color in [('fusion','Fusion prototype',BLUE),('otos','Uncorrected OTOS',ORANGE)]:
        ax[1].plot(xx,[r[prefix+'_median_2d_mm'] for r in laps],'o-',color=color,label=label+' median')
        ax[1].plot(xx,[r[prefix+'_p95_2d_mm'] for r in laps],'o--',color=color,label=label+' 95th percentile')
    ax[1].set(xticks=xx,xlabel='Lap',ylabel='Planar difference [mm]',title='Position comparison over four laps')
    ax[1].legend(fontsize=8)
    finish(fig,'curve-pilot.png',ax)

    fig,ax=plt.subplots(figsize=(7.5,3.8),layout='constrained')
    timing=first['fusion_timing_sensitivity']
    ax.plot([r['shift_s']*1000 for r in timing],[r['median_2d_mm'] for r in timing],'o-',color=BLUE,label='Median')
    ax.plot([r['shift_s']*1000 for r in timing],[r['p95_2d_mm'] for r in timing],'o-',color=ORANGE,label='95th percentile')
    ax.set(xticks=[-20,0,20],xlabel='Reference query-time shift [ms]',ylabel='Fusion / optical difference [mm]',title='Effect of the reference time offset')
    ax.legend();finish(fig,'timing-sensitivity.png',[ax])
    return files


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=DEFAULT_OUTPUT)
    parser.add_argument('--verify-only',action='store_true')
    args=parser.parse_args()
    manifest=verify_data()
    if args.verify_only:
        print(f"Verified {len(manifest['files'])} numerical exports. No hardware accessed.")
    else:
        print(json.dumps(dict(figures=render(args.output),output=str(args.output))))


if __name__=='__main__':main()
