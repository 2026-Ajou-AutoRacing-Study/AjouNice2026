#!/usr/bin/env python3
"""Git/ROS-free Docker capacity handoff. Host needs only Python3 + NVIDIA Docker."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import platform
import signal
import subprocess
import time
import zipfile

CASES=['controlled_baseline','controlled_image','controlled_range','controlled_both']
NAMES=['E25 current','Image only','Range only','Image + range']


def command(args):
    try:
        p=subprocess.run(args,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        return {'returncode':p.returncode,'output':p.stdout}
    except OSError as exc:
        return {'returncode':127,'output':repr(exc)}


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def validate(path):
    report=json.loads(path.read_text())
    rows=report['cases']
    if {r['case'] for r in rows}!=set(CASES):raise ValueError('Missing/extra cases')
    for r in rows:
        if not r.get('valid'):raise ValueError(str(r))
        assert r['voxel_size']==[.075,.075,.2]
        assert r['camera_bev_xbound'][2]==r['camera_bev_ybound'][2]==.3
        assert r['depth_bound'][2]==.5 and r['num_proposals']==200 and len(r['classes'])==3
        assert r['frustum_shape'][0]==(198 if r['case'] in CASES[2:] else 118)
    return {r['case']:r for r in rows}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--image',default=os.environ.get('BENCH_IMAGE','ajounice/bevfusion:team-cu128-v1'))
    p.add_argument('--gpu',default='0');p.add_argument('--iterations',type=int,default=60)
    p.add_argument('--repeat-iterations',type=int,default=30)
    a=p.parse_args()
    if min(a.iterations,a.repeat_iterations)<10:p.error('Each pass needs >=10 iterations')
    root=Path(__file__).resolve().parent
    stamp=datetime.now().strftime('%Y%m%d_%H%M%S')
    out=root/'results'/('capacity_'+stamp+'_'+str(os.getpid()));out.mkdir(parents=True)
    name='ajou-capacity-'+stamp+'-'+str(os.getpid())
    metadata={'status':'RUNNING','started':datetime.now().isoformat(),'platform':platform.platform(),
              'gpu_index':a.gpu,'image':a.image,'iterations':a.iterations,'repeat_iterations':a.repeat_iterations,
              'scope':'Offline batch1 AMP; CPU image prep + model stages. No ROS, MORAI, training or accuracy evaluation.'}
    monitor=None;monitor_file=None;ok=False;start=time.monotonic()
    try:
        manifest=json.loads((root/'manifest.json').read_text())
        for rel,digest in manifest['sha256'].items():
            if sha(root/rel)!=digest:raise ValueError('Bundle checksum mismatch: '+rel)
        metadata['bundle']=manifest
        metadata['docker_image']=command(['docker','image','inspect',a.image])
        if metadata['docker_image']['returncode']:
            raise RuntimeError('Docker image missing. Use --image EXISTING_COMPATIBLE_TAG; no auto download/build.')
        metadata['nvidia_before']=command(['nvidia-smi'])
        metadata['nvidia_query']=command(['nvidia-smi','--query-gpu=index,name,driver_version,memory.total,memory.used,utilization.gpu','--format=csv'])
        metadata['docker_before']=command(['docker','ps','--format','{{.Names}} {{.Image}}'])
        metadata['cpu']=Path('/proc/cpuinfo').read_text() if Path('/proc/cpuinfo').exists() else platform.processor()
        print('Keep MORAI/training/other inference OFF. No processes will be killed.',flush=True)
        print(metadata['nvidia_query']['output'],flush=True)
        monitor_file=(out/'gpu_telemetry.csv').open('w')
        monitor=subprocess.Popen(['nvidia-smi','--query-gpu=timestamp,index,name,utilization.gpu,memory.used,power.draw,temperature.gpu,clocks.sm,clocks.mem','--format=csv','--loop-ms=500'],stdout=monitor_file,stderr=subprocess.STDOUT)
        for order,iters,label in [(CASES,a.iterations,'forward'),(list(reversed(CASES)),a.repeat_iterations,'reverse')]:
            print('PASS '+label+': '+str(iters)+' measurements/case, 6 warmup/case',flush=True)
            args=['docker','run','--rm','--name',name,'--gpus','device='+a.gpu,'--shm-size=4g',
                  '-v',str(root/'payload')+':/workspace/AjouNice2026:ro',
                  '-v',str(root/'payload/docker/bevfusion/app')+':/opt/ajounice/app:ro',
                  '-v',str(out)+':/results',
                  '-e','PYTHONPATH=/opt/ajounice:/workspace/AjouNice2026:/opt/mmdetection3d',
                  '-e','TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1','--entrypoint','python',a.image,
                  '/workspace/AjouNice2026/perception/detection3d/bevfusion/scripts/profile_base5_stages.py',
                  '--samples','/workspace/AjouNice2026/artifacts/evaluation/bevfusion/capacity_261006/samples',
                  '--output','/results/'+label+'.json','--iterations',str(iters),'--cases',*order]
            with (out/(label+'.log')).open('w') as log:
                process=subprocess.Popen(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
                for line in process.stdout:
                    log.write(line);log.flush()
                    if line.startswith('{"case":'):
                        try:
                            r=json.loads(line)
                            print(r['case']+': '+(str(round(r['forward_wall_ms']['mean'],2))+' ms' if r.get('valid') else 'FAILED '+str(r.get('error'))),flush=True)
                        except (ValueError,KeyError):pass
                if process.wait()!=0:raise RuntimeError(label+' Docker failed; see log')
            validate(out/(label+'.json'))
        first=validate(out/'forward.json');repeat=validate(out/'reverse.json')
        lines=['# Batch1 capacity result','',metadata['scope'],'',
               'Model includes upload/normalization, voxelization, neural nets, geometry/pooling and decode.',
               'Offline service adds CPU compressed decode/resize/crop/pack; excludes ROS/TCP/publish.',
               'Range variants have 198 depth bins (0.5m spacing), untrained resized depth output layer.',
               '', '| Case | Model mean / p50 / p95 ms | Offline service mean ms | Reverse model mean ms | Steady allocated MiB |',
               '|---|---|---|---|---|']
        warning=[]
        for case,label in zip(CASES,NAMES):
            r=first[case];s=r['forward_wall_ms'];other=repeat[case]['forward_wall_ms']['mean']
            lines.append('| '+label+' | '+ ' / '.join(f'{s[k]:.2f}' for k in ('mean','p50','p95'))+f" | {r['offline_service_ms']['mean']:.2f} | {other:.2f} | {r['steady_peak_allocated_mib']:.1f} |")
            if abs(other/s['mean']-1)>.10:warning.append(case+': >10% order/repeat difference; rerun idle.')
        if warning:lines+=['','Repeat warnings:',*warning]
        lines+=['','Raw JSON includes all per-stage timings, shapes, spacing and retained-point counts.',
                'Small fixed development samples; not worst-case dense scenes or a ROS throughput guarantee.']
        (out/'SUMMARY.md').write_text('\n'.join(lines)+'\n')
        metadata['status']='PASS_WITH_REPEAT_WARNING' if warning else 'PASS';ok=True
    except (Exception,KeyboardInterrupt) as exc:
        metadata['status']='FAILED';metadata['error']=repr(exc)
        print('FAILED: '+repr(exc),flush=True)
    finally:
        # Only this run's uniquely named container; never stop other workloads.
        command(['docker','stop','--time','5',name])
        if monitor:
            monitor.terminate()
            try:monitor.wait(timeout=5)
            except subprocess.TimeoutExpired:monitor.kill();monitor.wait()
        if monitor_file:monitor_file.close()
        metadata['elapsed_sec']=time.monotonic()-start
        (out/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
        files=list(out.rglob('*'))
        checks={str(f.relative_to(out)):sha(f) for f in files if f.is_file()}
        (out/'SHA256.json').write_text(json.dumps(checks,indent=2)+'\n')
        archive=out.with_suffix('.zip')
        with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
            for file in out.rglob('*'):
                if file.is_file():z.write(file,str(Path(out.name)/file.relative_to(out)))
        print('RESULT: '+metadata['status']+'\nSEND THIS ZIP: '+str(archive),flush=True)
    return 0 if ok else 1


if __name__=='__main__':raise SystemExit(main())
