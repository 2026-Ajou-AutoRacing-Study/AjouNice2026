#!/usr/bin/env python3
"""Read-only localhost capture dashboard; no ROS dependency or capture controls."""
import argparse
from collections import Counter,deque
import hashlib
import json
import shutil
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path

HTML=Path(__file__).with_name('base5_capture_dashboard.html').read_text()


def status_payload(root):
    root=Path(root)
    try:status=json.loads((root/'status.json').read_text())
    except FileNotFoundError:
        status=dict(stage='not_started',requested=0,finished=0,accepted_candidate=0,review=0,failed=0,updated_at=0)
    except (OSError,ValueError) as exc:
        status=dict(stage='unreadable_status',error='상태 파일을 읽을 수 없음: '+str(exc),updated_at=0)
    # Resolve only child status beneath this campaign; never serve arbitrary files.
    child=status.get('current')
    if child:
        target=(Path(child)/'status.json').resolve()
        if root.resolve() in target.parents:
            try:status['child_progress']=json.loads(target.read_text())
            except (OSError,ValueError):pass
    # Recovery campaigns expose the currently running calibration/capture path
    # separately from their human-readable current-cell label.
    nested=status.get('child_progress',{}).get('child_status')
    if nested:
        target=Path(nested).resolve()
        if root.resolve() in target.parents:
            try:status['active_capture']=json.loads(target.read_text())
            except (OSError,ValueError):pass
    ancestor=root.resolve()
    while not ancestor.exists():ancestor=ancestor.parent
    status['disk_free_gib']=shutil.disk_usage(ancestor).free/(1024**3)
    elapsed=status.get('elapsed_sec',0)
    status['accepted_per_hour']=3600*status.get('accepted_candidate',0)/elapsed if elapsed>0 else 0
    stage=status.get('stage','unknown')
    idle=('not_started','complete','complete_with_deficit','paused','interrupted',
          'stopped','stopped_for_resume_test','prepared_waiting_for_MR_reconnect','prepared_waiting_for_resume')
    halted=stage.startswith('halted')
    status['status_stale']=not (stage in idle or halted) and time.time()-status.get('updated_at',0)>20
    if stage=='not_started':label='수집 전'
    elif halted or stage in ('interrupted','stopped'):label='중단 · 확인 필요'
    elif stage in ('paused','stopped_for_resume_test'):label='일시정지'
    elif stage=='prepared_waiting_for_MR_reconnect':label='연결 대기'
    elif stage=='prepared_waiting_for_resume':label='재개 대기'
    elif stage=='complete_with_deficit':label='부족분 남음'
    elif stage=='complete':label='완료'
    elif status.get('eta_sec',0)>0:label=f"{int(status['eta_sec']/60+.5)}분"
    else:label='계측 중'
    status['eta_label']=label
    return status


def read_optional(path):
    try:return json.loads(Path(path).read_text())
    except (OSError,ValueError):return None


def reason_counts(cells):
    """Count deficit cells per reason, not repeated nested attempt references."""
    counts=Counter()
    def reasons(value):
        if isinstance(value,str):return {value}
        if isinstance(value,list):return set().union(*(reasons(v) for v in value)) if value else set()
        if isinstance(value,dict):
            return set().union(*(reasons(value[k]) for k in ('reason','reasons','review_reasons') if k in value))
        return set()
    for cell in cells:
        for reason in reasons(cell.get('reasons',[])):counts[reason]+=1
    return dict(counts.most_common(12))


class Dashboard:
    """Cache status/plan files; disk usage is sampled separately every five minutes."""
    def __init__(self,root,manifest=None,suite=None,history_file=None):
        self.root=Path(root);self.manifest=Path(manifest) if manifest else None
        self.deck=json.loads(self.manifest.read_text()) if self.manifest else None
        self.manifest_sha256=hashlib.sha256(self.manifest.read_bytes()).hexdigest() if self.manifest else None
        self.suite=json.loads(Path(suite).read_text()) if suite else None
        prefix=read_optional(self.root/'imported_prefix.json') or {}
        self.imported_root=(Path(prefix['original_output']) if prefix.get('manifest_sha256')==self.manifest_sha256
            and prefix.get('original_output') else None)
        self.motion_modes=[]
        if self.deck:
            rows=self.deck['scenes'];order=self.deck.get('collection_order',{})
            # The scene entry describes PRIMARY motion. A moving secondary also
            # makes the scene animated: use the all-actor ordering contract.
            if order.get('policy')=='stationary-first; animated-last; stable within each group' and order.get('stationary',0)+order.get('animated',0)==len(rows):
                self.motion_modes=['stationary']*order['stationary']+['animated']*order['animated']
            else:
                for row in rows:
                    meta=read_optional(row['metadata']) if row.get('metadata') else None
                    movers=any(a.get('motion') not in ('static','standing') for a in (meta or {}).get('runtime_actors',[]))
                    self.motion_modes.append('animated' if movers or row.get('motion') not in ('static','standing') else 'stationary')
        self.history=deque(maxlen=20000);self.history_file=Path(history_file) if history_file else None
        self.last_sample_wall=0.;self.last_signature=None;self.cache={};self.lock=threading.RLock()
        self.storage_sample=None;self.storage_error=None
        if self.history_file:
            if self.history_file.resolve()==self.root.resolve() or self.root.resolve() in self.history_file.resolve().parents:
                raise ValueError('Dashboard history must live outside the capture root')
            try:
                with self.history_file.open() as handle:
                    for line in handle:
                        try:
                            row=json.loads(line)
                            if row.get('capture_root')==str(self.root.resolve()):
                                if all(isinstance(row.get(k),(int,float)) and row[k]>=0 for k in ('elapsed_sec','accepted','finished','review','observed_at')):
                                    if self.history and any(row[k]<self.history[-1][k] for k in ('elapsed_sec','accepted','finished')):self.history.clear()
                                    self.history.append(row)
                        except ValueError:continue # tolerate a torn last history sample
            except FileNotFoundError:pass
            if self.history:
                last=self.history[-1]
                self.last_signature=(last['elapsed_sec'],last['accepted'],last['finished'],last['review'])
                self.last_sample_wall=last['observed_at']

    def cached_json(self,path):
        path=Path(path)
        try:
            stat=path.stat();key=(stat.st_ino,stat.st_size,stat.st_mtime_ns)
        except OSError:return None
        old=self.cache.get(str(path))
        if old is None or old[0]!=key:
            self.cache[str(path)]=(key,read_optional(path))
        return self.cache[str(path)][1]

    def payload(self,now=None):
        with self.lock:return self._payload(time.time() if now is None else now)

    def sample_storage(self):
        """Read-only allocated-size scan, off the HTTP thread; hardlinks counted once."""
        if not self.root.exists():return
        status=read_optional(self.root/'status.json') or {}
        approved=status.get('accepted_candidate',0)
        try:
            roots=[str(self.root)]
            if self.imported_root and self.imported_root.resolve()!=self.root.resolve():roots.append(str(self.imported_root))
            result=subprocess.run(['du','-sk','--',*roots],capture_output=True,
                text=True,timeout=60,check=True)
            sample=dict(bytes=sum(int(line.split()[0]) for line in result.stdout.splitlines() if line.strip())*1024,accepted=approved,
                observed_at=time.time())
            with self.lock:self.storage_sample=sample;self.storage_error=None
        except (OSError,ValueError,IndexError,subprocess.SubprocessError) as exc:
            with self.lock:self.storage_error=str(exc)

    def _payload(self,now):
        status=status_payload(self.root);warnings=[]
        elapsed=status.get('elapsed_sec',0);accepted=status.get('accepted_candidate',0)
        finished=status.get('finished',0);review=status.get('review',0)
        # A new/replaced campaign or broken counter must not yield a negative rate.
        if self.history and (elapsed<self.history[-1]['elapsed_sec'] or accepted<self.history[-1]['accepted'] or finished<self.history[-1]['finished']):
            self.history.clear();self.last_signature=None
            warnings.append('누적 값이 되돌아가 차트를 새로 계측함')
        signature=(elapsed,accepted,finished,review)
        if status.get('stage') not in ('not_started','unreadable_status') and elapsed>0 and signature!=self.last_signature and now-self.last_sample_wall>=15:
            row=dict(capture_root=str(self.root.resolve()),observed_at=now,elapsed_sec=elapsed,
                accepted=accepted,finished=finished,review=review,stage=status.get('stage'))
            self.history.append(row);self.last_sample_wall=now;self.last_signature=signature
            if self.history_file:
                try:
                    self.history_file.parent.mkdir(parents=True,exist_ok=True)
                    if self.history_file.exists() and self.history_file.stat().st_size>16*1024**2:
                        temp=self.history_file.with_suffix('.tmp')
                        with temp.open('w') as f:
                            for point in self.history:f.write(json.dumps(point)+'\n')
                        temp.replace(self.history_file)
                    else:
                        torn=False
                        if self.history_file.exists() and self.history_file.stat().st_size:
                            with self.history_file.open('rb') as f:
                                f.seek(-1,2);torn=f.read(1)!=b'\n'
                        with self.history_file.open('a') as f:
                            if torn:f.write('\n')
                            f.write(json.dumps(row)+'\n')
                except OSError as exc:warnings.append('차트 이력 저장 실패: '+str(exc))
        points=list(self.history);rate_points=[]
        for i,point in enumerate(points):
            previous=next((p for p in points[max(0,i-80):i] if p['elapsed_sec']>=point['elapsed_sec']-600),None)
            dt=point['elapsed_sec']-previous['elapsed_sec'] if previous else 0
            rate=3600*(point['accepted']-previous['accepted'])/dt if dt>=60 else None
            rate_points.append(dict(elapsed_sec=point['elapsed_sec'],rate=rate))
        status['recent_accepted_per_hour']=rate_points[-1]['rate'] if rate_points else None
        # Limit HTTP/chart work even after multi-day collection.
        stride=max(1,len(points)//600)
        status['history']=points[::stride][-600:]
        status['rate_history']=rate_points[::stride][-600:]
        if points and status['history'][-1]!=points[-1]:status['history'].append(points[-1]);status['rate_history'].append(rate_points[-1])
        status['history_note']='대시보드 관측 이력 · 활성 누적시간 축 · 실행 이전 이력은 복원하지 않음'
        bins=[];family=Counter();directions=Counter();ranges=Counter()
        if status.get('accepted_bins') is not None:
            for key,count in status['accepted_bins'].items():
                try:fam,distance,role=json.loads(key)
                except (ValueError,TypeError):continue
                bins.append(dict(family=fam,distance=distance,role=role,accepted=count))
        else:
            for row in status.get('results',[]):
                if row.get('status')=='accepted_candidate':bins.append(dict(family=row.get('family'),distance=row.get('distance_bounds'),role=row.get('role'),accepted=1))
        for b in bins:
            family[b['family']]+=b['accepted'];directions[b['role'] or 'unknown']+=b['accepted']
            ranges['–'.join(map(str,b['distance']))+'m' if b['distance'] else '배경/미지정']+=b['accepted']
        status.update(coverage_bins=bins,direction_counts=dict(directions),distance_counts=dict(ranges))
        if self.deck:
            rows=self.deck['scenes'];requested=status.get('requested') or len(rows)
            if status.get('stage')=='not_started':status['requested']=len(rows)
            contract=self.cached_json(self.root/'batch_contract.json') or self.cached_json(self.root/'campaign_contract.json')
            indices=contract.get('indices') if contract else list(range(min(requested,len(rows))))
            try:planned=[rows[i] for i in indices]
            except (IndexError,TypeError):planned=[];warnings.append('manifest와 수집 indices가 맞지 않음')
            expected=Counter(r.get('family') for r in planned)
            status['family_progress']=[dict(family=f,planned=n,accepted=family[f]) for f,n in expected.items()]
            modes=[self.motion_modes[i] for i in indices] if planned else []
            status['motion_plan']=dict(Counter(modes))
            status['motion_checked']=dict(Counter(modes[:finished])) if status.get('mode')!='replenishment' else None
            status['dataset_id']=self.deck.get('dataset_id');status['environment']=self.deck.get('config',{}).get('collection_environment')
            status['manifest']=str(self.manifest.resolve())
            if len(planned)!=requested:warnings.append('요청 수와 manifest/indices 개수가 다름')
            if contract and contract.get('manifest')!=self.manifest_sha256:
                warnings.append('대시보드 manifest hash와 campaign 계약이 다름')
        else:
            status['family_progress']=[dict(family=f,planned=None,accepted=n) for f,n in family.items()]
        quota=self.cached_json(self.root/'quota.json')
        status['failure_reasons']=reason_counts((quota or {}).get('cells',[]))
        status['failure_checkpoint_finished']=(quota or {}).get('target',0)-(quota or {}).get('pending',0)
        status['pending']=max(0,status.get('requested',0)-finished)
        status['success_fraction']=accepted/finished if finished else None
        active=status.get('active_capture') or status.get('child_progress') or {}
        status['recent_attempts']=[{k:r.get(k) for k in ('id','family','status','review_reasons','duration_sec')} for r in active.get('results',[])[-8:]]
        status['suite_progress']=[]
        if self.suite:
            for env in self.suite.get('environments',[]):
                if self.imported_root and Path(env['capture_output']).resolve()==self.imported_root.resolve():
                    s=status
                else:s=self.cached_json(Path(env['capture_output'])/'status.json') or {}
                status['suite_progress'].append(dict(environment=env['environment'],stage=s.get('stage','not_started'),
                    requested=env['expected_frames'],accepted=s.get('accepted_candidate',0),finished=s.get('finished',0),review=s.get('review',0)))
            status['suite_target']=sum(e['requested'] for e in status['suite_progress'])
            status['suite_accepted']=sum(e['accepted'] for e in status['suite_progress'])
        status['capture_root']=str(self.root.resolve());status['diagnostic_warnings']=warnings
        sample=self.storage_sample
        storage=dict(sampled_at=sample['observed_at'] if sample else None,
            used_gib=sample['bytes']/1024**3 if sample else None,
            total_gib=None,additional_gib=None,free_after_gib=None,
            sample_accepted=sample['accepted'] if sample else 0,error=self.storage_error)
        if sample and sample['accepted']>0 and status.get('requested',0)>0:
            per_approved=sample['bytes']/sample['accepted']/1024**3
            storage['total_gib']=per_approved*status['requested']
            storage['additional_gib']=per_approved*max(0,status['requested']-accepted)
            storage['free_after_gib']=status['disk_free_gib']-storage['additional_gib']
        status['storage_estimate']=storage
        return status


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture-root',type=Path,required=True)
    p.add_argument('--manifest',type=Path,help='Read-only planned denominators; never changes collector inputs')
    p.add_argument('--suite',type=Path,help='Read-only multi-environment progress, e.g. environment_suite/suite.json')
    p.add_argument('--history-file',type=Path,help='Dashboard-only journal OUTSIDE capture root; default ~/.cache/ajounice/base5')
    p.add_argument('--port',type=int,default=8075);p.add_argument('--once',action='store_true');a=p.parse_args()
    journal=a.history_file or Path.home()/'.cache/ajounice/base5'/('dashboard_'+hashlib.sha256(str(a.capture_root.resolve()).encode()).hexdigest()[:16]+'.jsonl')
    dashboard=Dashboard(a.capture_root,a.manifest,a.suite,None if a.once else journal)
    if a.once:
        print(json.dumps(dashboard.payload(),ensure_ascii=False,indent=2));return
    def sample():
        while True:
            try:dashboard.payload()
            except Exception:pass # HTTP retries expose read errors; never control MR
            time.sleep(15)
    def sample_disk():
        while True:
            dashboard.sample_storage()
            time.sleep(300)
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path=='/': body=HTML.encode();mime='text/html; charset=utf-8'
            elif self.path=='/status.json':
                try:
                    status=dashboard.payload()
                    body=json.dumps(status,ensure_ascii=False).encode()
                except (OSError,ValueError,KeyError) as exc:
                    self.send_error(503,'Status temporarily unavailable');return
                mime='application/json'
            else: self.send_error(404);return
            self.send_response(200);self.send_header('Content-Type',mime);self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(body)
        def log_message(self,*args): pass
    print(f'http://127.0.0.1:{a.port}',flush=True)
    server=ThreadingHTTPServer(('127.0.0.1',a.port),Handler)
    threading.Thread(target=sample,daemon=True).start()
    threading.Thread(target=sample_disk,daemon=True).start()
    try:server.serve_forever()
    finally:server.server_close()


if __name__=='__main__':main()
