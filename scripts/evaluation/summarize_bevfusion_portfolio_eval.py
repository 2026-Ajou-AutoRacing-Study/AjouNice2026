#!/usr/bin/env python3
"""Summarize fresh same-device measurements; never splice historical timings."""
import argparse
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'perception/detection3d/bevfusion/tools'),
             str(ROOT/'perception/detection3d/bevfusion/scripts')]
from qualify_base5_camera_tensorrt import distribution,compare_detections,aggregate_comparisons,ros_delivery
from compare_base5_common30 import validate_payload


def load(path):
    with gzip.open(path,'rt') as f:return json.load(f)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();out=a.output
    import numpy as np
    comparison=json.loads((out/'common30_metrics/comparison.json').read_text())
    classes=('vehicle','pedestrian','obstacle')
    ap={}
    for name,model in comparison['models'].items():
        metrics=model['radial30']['metrics']
        scores={c:float(metrics[f'center/{c}_mean_AP'])*100 for c in classes}
        ap[name]=dict(scores,mAP=float(np.mean(list(scores.values()))))
    metric_path=ROOT/'perception/detection3d/common/evaluation/morai_detection.py'
    spec=importlib.util.spec_from_file_location('_portfolio_native_metric',metric_path)
    metric=importlib.util.module_from_spec(spec);sys.modules[spec.name]=metric;spec.loader.exec_module(metric)
    full_ap={}
    for backend in ('fp32','amp','trt_head'):
        payload=load(out/f'full_e12_{backend}/predictions.json.gz');validate_payload(payload)
        rows=[]
        for r in payload['rows']:
            row={k:np.asarray(v) if k.startswith(('gt_','pred_')) else v for k,v in r.items()}
            # JSON [] loses its (0, 7) box shape. Keep empty-GT/prediction
            # frames in the metric instead of deleting negative examples.
            for key in ('gt_boxes','pred_boxes'):
                row[key]=row[key].reshape(-1,7)
            rows.append(row)
        m=metric._evaluate_center(rows,list(classes),metric.DEFAULT_CENTER_THRESHOLDS,
            2.,.1,.1,'center',include_tp=False,include_operating_points=False)
        values={c:float(m[f'center/{c}_mean_AP'])*100 for c in classes}
        full_ap[backend]=dict(values,mAP=float(np.mean(list(values.values()))))
    timings={};parity={};failures={}
    reference=None;reference_name=None
    for case in ('original','cache','separated','parallel','trt','trt_head'):
        reports=[];raw=[]
        for round_index in (0,1):
            folder=out/f'timing_r{round_index}_{case}'
            if (folder/'FAILED.json').exists():
                failures[str(folder.name)]=json.loads((folder/'FAILED.json').read_text());continue
            report=json.loads((folder/'RESULT.json').read_text())
            if not report['complete']:raise ValueError('Incomplete timing case')
            reports.append(report);raw.extend(report['raw_times'])
        if not reports:continue
        timings[case]=dict(service_ms=distribution([r['service_ms'] for r in raw]),
            model_ms=distribution([r['model_ms'] for r in raw]),
            peak_allocated_bytes=max(r['peak_allocated_bytes'] for r in reports),
            peak_reserved_bytes=max(r['peak_reserved_bytes'] for r in reports),
            rounds=[dict(service_ms=r['timings']['service_ms'],pool_state=r['pool_state']) for r in reports])
        pred_path=out/f'timing_r0_{case}/predictions.json.gz'
        if not pred_path.exists():continue
        candidate=load(pred_path)
        if reference is None:
            reference=candidate;reference_name=case
        parity[case]={scope:aggregate_comparisons([compare_detections(transform(reference[key]),
            transform(candidate[key])) for key in reference]) for scope,transform in (
                ('raw',lambda x:x),('ros_delivery',lambda x:ros_delivery(x,True)))}
    inventory=json.loads((out/'INVENTORY.json').read_text())
    pins=json.loads((out/'input_sha256.json').read_text())
    changed=[]
    for filename,expected in pins.items():
        h=hashlib.sha256()
        with Path(filename).open('rb') as f:
            for data in iter(lambda:f.read(2**20),b''):h.update(data)
        if h.hexdigest()!=expected:changed.append(filename)
    if changed:raise ValueError('Sensor input changed during campaign: '+str(changed[:3]))
    result=dict(complete=True,inventory=inventory,common30_ap_percent=ap,
        e12_full_roi_ap_percent=full_ap,timings=timings,parity=parity,
        parity_reference_case=reference_name,failures=failures,
        all_sensor_hashes_unchanged=True,validation_is_new_test_set=False,
        caveats=['Existing validation, not independent test or causal data-only ablation.',
            'E25 native FP32 vs E12 native FP32 for model comparison; common radial <30m.',
            'E12 native AMP vs TRT/head AMP for accuracy preservation; same GPU and GT.',
            'Runtime service: in-memory JPEG decode to CPU boxes, not sensor/ROS/LAN delay.',
            'Desktop GPU processes remain; forward/reverse rounds expose but do not eliminate noise.',
            'Pooling originals are restored on one instance; no production source/default changed.'])
    (out/'SUMMARY.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    lines=['# 5060 Ti 재평가 결과','',
        '동일 GPU에서 다시 실행한 결과다. 과거 4090 수치와 합산하지 않는다.',
        '기존 validation의 재평가이며 독립 test set 또는 데이터만의 인과효과 검증이 아니다.','',
        '## 공통 30m native 검출 성능','',
        '| 모델 | Vehicle AP | Pedestrian AP | Obstacle AP | mAP |',
        '|---|---:|---:|---:|---:|']
    for name,r in ap.items():lines.append('| '+name+' | '+' | '.join(f'{r[k]:.4f}' for k in (*classes,'mAP'))+' |')
    lines+=['','## E12 전체 ROI 검증셋 AP','',
        '| 실행 | Vehicle AP | Pedestrian AP | Obstacle AP | mAP |','|---|---:|---:|---:|---:|']
    for name,r in full_ap.items():lines.append('| '+name+' | '+' | '.join(f'{r[k]:.4f}' for k in (*classes,'mAP'))+' |')
    lines+=['','## 동일 E12 속도 ablation','',
        '96개 고유 validation 입력, 각 순서에서 2회 반복, 순방향/역방향 별도 프로세스다.',
        'JPEG 메모리 입력부터 CPU raw box 반환까지이며 디스크·네트워크·ROS는 제외한다.','',
        '| 단계 | 서비스 평균 ms | p95 ms | 모델 평균 ms | peak allocated MiB |',
        '|---|---:|---:|---:|---:|']
    for name,r in timings.items():lines.append(f"| {name} | {r['service_ms']['mean']:.3f} | {r['service_ms']['p95']:.3f} | {r['model_ms']['mean']:.3f} | {r['peak_allocated_bytes']/2**20:.1f} |")
    if failures:lines+=['','OOM/실패 단계는 SUMMARY.json에 보존했으며 더 작은 해상도로 대체하지 않았다.']
    lines+=['','## 출력 대조','',
        '96개 입력의 예측끼리 대조한 수치 일치 검사이며 GT 정확도 평가는 위 AP 표다.',
        '최적화는 수치적으로 완전히 동일하지 않으며 strict 실패를 AP 저하와 혼동하지 않는다.',
        '기준: '+str(reference_name)+'; 허용치 XY/Z/크기 0.05m, yaw 0.02rad, score 0.002.','',
        '| 단계 | raw strict | ROS 전달 후 strict |','|---|---|---|']
    for name,r in parity.items():
        lines.append(f"| {name} | {r['raw']['engineering_passed']} | {r['ros_delivery']['engineering_passed']} |")
    lines+=['','## 범위와 한계','',*['- '+v for v in result['caveats']]]
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        names=list(timings);x=np.arange(len(names))
        fig,ax=plt.subplots(figsize=(10,4.5))
        ax.bar(x,[timings[n]['service_ms']['mean'] for n in names],label='Service mean')
        ax.scatter(x,[timings[n]['service_ms']['p95'] for n in names],color='black',label='Service p95')
        ax.set_xticks(x);ax.set_xticklabels(names,rotation=15);ax.set_ylabel('ms');ax.set_title('RTX 5060 Ti: fixed E12 runtime ablation')
        ax.legend();fig.tight_layout();fig.savefig(out/'runtime_ablation.png',dpi=160);plt.close(fig)
        names=list(ap);x=np.arange(len(names));fig,ax=plt.subplots(figsize=(9,4.5))
        for i,c in enumerate((*classes,'mAP')):
            ax.bar(x+(i-1.5)*.18,[ap[n][c] for n in names],width=.18,label=c)
        ax.set_xticks(x);ax.set_xticklabels(names);ax.set_ylim(0,100);ax.set_ylabel('AP (%)');ax.set_title('Same validation / common radial <30m')
        ax.legend(loc='upper left',bbox_to_anchor=(1.02,1.));fig.tight_layout();fig.savefig(out/'accuracy_common30.png',dpi=160);plt.close(fig)
    except ImportError:
        pass
    print(json.dumps(dict(complete=True,ap=ap,full_ap=full_ap,timing_cases=len(timings),failures=len(failures))),flush=True)


if __name__=='__main__':main()
