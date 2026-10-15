#!/usr/bin/env python3
"""Current-code ground re-audit; preserves capture ledgers and raw annotations."""
import argparse
import json
from collections import Counter
from pathlib import Path
from base5_capture_audit import audit_capture
from base5_capture_safety import atomic_json


def report(roots):
    rows=[];failed=[];shutdown=[]
    ground_reasons={'actor_ground_plane_unverified','actor_annotation_above_ground','actor_annotation_deep_below_ground'}
    for root in roots:
        ledger=[json.loads(l) for l in (root/'ledger.jsonl').read_text().splitlines() if l]
        failed.extend(dict(capture_root_session=str(root),**r) for r in ledger if r.get('status')=='failed')
        latest={r['id']:r for r in ledger}
        if (root/'shutdown_safety.json').exists():shutdown.append(dict(root=str(root),**json.loads((root/'shutdown_safety.json').read_text())))
        for result in latest.values():
            if not result.get('capture_root'):continue
            cap=Path(result['capture_root']);meta=json.loads((cap/'metadata.json').read_text())
            current=audit_capture(cap,meta)
            reasons=set(result['audit']['review_reasons'])-ground_reasons
            reasons.update(current['review_reasons'])
            before=None
            if meta.get('ground_retry_provenance'):
                source=Path(meta['ground_retry_provenance']['source_capture'])
                source_meta=json.loads((source/'metadata.json').read_text())
                source_meta['ground_support_check']=dict(enabled=True,max_range_m=20.,maximum_gap_m=.15)
                before=audit_capture(source,source_meta)['ground_support']
            rows.append(dict(id=result['id'],root=str(root),capture_root=str(cap),original_status=result['status'],
                current_accepted_candidate=not reasons,current_review_reasons=sorted(reasons),
                collision_clear=result['audit']['capture_safety']['collision']['no_collision_verified'],
                vertical_motion_actors=result['audit'].get('vertical_motion_actors',[]),
                runtime_actors=[dict(uid=a['unique_id'],asset=a['name'],motion=a['motion']) for a in meta['runtime_actors']],
                ground=current['ground_support'],before_ground=before))
    return dict(rows=rows,failed_attempts=failed,shutdown=shutdown,
        summary=dict(raw_captures=len(rows),current_accepted_candidate=sum(r['current_accepted_candidate'] for r in rows),
            review_reasons=dict(Counter(k for r in rows for k in r['current_review_reasons']))),
        limitation='Current-code re-audit, not original ledger replacement. Lowest annotation corner against local fitted plane is not collider/mesh contact proof; unmeasured far actors not certified.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture-root',type=Path,action='append',required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError('Report overwrite refused')
    r=report(a.capture_root);atomic_json(a.output,r);print(json.dumps(r['summary'],indent=2))
