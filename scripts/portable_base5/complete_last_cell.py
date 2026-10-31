#!/usr/bin/env python3
"""사용자가 승인한 마지막 한 슬롯만 별도 출력에서 마감한다.

4999개 확정 선택을 그대로 보존하고 shadow/static/split은 유지한다.
이번 한 장만 primary 의미군·거리·음영 phase의 미세 quota를 풀며, 새 native
QA·GT·통신·충돌·Park 검사는 원형대로 실행한다. 기본은 읽기 전용 계획이다.
"""
import sys
sys.dont_write_bytecode = True
import argparse
from contextlib import contextmanager
import copy
from pathlib import Path

import flexible_complete as flex

POLICY = 'user-approved-last-shadow-cell-readable-static-v1'


def same_family_readable(source, candidate, pools):
    a, b = source['spec'], candidate['spec']
    return (a['family'] == b['family'] == 'shadow'
        and bool(a['mission']) == bool(b['mission'])
        and source['split'] == candidate['split']
        and not flex.moving_modes(source) and not flex.moving_modes(candidate)
        and bool(candidate['runtime_actors'])
        and 5 <= flex.primary_range(candidate) <= 30
        and not any(x.get('asset_scale') is not None for x in candidate['runtime_actors']))


def easy_donors(source, donors, pools, ident, ordinal, used):
    preferred = ('2017_Kia_Niro(HEV)', '2017_Kia_Sorento', '2020_volvo_xc90',
                 '2021_Kia_Carnival', 'concrete-barrier')
    options = [(entry, meta) for entry, meta in donors
        if same_family_readable(source, meta, pools)
        and entry['scenario_sha256'] not in used
        and meta['spec'].get('shadow_phase') == 'interior'
        and meta['spec']['role'] == 'front'
        and len(meta['runtime_actors']) == 1
        and meta['runtime_actors'][0]['name'] in preferred]
    options.sort(key=lambda item: (preferred.index(item[1]['runtime_actors'][0]['name']),
                                  abs(flex.primary_range(item[1])-15), item[0]['id']))
    return options


@contextmanager
def installed(args):
    parent = flex.common.physical(args.parent)
    old = flex.read(parent/'state.json')
    exact_parent = Path(old['contract']['parent'])
    original_summary = flex.summary
    root = original_summary(args.profile, exact_parent, args.output)
    doc, baseline, _, output, deck, prefix, deficits, pins = root
    status, quota = flex.read(parent/'status.json'), flex.read(parent/'quota.json')
    flex.require(old['contract']['policy'] == flex.POLICY
        and old['contract']['adapter_sha256'] == flex.common.sha(flex.__file__)
        and old['contract']['source_sha256'] == doc['manifest_sha256']
        and old['stage'] == status['stage'] == 'complete_with_deficit'
        and status.get('error') is None and status.get('failed') == 0
        and not status.get('selected_not_certified') and status.get('current') is None
        and old.get('current') is None and quota.get('pending') == 0,
        'Only a safe terminal, unchanged flexible parent may supply the last cell')
    flex.require(all(flex.common.sha(p) == h for p,h in old['contract']['parent_pins'].items())
        and all(flex.common.sha(p) == h for p,h in old.get('import_receipt',{}).get('pins',{}).items()),
        'Previous source/import snapshot changed')
    # Small contracts, saved selections and new attempt hashes only. No replay
    # of the 4999 old native QA or RGB/LiDAR bitrot scan.
    flex.validate_resume(old, parent, deck, deficits, prefix)
    selected, missing = flex.merge(old, prefix)
    flex.require(len(selected) == status['accepted_candidate'] == quota['accepted'] == 4999
        and len(missing) == quota['deficit'] == status['review'] == 1
        and missing == [r['id'] for r in quota['cells']], 'Exactly one authoritative deficit required')
    entry = next(e for e in deck['scenes'] if e['id'] == missing[0])
    original = flex.read(entry['metadata'])
    flex.require(original['spec']['family'] == 'shadow' and not flex.moving_modes(original),
                 'This explicit exception is for one static shadow slot only')
    history = [copy.deepcopy(a) for a in old['attempts'] if a['id'] == missing[0]]
    history += [copy.deepcopy(a) for a in old.get('import_receipt',{}).get('history',[])
                if a['id'] == missing[0]]
    flex.require({a['ordinal'] for a in history} == {1,2,3}, 'Three prior consumed attempts must remain recorded')
    pins.update({str(parent/name):flex.common.sha(parent/name) for name in
                 ('state.json','status.json','quota.json','selected_ledger.jsonl','adopted_manifest.json')})
    pins[str(Path(__file__).absolute())] = flex.common.sha(__file__)
    known = flex.exact.prior_payloads(parent,dict(scenes=[]))
    receipt = dict(selected=[],history=history,pins=dict(pins),known_payloads=sorted(known),
        policy=POLICY,existing_native_qa_recomputed=False,approved_prefix_preserved=4999,
        user_approved_extra_attempt_ordinals=[4,5],
        relaxed_fields=['primary_semantic_pool','purpose_range_bucket','shadow_phase'])
    saved = (flex.summary, flex.import_completion, flex.compatible,
             flex.donor_order, flex.flexible_rebinding)
    def summary(*unused):
        return doc,baseline,parent,output,deck,selected,missing,dict(pins)
    def imported(sources,*unused):
        flex.require(not sources, 'The trusted complete prefix is not a nested-import operation')
        return copy.deepcopy(receipt)
    @contextmanager
    def rebinding(pools):
        with saved[4](pools):
            import base5_layout_split as layout
            register = layout.register_recovery_overlay
            def declared(source_entry, source_meta, candidate, proposed, target):
                proposed['last_cell_completion_exception'] = dict(policy=POLICY,
                    wrapper_sha256=flex.common.sha(__file__),parent=str(parent),
                    parent_selected_sha256=flex.common.sha(parent/'selected_ledger.jsonl'),
                    relaxed_fields=receipt['relaxed_fields'],native_qa_unchanged=True)
                return register(source_entry,source_meta,candidate,proposed,target)
            layout.register_recovery_overlay = declared
            try:
                yield
            finally:
                flex.require(layout.register_recovery_overlay is declared, 'Last-cell adapter ownership changed')
                layout.register_recovery_overlay = register
    flex.summary, flex.import_completion = summary, imported
    flex.compatible, flex.donor_order, flex.flexible_rebinding = same_family_readable, easy_donors, rebinding
    try:
        yield
    finally:
        flex.summary, flex.import_completion, flex.compatible, flex.donor_order, flex.flexible_rebinding = saved


def execute(args):
    flex.require(not args.import_native_completion and args.attempts == 5,
                 'Preserve three historical attempts and authorize at most two additional candidates')
    with installed(args):
        return flex.execute(args)


if __name__ == '__main__':
    import json
    p = flex.parser()
    p.description = __doc__
    p.set_defaults(attempts=5,time_budget_sec=600)
    try:
        print(json.dumps(execute(p.parse_args()),ensure_ascii=False),flush=True)
    except (OSError,ValueError,RuntimeError) as error:
        print(json.dumps(dict(event='last_cell_blocked',error=str(error),fallback_allowed=False),
                         ensure_ascii=False),file=sys.stderr)
        raise SystemExit(1)
