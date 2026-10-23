#!/usr/bin/env python3
"""정상 종료된 마지막 정적 동물 한 장을 읽기 쉬운 거리에서 보충한다.

4,999개 승인 선택과 소진한 1~3회 이력을 보존하고 새 출력의 4~5회만 허용한다.
obstacle 패밀리·미션·train·animal 의미군·정적·원형 및 native QA를 유지한다.
추가 완화는 목적 거리 구간뿐이며 새 장면을 5~30m에 배치한다.
"""
import sys
sys.dont_write_bytecode = True

from contextlib import contextmanager
import copy
from pathlib import Path

import flexible_complete as flex

POLICY = 'terminal-static-animal-readable-distance-cell-v1'
RELAXED_FIELDS = ['purpose_range_bucket']


def same_family_readable_animal(source, candidate, pools):
    a, b = source['spec'], candidate['spec']
    aa, bb = source['runtime_actors'], candidate['runtime_actors']
    return (a['family'] == b['family'] == 'obstacle'
        and bool(a['mission']) and bool(b['mission'])
        and source['split'] == candidate['split'] == 'train'
        and bool(aa) and bool(bb)
        and pools.get(aa[0]['data_id']) == pools.get(bb[0]['data_id']) == 'animal'
        and not flex.moving_modes(source) and not flex.moving_modes(candidate)
        and 5 <= flex.primary_range(candidate) <= 30
        and not any(actor.get('asset_scale') is not None for actor in aa + bb))


def readable_donors(source, donors, pools, ident, ordinal, used):
    options = [(entry, meta) for entry, meta in donors
        if same_family_readable_animal(source, meta, pools)
        and entry['scenario_sha256'] not in used]
    options.sort(key=lambda item: (
        len(item[1]['runtime_actors']),
        not 10 <= flex.primary_range(item[1]) <= 20,
        abs(flex.primary_range(item[1]) - 15), item[0]['id']))
    return options


@contextmanager
def installed(args):
    parent = flex.common.physical(args.parent)
    old = flex.read(parent/'state.json')
    exact_parent = Path(old['contract']['parent'])
    root = flex.summary(args.profile, exact_parent, args.output)
    doc, baseline, _, output, deck, prefix, deficits, pins = root
    status, quota = flex.read(parent/'status.json'), flex.read(parent/'quota.json')
    flex.require(old['contract']['policy'] == flex.POLICY
        and old['contract']['adapter_sha256'] == flex.common.sha(flex.__file__)
        and old['contract']['source_sha256'] == doc['manifest_sha256']
        and old['stage'] == status['stage'] == 'complete_with_deficit'
        and status.get('error') is None and old.get('error') is None
        and status.get('failed') == 0
        and not status.get('selected_not_certified') and not old.get('selected_not_certified')
        and status.get('current') is None and old.get('current') is None
        and old.get('current_child') is None and quota.get('pending') == 0,
        'Only a safe terminal, unchanged flexible parent may supply the last animal cell')
    flex.require(all(flex.common.sha(path) == digest
        for path, digest in old['contract']['parent_pins'].items())
        and all(flex.common.sha(path) == digest
            for path, digest in old.get('import_receipt', {}).get('pins', {}).items()),
        'Previous source/import snapshot changed')
    # Small contracts and attempted-child evidence only; never replay 4,999
    # old native QA or scan their RGB/LiDAR payloads for this one missing slot.
    flex.validate_resume(old, parent, deck, deficits, prefix)
    selected, missing = flex.merge(old, prefix)
    flex.require(len(selected) == status['accepted_candidate'] == quota['accepted'] == 4999
        and len(missing) == quota['deficit'] == status['review'] == 1
        and missing == [row['id'] for row in quota['cells']],
        'Exactly one authoritative deficit and 4,999 preserved selections required')
    entry = next(entry for entry in deck['scenes'] if entry['id'] == missing[0])
    original = flex.read(entry['metadata'])
    generator = flex.common.modules()[0].batch.campaign.loop.gen
    with flex.common.route_compatibility(doc):
        context = generator.context(generator.geo.DEFAULT_RUNTIME_ASSET_DIR, generator.geo.DEFAULT_MGEO)
    animal_ids = {asset['data_id'] for asset in context['pools']['animal']}
    actors = original['runtime_actors']
    flex.require(original['spec']['family'] == 'obstacle'
        and bool(original['spec']['mission']) and original['split'] == 'train'
        and len(actors) == 1 and actors[0]['data_id'] in animal_ids
        and not flex.moving_modes(original)
        and actors[0].get('asset_scale') is None
        and flex.range_purpose(original) == 'middle_30_80m',
        'Exception applies only to one original middle-range static mission animal')
    history = [copy.deepcopy(attempt) for attempt in old['attempts']
               if attempt['id'] == missing[0]]
    history += [copy.deepcopy(attempt)
        for attempt in old.get('import_receipt', {}).get('history', [])
        if attempt['id'] == missing[0]]
    flex.require(len(history) == 3 and {attempt['ordinal'] for attempt in history} == {1, 2, 3}
        and all(attempt.get('unavailable') and attempt.get('manifest') is None
                and attempt.get('result') is None for attempt in history),
        'Preserve three consumed CPU-only unavailable attempts; never bypass native failure')
    pins.update({str(parent/name): flex.common.sha(parent/name) for name in
        ('state.json', 'status.json', 'quota.json', 'selected_ledger.jsonl', 'adopted_manifest.json')})
    pins[str(Path(__file__).absolute())] = flex.common.sha(__file__)
    known = flex.exact.prior_payloads(parent, dict(scenes=[]))
    receipt = dict(selected=[], history=history, pins=dict(pins), known_payloads=sorted(known),
        policy=POLICY, existing_native_qa_recomputed=False, approved_prefix_preserved=4999,
        user_approved_extra_attempt_ordinals=[4, 5], relaxed_fields=list(RELAXED_FIELDS),
        inherited_relaxed_fields=list(flex.RELAXED), preserved_primary_semantic_pool='animal')
    saved = (flex.summary, flex.import_completion, flex.compatible,
             flex.donor_order, flex.flexible_rebinding)

    def summary(*unused):
        return doc, baseline, parent, output, deck, selected, missing, dict(pins)

    def imported(sources, *unused):
        flex.require(not sources, 'Trusted complete prefix is not a nested-import operation')
        return copy.deepcopy(receipt)

    @contextmanager
    def rebinding(semantic_pools):
        with saved[4](semantic_pools):
            import base5_layout_split as layout
            register = layout.register_recovery_overlay

            def declared(source_entry, source_meta, candidate, proposed, target):
                proposed['terminal_animal_completion_exception'] = dict(policy=POLICY,
                    wrapper_sha256=flex.common.sha(__file__), parent=str(parent),
                    parent_selected_sha256=flex.common.sha(parent/'selected_ledger.jsonl'),
                    relaxed_fields=list(RELAXED_FIELDS), preserved_primary_semantic_pool='animal',
                    readable_distance_m=[5, 30], native_qa_unchanged=True)
                return register(source_entry, source_meta, candidate, proposed, target)

            layout.register_recovery_overlay = declared
            try:
                yield
            finally:
                flex.require(layout.register_recovery_overlay is declared,
                             'Terminal-animal adapter ownership changed')
                layout.register_recovery_overlay = register

    flex.summary, flex.import_completion = summary, imported
    flex.compatible, flex.donor_order, flex.flexible_rebinding = (
        same_family_readable_animal, readable_donors, rebinding)
    try:
        yield
    finally:
        flex.summary, flex.import_completion, flex.compatible, flex.donor_order, flex.flexible_rebinding = saved


def execute(args):
    flex.require(not args.import_native_completion and args.attempts == 5,
        'Preserve three historical attempts and authorize at most two additional candidates')
    with installed(args):
        return flex.execute(args)


def parser():
    result = flex.parser()
    result.description = __doc__
    result.set_defaults(attempts=5, time_budget_sec=600)
    return result


if __name__ == '__main__':
    import json
    try:
        print(json.dumps(execute(parser().parse_args()), ensure_ascii=False), flush=True)
    except (OSError, ValueError, RuntimeError) as error:
        print(json.dumps(dict(event='terminal_animal_cell_blocked', error=str(error),
            fallback_allowed=False), ensure_ascii=False), file=sys.stderr)
        raise SystemExit(1)
