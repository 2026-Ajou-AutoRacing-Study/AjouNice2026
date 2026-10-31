#!/usr/bin/env python3
"""Bounded Base5 quality recovery: capture -> Z/probe/replacement -> recheck."""
import argparse
import copy
import fcntl
import json
from pathlib import Path
import subprocess
import sys
import time
import generate_base5_independent as gen
from base5_capture_safety import atomic_json,sha256,prepare_session,load_ledger,is_collision_failure,validate_collision_failure,is_stationary_failure,validate_stationary_failure
from prepare_base5_ground_retry import prepare as height_retry,prepare_from_near as near_height_retry
from prepare_base5_empty_ground_probe import prepare as empty_probe
from prepare_base5_probed_retry import prepare as probed_retry
from prepare_base5_setup_budget_pilot import prepare as budget_retry
import base5_motion_ground as motion_ground
from prepare_base5_static_height_reference import prepare as static_height_reference
from base5_static_ground_reference import prepare as static_reference_retry,reaudit_original,COMPOSITE_ANCHORS
from base5_static_ground_policy import lightweight_static
from base5_intersection_placement import ready as editor_prepared
import base5_shadow_ground_reuse as shadow_ground_reuse


def scene_rejection(row):
    if is_collision_failure(row):validate_collision_failure(row);return 'physical_collision'
    if is_stationary_failure(row):validate_stationary_failure(row);return 'ego_stationary_support'
    return None


def arrival_retry_eligible(result,manifest):
    if result.get('status')!='failed' or result.get('error')!='Natural target arrival not observed for all moving actors; no capture':
        return False
    if gen.geo.load_json(manifest)['config']['capture'].get('natural_setup_budget_sec',9)>=9:return False
    deck=gen.geo.load_json(manifest)
    if any(a.get('motion_path',{}).get('ground_path_evidence') for s in deck.get('scenes',[]) if s.get('metadata')
           for a in gen.geo.load_json(Path(s['metadata']))['runtime_actors']):return False
    safety=result.get('no_capture_safety',{})
    return (all(safety.get(k) is True for k in ('no_capture_requested','topic_freshness_verified',
        'actor_presence_verified','ego_stationary_verified','actor_z_steps_verified'))
        and safety.get('collision',{}).get('no_collision_verified') is True)


def validate_miss_evidence(row,manifest):
    if not arrival_retry_eligible(row,manifest):raise ValueError('Unsafe/non-budget target failure')
    path=Path(row['failure_evidence'])
    if sha256(path)!=row['failure_evidence_sha256']:raise ValueError('Target miss evidence changed')
    data=gen.geo.load_json(path)
    if (data['scenario_sha256']!=row['scenario_sha256'] or data['no_capture_safety']!=row['no_capture_safety']
            or data['reason']!='natural_targets_not_simultaneously_observed'):
        raise ValueError('Target miss evidence does not match ledger')


class PlacementProposalExhausted(ValueError):
    """No new MORAI load: bounded offline geometry search could not fill a cell."""


class ReplacementContractError(RuntimeError):
    """Frozen unit/layout provenance failed; never an ordinary geometry deficit."""


class ReplacementGeometryRejected(PlacementProposalExhausted):
    """A valid frozen source produced a layout/coverage-ineligible proposal."""


class NativePayloadRepeated(ReplacementContractError):
    """No load/capture: this cell proposed an already observed native payload."""


def payload_set(values):
    try:
        result=set(values or ())
    except TypeError as error:
        raise ReplacementContractError('Native nonrepeat SHA inventory is malformed') from error
    if any(not isinstance(value,str) or len(value)!=64
            or any(c not in '0123456789abcdef' for c in value) for value in result):
        raise ReplacementContractError('Native nonrepeat requires actual SHA256 values')
    return result


def native_payloads(attempts):
    """Actual native exports only; a verified pre-capture target miss is not one.

    This is deliberately separate from the existing far-only load-history API.
    Completed children and raw re-audits are never recaptured by this helper.
    """
    known=set()
    try:
        for attempt in attempts:
            if not attempt.get('capture_root') or attempt.get('kind')=='static_rigid_raw_reaudit':continue
            root=Path(attempt['capture_root'])
            if gen.geo.load_json(root/'shutdown_safety.json').get('park_verified') is not True:
                raise ReplacementContractError('Native nonrepeat requires verified prior Park')
            content=(root/'ledger.jsonl').read_bytes()
            if not content or not content.endswith(b'\n'):
                raise ReplacementContractError('Native nonrepeat requires a complete retained ledger')
            for row in (json.loads(line) for line in content.splitlines()):
                if row.get('capture_root'):
                    raw=Path(row['capture_root']);digest=row['scenario_sha256']
                    if (sha256(raw/'scenario.json')!=digest
                            or len(list(raw.glob('LIDAR_*/*_Instance.txt')))!=1):
                        raise ReplacementContractError('Retained native payload/export changed or missing')
                    known.update(payload_set([digest]));continue
                if is_collision_failure(row):validate_collision_failure(row);continue
                if is_stationary_failure(row):validate_stationary_failure(row);continue
                safety=row.get('no_capture_safety',{})
                if (row.get('status')!='failed'
                        or row.get('error')!='Natural target arrival not observed for all moving actors; no capture'
                        or not all(safety.get(k) is True for k in ('no_capture_requested','topic_freshness_verified',
                            'actor_presence_verified','ego_stationary_verified','actor_z_steps_verified'))
                        or safety.get('collision',{}).get('no_collision_verified') is not True):
                    raise ReplacementContractError('Missing raw is not a verified no-capture target miss')
                proof=Path(row['failure_evidence']);data=gen.geo.load_json(proof)
                if (sha256(proof)!=row['failure_evidence_sha256'] or data.get('scenario_sha256')!=row['scenario_sha256']
                        or data.get('no_capture_safety')!=safety
                        or data.get('reason')!='natural_targets_not_simultaneously_observed'):
                    raise ReplacementContractError('No-capture target evidence changed')
        return known
    except ReplacementContractError:
        raise
    except (OSError,ValueError,TypeError,KeyError) as error:
        raise ReplacementContractError('Retained native nonrepeat evidence invalid: '+str(error)) from error


def replacement_contract(deck, entry, metadata):
    """Validate opt-in parent contracts before generation, including nested roots."""
    import base5_unit_geometry_contract as unit
    import base5_layout_split as layout
    expected = deck.get('config', {}).get(unit.KEY)
    if metadata.get(unit.KEY) != expected:
        raise ReplacementContractError('Replacement source/config unit reference differs')
    if expected is not None:
        unit.validate_actor_identities(metadata, unit.load(expected))
    declared = deck.get('split_overlay')
    tagged = metadata.get('training_split_overlay')
    if (declared is None) != (tagged is None):
        raise ReplacementContractError('Replacement source/manifest layout reference differs')
    if declared is None:
        return dict(unit_reference=expected, overlay=False)
    try:
        layout.validate_manifest_overlay(deck, allow_subset=True)
        overlay = layout.overlay_for_metadata(metadata)
        current_binding = layout.resolve(overlay, entry['id'], entry, metadata)
        proof = metadata.get('quality_replacement_contract')
        root = copy.deepcopy(proof['root_entry'] if proof else entry)
        root_meta = gen.geo.load_json(Path(root['metadata']))
        if proof and (sha256(root['metadata']) != proof['root_metadata_sha256']
                or sha256(root['scenario']) != root['scenario_sha256']):
            raise ReplacementContractError('Replacement canonical root changed')
        root_overlay = layout.overlay_for_metadata(root_meta)
        root_binding = layout.resolve(root_overlay, root['id'], root, root_meta)
        canonical = overlay.get('recovery_aliases', {}).get(entry['id'], entry['id'])
        if (canonical != root['id'] or root_meta.get(unit.KEY) != expected
                or current_binding['split'] != root_binding['split']
                or current_binding['group_id'] != root_binding['group_id']):
            raise ReplacementContractError('Replacement canonical split/group/unit lineage differs')
        layout.verify_source_pins(root_overlay)
        if proof and (type(proof.get('ordinal')) is not int or proof['ordinal'] < 1):
            raise ReplacementContractError('Replacement lineage ordinal differs')
        ordinal = (proof['ordinal'] + 1) if proof else 1
        if type(ordinal) is not int or ordinal < 1:
            raise ReplacementContractError('Replacement lineage ordinal differs')
        return dict(unit_reference=expected, overlay=True, root_entry=root,
            root_metadata=root_meta, root_metadata_sha256=sha256(root['metadata']), ordinal=ordinal)
    except ReplacementContractError:
        raise
    except (OSError, ValueError, TypeError, KeyError) as error:
        raise ReplacementContractError('Replacement frozen layout source invalid: '+str(error)) from error


def preserve_replacement_unit(metadata, proposed, context):
    import base5_unit_geometry_contract as unit
    expected = context['unit_reference']
    if expected is not None:
        if proposed.get(unit.KEY) not in (None, expected):
            raise ReplacementContractError('Generated replacement changed unit reference')
        proposed[unit.KEY] = copy.deepcopy(expected)
        unit.validate_actor_identities(proposed, unit.load(expected))
    elif unit.KEY in proposed:
        raise ReplacementContractError('Legacy replacement cannot silently gain unit QA')


def bind_replacement_contract(source_entry, original_metadata, candidate_entry,
                              proposed_metadata, candidate_deck, output_dir, *, context=None):
    """Bind a newly serialized training proposal to its immutable canonical root.

    Reusable by final ground/path preparation. No scenario, parent metadata,
    frozen overlay or raw capture is rewritten. Callers own the fresh candidate
    scenario/metadata paths and final manifest serialization.
    """
    frozen = context if context is not None else replacement_contract(candidate_deck,source_entry,original_metadata)
    new, old, m = proposed_metadata, candidate_entry, candidate_deck
    preserve_replacement_unit(original_metadata,new,frozen)
    if frozen['overlay']:
        new['fresh_completion_source']=dict(source_cell_id=frozen['root_entry']['id'],
            source_scenario_sha256=frozen['root_entry']['scenario_sha256'],ordinal=frozen['ordinal'],
            origin='bounded-quality-loop; separate fresh completion not invoked')
        prior=set(original_metadata.get('quality_replacement_contract',{}).get('prior_proposal_sha256',[]))
        prior.update((source_entry['scenario_sha256'],frozen['root_entry']['scenario_sha256']))
        new['quality_replacement_contract']=dict(root_entry=frozen['root_entry'],
            root_metadata_sha256=frozen['root_metadata_sha256'],ordinal=frozen['ordinal'],
            immediate_parent_entry=copy.deepcopy(source_entry),
            immediate_parent_metadata_sha256=sha256(source_entry['metadata']),prior_proposal_sha256=sorted(prior),
            actor_identity_map=[dict(ordinal=i,source_uid=a['unique_id'],candidate_uid=b['unique_id'],
                data_id=a['data_id'],source_list=a['source_list'],motion=a['motion'])
                for i,(a,b) in enumerate(zip(original_metadata['runtime_actors'],new['runtime_actors']))],
            layout_split_finalization_pending=True,raw_gt_or_parent_modified=False,separate_completion_invoked=False)
    atomic_json(Path(old['metadata']),new)
    if not frozen['overlay']:
        return None
    import base5_layout_split as layout
    old['source_cell_id']=old['id']
    try:
        reference=layout.register_recovery_overlay(frozen['root_entry'],frozen['root_metadata'],
            old,new,Path(output_dir)/'layout_split_rebind')
        new['training_split_overlay']=reference;atomic_json(Path(old['metadata']),new)
        m['split_overlay']=dict(reference,policy=layout.POLICY)
        m.update(layout_split_finalization_pending=True,bulk_ready=False)
        layout.validate_manifest_overlay(m,allow_subset=True)
        return reference
    except (OSError, ValueError, TypeError, KeyError) as error:
        if isinstance(error,ValueError) and str(error).startswith((
                'Recovery duplicates another actual layout; propose distinct geometry',
                'Recovery layout leakage/coverage changed:')):
            raise ReplacementGeometryRejected(str(error)) from error
        raise ReplacementContractError('Replacement layout rebind failed: '+str(error)) from error


def captured_payloads(attempts):
    """Read retained native session ledgers; never infer a failed pose from names.

    Callers first validate the session/Park contract. Recheck raw payload hashes
    here as well, including reviewed captures, so recovery cannot recycle them.
    """
    known=set()
    for attempt in attempts:
        if not attempt.get('capture_root') or attempt.get('kind')=='static_rigid_raw_reaudit':continue
        root=Path(attempt['capture_root']);ledger=root/'ledger.jsonl'
        if not ledger.exists():raise ValueError('Native attempt ledger missing for nonrepeat proof')
        raw_ledger=ledger.read_bytes()
        if raw_ledger and not raw_ledger.endswith(b'\n'):
            raise ValueError('Native attempt ledger is unterminated; nonrepeat proof cannot repair it')
        for row in (json.loads(line) for line in raw_ledger.splitlines()):
            digest=row['scenario_sha256'];raw=row.get('capture_root')
            if raw and sha256(Path(raw)/'scenario.json')!=digest:
                raise ValueError('Native attempted payload changed')
            known.add(digest)
    return known


def replacement(manifest,output,require_ground_observable=False,
                far_known_payloads=None,far_retry_block=0,known_payloads=None):
    m=gen.geo.load_json(manifest);old=m['scenes'][0];meta=gen.geo.load_json(Path(old['metadata']))
    source_entry=copy.deepcopy(old)
    frozen=replacement_contract(m,old,meta)
    far_plan='secondary_far_vehicle' in meta.get('spec',{})
    if type(far_retry_block) is not int or not 0<=far_retry_block<=15:
        raise ValueError('Far retry block must be an integer in 0..15')
    if not far_plan and (far_retry_block or far_known_payloads):
        raise ValueError('Far-only nonrepeat options require an explicit far plan')
    known=set(far_known_payloads or ())|payload_set(known_payloads)|{old['scenario_sha256']}
    if frozen['overlay']:
        known.update(meta.get('quality_replacement_contract',{}).get('prior_proposal_sha256',[]))
        known.add(frozen['root_entry']['scenario_sha256'])
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    if m['config'].get('obstacle_support_policy'):
        # Cache pure road/region indices on the shared context BEFORE the
        # builder's shallow static-geometry copy. No pose or QA gate changes.
        from run_base5_failure_aware_replenishment_fast import prewarm
        ctx=prewarm(ctx,m['config'])
    start=meta['retry']+1+4096*far_retry_block
    layout_attempts=[]
    proposal_counts=dict(geometry=0,generation_rejections=0,split_rejections=0,roster_rejections=0,
        known_payload_rejections=0,ground_witness_rejections=0)
    if frozen['overlay']:output.mkdir()
    def commit(scene,new,retry,directory):
        child=copy.deepcopy(m);candidate=child['scenes'][0];directory.mkdir()
        sid=old['id']+'_replace';sp=directory/(sid+'.json');mp=directory/(sid+'_metadata.json')
        provenance=dict(source_cell_id=old.get('source_cell_id',old['id']),source_scenario_sha256=old['scenario_sha256'],
            policy='same primary asset, family, mission, motion request, role, distance cell and split; no GT rewriting',
            ground_observation_preflight=require_ground_observable,
            ground_observation_limitation='Mapped-road witness pose availability only; actual terrain and visibility still required')
        if frozen['overlay'] or frozen['unit_reference'] is not None or known_payloads is not None:
            provenance.update(avoided_payload_sha256=sorted(known),offline_proposals=retry-start+1)
        if far_plan:
            new['secondary_far_vehicle_replacement']=provenance
            provenance.update(retry_block=far_retry_block,avoided_payload_sha256=sorted(known),offline_proposals=retry-start+1)
        else:new['replacement']=provenance
        for key in ('scene_group_id','environment_split_contract','logical_quota_slot_id',
                    'fresh_environment_provenance','obstacle_support_pair'):
            if key in meta:new[key]=copy.deepcopy(meta[key])
        if 'obstacle_support_pair' in new and not far_plan:
            new['obstacle_support_pair'].update(actual_retry=retry,ego_xy_delta_m=None,
                recovery_geometry_changed=True,pair_precision='same identity; replacement geometry, not exact pose pair')
        atomic_json(sp,scene);atomic_json(mp,new)
        candidate.update(id=sid,scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=sha256(sp),
            motion=new['runtime_actors'][0]['motion'] if new['runtime_actors'] else old.get('motion',meta['spec'].get('motion','static')))
        candidate['source_cell_id']=provenance['source_cell_id']
        bind_replacement_contract(source_entry,meta,candidate,new,child,directory,context=frozen)
        return child
    for retry in range(start,start+512):
        proposal_counts['geometry']+=1
        try:scene,new=gen.build(meta['spec'],m['config'],ctx,retry)
        except ValueError as exc:
            if 'secondary_far_vehicle' in meta.get('spec',{}):
                from base5_secondary_far_vehicle import FarGeometryExhausted
                if not isinstance(exc,FarGeometryExhausted):raise
            proposal_counts['generation_rejections']+=1;continue
        if new['split']!=meta['split']:
            proposal_counts['split_rejections']+=1;continue
        primary=lambda m:m['runtime_actors'][0]['data_id'] if m['runtime_actors'] else None
        if primary(new)!=primary(meta):raise ValueError('Replacement changed primary asset')
        if frozen['overlay']:
            roster=lambda value:[(a['data_id'],a['source_list'],a['motion'],
                (a.get('asset_scale') or {}).get('profile_id')) for a in value['runtime_actors']]
            if new['spec'] != meta['spec']:
                raise ReplacementContractError('Replacement changed frozen spec')
            if roster(new) != roster(meta):
                proposal_counts['roster_rejections']+=1;continue
        if frozen['overlay'] or frozen['unit_reference'] is not None or known_payloads is not None:
            import hashlib
            payload=hashlib.sha256((json.dumps(scene,ensure_ascii=False,indent=2)+'\n').encode()).hexdigest()
            if payload in known:
                proposal_counts['known_payload_rejections']+=1;continue
        if 'secondary_far_vehicle' in meta.get('spec',{}):
            from base5_secondary_far_vehicle import verify_preserved
            import hashlib
            verify_preserved(meta,new)
            payload=hashlib.sha256((json.dumps(scene,ensure_ascii=False,indent=2)+'\n').encode()).hexdigest()
            target=meta['spec']['secondary_far_vehicle']['target_ordinal']
            before=meta['runtime_actors'][target]['geometric_center_estimate']
            after=new['runtime_actors'][target]['geometric_center_estimate']
            import math
            if payload in known or math.dist(before,after)<.25:
                continue  # Never reload the same failed far-secondary pose.
        if require_ground_observable:
            from prepare_base5_empty_ground_probe import target_probe_pose
            # Animated preparation observes every target through empty probes
            # with the 8m road bound. Static-only rigid references allow 12m.
            # A 12m preflight for a mixed scene promised an unavailable witness.
            static_maximum=8. if motion_ground.moving_actors(new) else 12.
            try:
                for actor in new['runtime_actors']:
                    if actor['motion'] in ('static','standing'):
                        target_probe_pose(ctx['links'],actor,maximum_road_distance=static_maximum)
                    else:
                        target_probe_pose(ctx['links'],dict(actor,position=actor['motion_path']['target_position']))
            except ValueError:
                proposal_counts['ground_witness_rejections']+=1;continue
        if frozen['overlay']:
            directory=output/('proposal_'+str(retry).zfill(5))
            try:
                child=commit(scene,new,retry,directory)
            except ReplacementGeometryRejected as error:
                layout_attempts.append(dict(retry=retry,status='layout_geometry_rejected',reason=str(error)))
                atomic_json(output/'offline_proposals.json',dict(maximum_geometry_proposals=512,
                    maximum_layout_bindings=8,geometry_proposals=retry-start+1,layout_attempts=layout_attempts,
                    proposal_counts=proposal_counts,native_capture_started=False))
                if len(layout_attempts)>=8:break
                continue
            layout_attempts.append(dict(retry=retry,status='cpu_bound_not_native_approval'))
            atomic_json(output/'offline_proposals.json',dict(maximum_geometry_proposals=512,
                maximum_layout_bindings=8,geometry_proposals=retry-start+1,layout_attempts=layout_attempts,
                proposal_counts=proposal_counts,native_capture_started=False))
        else:child=commit(scene,new,retry,output)
        atomic_json(output/'manifest.json',child);return output/'manifest.json'
    if frozen['overlay']:
        atomic_json(output/'offline_proposals.json',dict(maximum_geometry_proposals=512,
            maximum_layout_bindings=8,layout_attempts=layout_attempts,proposal_counts=proposal_counts,
            status='bounded_offline_geometry_exhausted',native_capture_started=False))
    raise PlacementProposalExhausted('Same-cell/split replacement unavailable within 512 geometry proposals / 8 layout bindings')


def validate_seed(path,original):
    """Reuse only authoritative, safe same-payload batch review; never transplant raw."""
    seed=gen.geo.load_json(path);root=Path(seed['capture_root']);manifest=Path(seed['manifest'])
    safety=gen.geo.load_json(root/'shutdown_safety.json')
    if not safety.get('park_verified'):raise ValueError('Seed final Park unverified')
    session=gen.geo.load_json(root/'session_contract.json');repo=Path(__file__).resolve().parents[3]
    rows,_=prepare_session(root,manifest,
        repo/'configs/network/ego_network_mr_base5_savesensordata_collision_local.json',
        repo/'configs/sensors/Sensor_competition_local.json',session.get('runner_options'))
    if not session.get('runner_options',{}).get('shared_ros_batch'):
        raise ValueError('Seed must be a guarded shared-ROS batch')
    row=next((r for r in reversed(rows) if r['id']==seed['id']),None)
    if row is None or row['id']!=original['id'] or row['scenario_sha256']!=original['scenario_sha256']:
        raise ValueError('Seed is not the original quota payload')
    missed=row['status']=='failed'
    if missed:validate_miss_evidence(row,manifest)
    elif row['status']!='review':raise ValueError('Only reviewed seed may enter recovery')
    if any((r['status'] not in ('accepted_candidate','review') and r is not row)
            or not r.get('between_scene_park',{}).get('park_verified') for r in rows):
        raise ValueError('Seed batch unsafe or failed')
    if missed:return copy.deepcopy(row),root,safety
    capture=Path(row['capture_root'])
    if gen.geo.load_json(capture/'audit.json')!=row['audit']:raise ValueError('Seed audit differs from authoritative ledger')
    if gen.geo.load_json(capture/'metadata.json').get('calibration_only'):raise ValueError('Calibration seed forbidden')
    if 'physical_collision_during_capture_window' in row['audit'].get('review_reasons',[]):
        raise ValueError('Physical collision seed forbidden')
    return copy.deepcopy(row),root,safety


class StaticQualityBudget:
    """Bound expensive static work; missing evidence never becomes a pass."""
    def __init__(self,config):
        self.limits=config.get('capture',{}).get('static_quality_budget',{})
        self.counts={};self.witnessed=set();self.reason=None

    def spend(self,kind,uid=None):
        if not self.limits:return True
        key={'witness':'max_ground_witnesses','height':'max_height_recaptures',
            'replacement':'max_placement_recaptures'}[kind]
        if kind=='witness' and uid in self.witnessed:
            self.reason='static_ground_witness_already_attempted';return False
        if self.counts.get(kind,0)>=self.limits[key]:
            self.reason='static_quality_budget_exhausted_'+kind;return False
        self.counts[kind]=self.counts.get(kind,0)+1
        if kind=='witness':self.witnessed.add(uid)
        return True


def near_motion_probe_requests(metadata):
    """Existing target witnesses first; conditional departure view for tunnel paths."""
    targets=motion_ground.near_probe_actors(metadata)
    requests=[(actor,0.) for actor in targets]
    if metadata.get('spec',{}).get('shadow_moving'):
        for actor in targets:
            path=actor.get('motion_path')
            if path:
                first=path['points'][0];target=path['target_position']
                if gen.math.hypot(first['x']-target['x'],first['y']-target['y'])>.1:
                    # Complementary views fill disjoint bonnet/near-ray holes.
                    # The second departure view runs ONLY if prior proof fails.
                    requests.extend([(actor,16.),(actor,12.)])
    if len(requests)>8:raise ValueError('Near terrain witnesses bounded to eight views')
    return requests


def execute(source,indices,output,resume=False,stop_path=None,progress=None,seed_capture=None,
            far_known_payloads=None,known_payloads=None):
    started=time.monotonic()
    if output.exists() and not resume:raise FileExistsError('New quality-loop output required; use campaign resume')
    if not 1<=len(indices)<=4 or len(set(indices))!=len(indices):raise ValueError('Pilot limited to 1–4 cells')
    if seed_capture is not None and len(indices)!=1:raise ValueError('Seed recovery is single-cell only')
    output.mkdir(parents=True,exist_ok=True);lock=(output/'.quality.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    prior_elapsed=(gen.geo.load_json(output/'report.json').get('elapsed_sec',0.)
        if resume and (output/'report.json').exists() else 0.)
    source_data=gen.geo.load_json(source);attempts=[];selected=[];unresolved=[]
    inherited_native=payload_set(known_payloads)
    enforce_nonrepeat=bool(source_data.get('split_overlay')
        or source_data.get('config',{}).get('native_unit_geometry_reference') or known_payloads is not None)
    # Opt-in registrations are immutable provenance, not recoverable scene
    # deficits. Validate even before local completed children can be resumed.
    shadow_ground_reuse.validate_manifest(source_data)
    runner=Path(__file__).with_name('run_base5_capture.py')
    def stopped():return (output/'STOP').exists() or (stop_path is not None and stop_path.exists())
    def prepared(path,make):
        nonlocal preparation_context
        if path.exists():
            # Only complete, durable preparation decks are reusable.
            try:
                deck=gen.geo.load_json(path)
                for item in deck['scenes']:
                    if sha256(Path(item['scenario']))!=item['scenario_sha256']:raise ValueError('Prepared deck checksum mismatch')
                    gen.geo.load_json(Path(item['metadata']))
            except (OSError,ValueError,TypeError,KeyError) as error:
                if preparation_source.get('split_overlay'):
                    raise ReplacementContractError('Prepared frozen input invalid: '+str(error)) from error
                raise
            result=path
        else:result=Path(make())
        if preparation_source.get('split_overlay'):
            # Legacy preparation helpers preserve their original outputs. A
            # separate, hash-pinned view either detaches calibration from the
            # training overlay or binds actual geometry to the canonical root.
            from base5_calibration_layout import normalize_prepared_deck
            if preparation_context is None:
                preparation_context=replacement_contract(preparation_source,original,
                    gen.geo.load_json(Path(original['metadata'])))
            try:
                return normalize_prepared_deck(result,preparation_source,original,
                    path.parent.with_name(path.parent.name+'_contract'),path.parent.name,
                    bind_training=bind_replacement_contract,context=preparation_context)
            except PlacementProposalExhausted:
                raise  # A valid source may still have an ineligible geometry.
            except (OSError,ValueError,TypeError,KeyError) as error:
                raise ReplacementContractError('Prepared unit/layout contract invalid: '+str(error)) from error
        return result
    def replace_far_aware(manifest,folder,require_ground_observable=False):
        deck=gen.geo.load_json(manifest)
        meta=gen.geo.load_json(Path(deck['scenes'][0]['metadata']))
        options=dict(require_ground_observable=require_ground_observable)
        if enforce_nonrepeat:options['known_payloads']=inherited_native|native_payloads(attempts)
        if 'secondary_far_vehicle' in meta.get('spec',{}):
            options['far_known_payloads']=set(far_known_payloads or ())|captured_payloads(attempts)
        return replacement(manifest,folder,**options)
    def save(stage,error=None):
        # Preserve usable selections even when a later child fails. Unsafe
        # children never enter this list or the conversion handoff.
        target=output/'selected_ledger.jsonl';temp=output/'.selected_ledger.tmp'
        temp.write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in selected));temp.replace(target)
        elapsed=prior_elapsed+time.monotonic()-started
        atomic_json(output/'report.json',dict(stage=stage,error=error,elapsed_sec=elapsed,source_manifest_sha256=sha256(source),
            attempts=attempts,selected=selected,unresolved=unresolved,bulk_capture=False,
            native_nonrepeat=dict(enabled=enforce_nonrepeat,inherited_native_payload_sha256=sorted(inherited_native))))
        atomic_json(output/'status.json',dict(stage=stage,error=error,current=attempts[-1]['cell'] if attempts else None,
            requested=len(indices),finished=len(selected)+len(unresolved),accepted_candidate=len(selected),review=len(unresolved),
            failed=int(stage=='halted'),updated_at=time.time(),elapsed_sec=elapsed,eta_sec=0,results=[]))
    def capture(manifest,root,cell,kind):
        if stopped():raise RuntimeError('Quality-loop STOP requested')
        root.mkdir(parents=True,exist_ok=True)
        if (root/'session_contract.json').exists():
            safety_path=root/'shutdown_safety.json'
            if not safety_path.exists() or not gen.geo.load_json(safety_path).get('park_verified'):
                raise RuntimeError('Interrupted/unsafe child requires manual safety recovery and a new campaign')
            if resume:
                session=gen.geo.load_json(root/'session_contract.json');repo=Path(__file__).resolve().parents[3]
                rows,_=prepare_session(root,manifest,
                    repo/'configs/network/ego_network_mr_base5_savesensordata_collision_local.json',
                    repo/'configs/sensors/Sensor_competition_local.json',session.get('runner_options'))
                if rows and rows[-1]['status'] in ('accepted_candidate','review'):
                    row=rows[-1]
                    if gen.geo.load_json(Path(row['capture_root'])/'audit.json')!=row['audit']:
                        raise RuntimeError('Resumed capture audit differs from ledger')
                    if is_collision_failure(row):validate_collision_failure(row)
                    attempts.append(dict(cell=cell,kind=kind,status=row['status'],capture_root=str(root),
                        reasons=row['audit']['review_reasons'],reused_completed_child=True,
                        shutdown=gen.geo.load_json(safety_path)))
                    save('running');return row
        if enforce_nonrepeat:
            candidate=gen.geo.load_json(manifest)['scenes'][0]
            if candidate['scenario_sha256'] in inherited_native|native_payloads(attempts):
                raise NativePayloadRepeated('Native payload already observed; no new MORAI load/capture: '+candidate['scenario_sha256'])
        cmd=[sys.executable,str(runner),'--manifest',str(manifest),'--output',str(root),'--limit','1']
        with (root.parent/(root.name+'.log')).open('w') as log:
            child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+100.
            heartbeat=0.
            while child.poll() is None:
                if progress and time.monotonic()>=heartbeat:
                    progress(cell,kind,root);heartbeat=time.monotonic()+2.
                if stopped() or time.monotonic()>deadline:
                    (root/'STOP').touch();child.terminate()
                    try:child.wait(timeout=15.)
                    except subprocess.TimeoutExpired:raise RuntimeError('Child remains alive after interrupt; no further loads')
                    raise RuntimeError('Child interrupted; check shutdown evidence before any resume')
                try:time.sleep(.1)
                except KeyboardInterrupt:
                    (root/'STOP').touch();child.terminate()
                    child.wait(timeout=15.)
                    raise RuntimeError('Interrupted; child shutdown must be checked before resume')
        if child.returncode:
            safety=gen.geo.load_json(root/'shutdown_safety.json') if (root/'shutdown_safety.json').exists() else {}
            rows=load_ledger(root/'ledger.jsonl')
            child_status=gen.geo.load_json(root/'status.json') if (root/'status.json').exists() else {}
            collision_exit=child_status.get('error') in (
                'Batch review/failure or unsafe boundary Park; halt before next scene',
                'Capture halted after connection failure or three consecutive scene failures')
            if rows and (is_collision_failure(rows[-1]) or is_stationary_failure(rows[-1])) and safety.get('park_verified') and collision_exit:
                reason=scene_rejection(rows[-1])
                row=rows[-1]
                attempts.append(dict(cell=cell,kind=kind,capture_root=str(root),status=row['status'],
                    reasons=[reason],shutdown=safety))
                save('running');return row
            attempts.append(dict(cell=cell,kind=kind,capture_root=str(root),status='failed',
                reasons=['child_failed_or_exit_unsafe'],shutdown=safety))
            raise RuntimeError('Capture child failed; see '+str(root)+'.log')
        safety=gen.geo.load_json(root/'shutdown_safety.json')
        if not safety['park_verified']:raise RuntimeError('Exit Park unverified; no further loads')
        rows=[json.loads(l) for l in (root/'ledger.jsonl').read_text().splitlines() if l]
        row=rows[-1];attempts.append(dict(cell=cell,kind=kind,capture_root=str(root),status=row['status'],
            reasons=row.get('audit',{}).get('review_reasons',[]),shutdown=safety));save('running')
        if scene_rejection(row):return row
        if row['status']=='failed' and (kind!='initial' or not arrival_retry_eligible(row,manifest)):
            raise RuntimeError('Scene failed without safe budget-retry evidence; no automatic continuation')
        return row
    try:
        save('running')
        for index in indices:
            original=copy.deepcopy(source_data['scenes'][index]);cell=original['id'];base=output/cell;base.mkdir(exist_ok=True)
            m=copy.deepcopy(source_data);m.update(scenes=[original],count=1)
            preparation_source=copy.deepcopy(m);preparation_context=None
            manifest=base/'manifest.json';atomic_json(manifest,m)
            if seed_capture is not None:
                if stopped():raise RuntimeError('STOP before seeded recovery')
                result,seed_root,safety=validate_seed(seed_capture,original)
                attempts.append(dict(cell=cell,kind='seeded_batch_target_miss' if result['status']=='failed' else 'seeded_batch_review',capture_root=str(seed_root),
                    status=result['status'],reasons=result.get('audit',{}).get('review_reasons',[]),shutdown=safety,
                    reused_raw=True,seed_source=str(seed_capture)))
                save('running')
            else:
                metadata=gen.geo.load_json(Path(original['metadata'])) if original.get('metadata') else {}
                reused_ground=shadow_ground_reuse.adopt(manifest,base/'motion_reuse_deck')
                if reused_ground is not None:
                    manifest=prepared(reused_ground,lambda:reused_ground)
                    adopted=gen.geo.load_json(manifest)['scenes'][0]
                    metadata=gen.geo.load_json(Path(adopted['metadata']))
                    motion_ground.validate_motion_ground(metadata,gen.geo.load_json(Path(adopted['scenario'])))
                if editor_prepared(metadata):
                    motion_ground.validate_motion_ground(metadata,gen.geo.load_json(Path(original['scenario'])))
                if motion_ground.moving_actors(metadata) and not editor_prepared(metadata) and reused_ground is None:
                    for actor in motion_ground.moving_actors(metadata):
                        path=actor.get('motion_path',{})
                        if path.get('policy')!='natural-target-v1' or 'target_position' not in path:
                            raise ValueError('Moving pilot requires natural-target-v1 metadata before calibration')
                    # Observe terrain and this asset at the target before loading
                    # an animated path. Calibration frames never fill training quota.
                    probes=prepared(base/'motion_terrain_deck'/'manifest.json',
                        lambda:empty_probe(manifest,[0],base/'motion_terrain_deck',backward_route_m=0.))
                    terrain_result=capture(probes,base/'motion_terrain',cell,'motion_empty_terrain')
                    if scene_rejection(terrain_result):
                        unresolved.append(dict(cell=cell,reasons=[scene_rejection(terrain_result)],evidence=terrain_result));save('running');continue
                    refs=prepared(base/'motion_static_deck'/'manifest.json',
                        lambda:motion_ground.static_reference(manifest,base/'motion_static_deck'))
                    reference=capture(refs,base/'motion_static',cell,'motion_static_reference')
                    if scene_rejection(reference):
                        unresolved.append(dict(cell=cell,reasons=[scene_rejection(reference)],evidence=reference));save('running');continue
                    try:
                        manifest=prepared(base/'motion_ground_deck'/'manifest.json',
                            lambda:motion_ground.prepare(manifest,base/'motion_terrain',base/'motion_static',base/'motion_ground_deck'))
                    except ValueError as exc:
                        if str(exc) not in ('Static pivot terrain unsupported',
                                'No independently supported safe motion direction in same target cell'):
                            unresolved.append(dict(cell=cell,reasons=['motion_ground_preparation_unavailable'],error=str(exc)))
                            save('running');continue
                        # Sparse original-view terrain is not a bad placement certificate.
                        # Add bounded empty near-target observations; never extrapolate/waive support.
                        additional=[];contact=None
                        try:
                            requests=near_motion_probe_requests(metadata)
                            for n,(actor,standoff) in enumerate(requests):
                                near_deck=base/f'motion_near_terrain_deck_{n}';near_root=base/f'motion_near_terrain_{n}'
                                probes=prepared(near_deck/'manifest.json',lambda actor=actor,standoff=standoff:
                                    empty_probe(manifest,[0],near_deck,target_uid=actor['unique_id'],
                                        target_motion=bool(actor.get('motion_path')),
                                        target_departure=bool(standoff),target_standoff=standoff or None))
                                witness=capture(probes,near_root,cell,'motion_near_empty_terrain')
                                if scene_rejection(witness):contact=witness;break
                                additional.append(near_root)
                                # Stop calibration as soon as every actor/path is
                                # independently supported, not after probing all eight.
                                try:
                                    prepared(base/'motion_ground_deck'/'manifest.json',lambda:
                                        motion_ground.prepare(manifest,base/'motion_terrain',base/'motion_static',
                                            base/'motion_ground_deck',additional_terrain=additional))
                                    break
                                except ValueError as pending:
                                    if str(pending) not in ('Static pivot terrain unsupported',
                                            'No independently supported safe motion direction in same target cell'):
                                        raise
                            if contact:
                                unresolved.append(dict(cell=cell,reasons=[scene_rejection(contact)],evidence=contact))
                                save('running');continue
                            manifest=prepared(base/'motion_ground_deck'/'manifest.json',lambda:
                                motion_ground.prepare(manifest,base/'motion_terrain',base/'motion_static',
                                    base/'motion_ground_deck',additional_terrain=additional))
                        except ValueError as near_exc:
                            unresolved.append(dict(cell=cell,reasons=['motion_ground_preparation_unavailable'],error=str(near_exc)))
                            save('running');continue
                result=capture(manifest,base/'initial',cell,'initial')
            if scene_rejection(result):
                unresolved.append(dict(cell=cell,reasons=[scene_rejection(result)],evidence=result));save('running');continue
            if result['status']=='failed':
                if not arrival_retry_eligible(result,manifest):
                    raise RuntimeError('Target miss lacks safe evidence or already used 9-second budget')
                manifest=prepared(base/'budget9_deck'/'manifest.json',
                    lambda:budget_retry(manifest,[0],base/'budget9_deck',9.))
                result=capture(manifest,base/'budget9',cell,'natural_budget9_retry')
                if scene_rejection(result):
                    unresolved.append(dict(cell=cell,reasons=[scene_rejection(result)],evidence=result));save('running');continue
                if result['status']=='failed':raise RuntimeError('9-second target retry failed; no further loads')
            if result['status']=='review':
                current_meta=gen.geo.load_json(Path(gen.geo.load_json(manifest)['scenes'][0]['metadata']))
                if motion_ground.moving_actors(current_meta):
                    # Outer same-quota replacement retains the editor-path contract;
                    # legacy movers still need fresh independent terrain preparation.
                    unresolved.append(dict(cell=cell,reasons=result['audit']['review_reasons'],
                        next_policy=('same-quota replacement within live editor-height domain'
                            if editor_prepared(current_meta) else 'same-quota replacement with fresh motion calibration')))
                    save('running');continue
            # Up to eight actor witnesses, plus bounded height/replacement work.
            # A larger population must not exhaust the old three-actor budget.
            # Every actual terrain/contact/visibility gate remains unchanged.
            budget=StaticQualityBudget(source_data.get('config',{}))
            for ordinal in range(2*motion_ground.MAX_MOVING_ACTORS+2):
                if result['status']=='accepted_candidate':break
                reasons=set(result.get('audit',{}).get('review_reasons',[]))
                if scene_rejection(result):break
                current_meta=gen.geo.load_json(Path(gen.geo.load_json(manifest)['scenes'][0]['metadata']))
                if lightweight_static(current_meta):
                    # No terrain witnesses / height recaptures / Ego relocation
                    # solely to verify static ground. A severe gap remains deficit;
                    # the outer campaign may make one same-quota replacement.
                    if any(r.startswith('static_ground_') for r in reasons):break
                    if not budget.spend('replacement'):break
                    try:
                        next_manifest=prepared(base/f'replacement_deck_{ordinal}'/'manifest.json',
                            lambda:replace_far_aware(manifest,base/f'replacement_deck_{ordinal}'))
                    except PlacementProposalExhausted:
                        if ('secondary_far_vehicle' not in current_meta.get('spec',{})
                                and not current_meta.get('training_split_overlay')):raise
                        budget.reason=('far_secondary_offline_proposals_exhausted'
                            if 'secondary_far_vehicle' in current_meta.get('spec',{})
                            else 'replacement_offline_proposals_exhausted')
                        break  # No new load; keep the native rejected scene as deficit.
                    manifest=next_manifest
                    result=capture(manifest,base/f'retry_{ordinal}',cell,'same_cell_replacement')
                    continue
                current=Path(attempts[-1]['capture_root'])
                local_height=any(a.get('reason')=='actor_annotation_above_ground' and a.get('plane')
                    for a in result.get('audit',{}).get('ground_support',{}).get('actors',[]))
                if local_height and reasons<={'actor_annotation_above_ground','actor_ground_plane_unverified'}:
                    if not budget.spend('height'):break
                    next_manifest=prepared(base/'height_deck'/'manifest.json',lambda:height_retry(manifest,current,base/'height_deck'))
                    kind='input_z_retry'
                elif reasons and reasons<={'actor_ground_plane_unverified','actor_annotation_deep_below_ground','actor_annotation_above_ground'}:
                    try:
                        probe_meta=gen.geo.load_json(Path(gen.geo.load_json(manifest)['scenes'][0]['metadata']))
                        failed_ground=[a['uid'] for a in result['audit'].get('ground_support',{}).get('actors',[]) if a.get('reason')]
                        target_uid=((failed_ground or [probe_meta['runtime_actors'][0]['unique_id']])[0]
                            if probe_meta.get('ground_support_check',{}).get('policy')=='all-actors-observed-ground-v1'
                            and probe_meta.get('runtime_actors') else None)
                        witness_key=(target_uid,gen.geo.load_json(manifest)['scenes'][0]['scenario_sha256'])
                        if not budget.spend('witness',witness_key):break
                        terrain_deck=base/f'terrain_deck_{ordinal}';terrain_root=base/f'terrain_{ordinal}'
                        retry_deck=base/f'probed_deck_{ordinal}'
                        target=next((a for a in probe_meta.get('runtime_actors',[]) if a['unique_id']==target_uid),None)
                        components=[c for c in result['audit'].get('components',[]) if c['parent_uid']==target_uid]
                        if (reasons=={'actor_ground_plane_unverified'} and target
                                and (len(components)==1 or target['name'] in COMPOSITE_ANCHORS)
                                and not motion_ground.moving_actors(probe_meta)):
                            near_deck=base/f'static_near_deck_{ordinal}';near_root=base/f'static_near_{ordinal}'
                            probes=prepared(near_deck/'manifest.json',lambda:static_height_reference(manifest,0,near_deck,target_uid=target_uid))
                            reference=capture(probes,near_root,cell,'static_near_ground_reference')
                            if scene_rejection(reference):result=reference;break
                            if reference['status']=='review' and reference['audit']['review_reasons']==['actor_annotation_above_ground']:
                                if not budget.spend('height'):break
                                next_manifest=prepared(retry_deck/'manifest.json',
                                    lambda:near_height_retry(manifest,current,near_root,retry_deck,target_uid))
                                manifest=next_manifest
                                result=capture(manifest,base/f'retry_{ordinal}',cell,'near_measured_input_z_retry')
                                save('running');continue
                            next_manifest=prepared(retry_deck/'manifest.json',lambda:static_reference_retry(manifest,near_root,retry_deck))
                            kind='static_rigid_ground_retry'
                            result=reaudit_original(result,next_manifest,base/f'reaudited_{ordinal}')
                            attempts.append(dict(cell=cell,kind='static_rigid_raw_reaudit',capture_root=result['capture_root'],
                                status=result['status'],reasons=result['audit']['review_reasons'],reused_sensor_capture=True,
                                shutdown=gen.geo.load_json(near_root/'shutdown_safety.json')))
                            manifest=next_manifest
                            save('running')
                            continue
                        else:
                            probes=prepared(terrain_deck/'manifest.json',lambda:empty_probe(manifest,[0],terrain_deck,target_uid=target_uid))
                            terrain_result=capture(probes,terrain_root,cell,'empty_terrain')
                            if scene_rejection(terrain_result):result=terrain_result;break
                            if not budget.spend('height'):break
                            next_manifest=prepared(retry_deck/'manifest.json',lambda:probed_retry(manifest,current,terrain_root,retry_deck))
                            kind='independent_ground_retry'
                    except ValueError as exc:
                        # Unsupported terrain is a quality deficit, not permission
                        # to waive the gate. Communication exceptions stay fatal.
                        attempts.append(dict(cell=cell,kind='ground_preparation_unavailable',error=str(exc)))
                        if not budget.spend('replacement'):break
                        fallback_deck=base/f'fallback_deck_{ordinal}'
                        next_manifest=prepared(fallback_deck/'manifest.json',lambda:replace_far_aware(manifest,fallback_deck,require_ground_observable=True))
                        kind='same_cell_replacement'
                else:
                    if not budget.spend('replacement'):break
                    next_manifest=prepared(base/f'replacement_deck_{ordinal}'/'manifest.json',lambda:replace_far_aware(manifest,base/f'replacement_deck_{ordinal}',
                        require_ground_observable='actor_ground_plane_unverified' in reasons))
                    kind='same_cell_replacement'
                if not gen.geo.load_json(next_manifest)['scenes']:break
                manifest=next_manifest
                result=capture(manifest,base/f'retry_{ordinal}',cell,kind)
            if result['status']=='accepted_candidate':
                planned=gen.geo.load_json(Path(gen.geo.load_json(manifest)['scenes'][0]['metadata']))
                retained=gen.geo.load_json(Path(result['capture_root'])/'metadata.json')
                if planned.get('calibration_only') or retained.get('calibration_only'):
                    raise ReplacementContractError('Calibration capture cannot fill a training quota')
                result['source_cell_id']=cell;selected.append(result)
            else:unresolved.append(dict(cell=cell,reasons=result.get('audit',{}).get('review_reasons',[result.get('error','quality deficit')]),
                static_quality_budget_reason=budget.reason,static_quality_work=budget.counts))
            save('running')
        save('complete');return 0
    except Exception as exc:save('halted',str(exc));print(str(exc),file=sys.stderr);return 1
    finally:lock.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--indices',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    raise SystemExit(execute(a.manifest,[int(i) for i in a.indices.split(',')],a.output))
