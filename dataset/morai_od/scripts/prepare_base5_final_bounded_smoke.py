#!/usr/bin/env python3
"""Preregister a bounded exact-source smoke for final scale/unit/layout inputs.

No native outcomes select these rows. Paired stability/unit diagnostics remain
separate policy evidence, never extra smoke approvals or new training data.
"""
import argparse
from collections import Counter
import copy
from fractions import Fraction
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import audit_base5_scenarios
import base5_asset_scale as scale
import base5_front_release_inputs as pins
import base5_layout_split as layout
import base5_shadow_ground_reuse as shadow
import base5_unit_geometry_contract as unit
from base5_scale_capture_contract import validate_entry
from base5_capture_safety import atomic_json,sha256
import prepare_base5_scale_extent_smoke as legacy_features
import prepare_base5_scale_stability as stability

SCHEMA='base5_final_bounded_smoke_registration_v1'
POLICY='final-scale-unit-layout-exact-source-smoke-v156'
DEFAULT_TARGET=24
MAX_FRAMES=64
RELEASE_HELPER='finalize_base5_final_bounded_release.py'
STABILITY_GATE=dict(independent_pairs=60,first_attempt_captures=120,minimum_per_profile=3,
    minimum_initial_approval_fraction=.90,maximum_scaled_nonaccept_excess=.05,
    maximum_scaled_cost_per_approval_ratio=1.10,balanced_pair_orders={'original/scaled':30,'scaled/original':30},
    cost_scope='All observed ledger durations per latest approved scene; outside process/operator/wall-clock cost excluded',
    source_uid_policy='Exact registered preparation; no undeclared diagnostic UID transform',
    report_summaries_trusted_without_recomputation=False)
CPU_EXCLUSIONS=[dict(node='dataset/morai_od/tests/test_prepare_base3_lraspp_4class.py::test_segmentation_metrics_include_foreground_quality',
    reason='Legacy Base3 segmentation metrics require unavailable host torch; outside Base5 qualification. Static Base3 tests remain included.')]
CPU_ENVIRONMENT=dict(PYTEST_ADDOPTS='',PYTEST_PLUGINS='',PYTEST_DISABLE_PLUGIN_AUTOLOAD='1')


def read(path):return json.loads(Path(path).read_text())


def require(value,message):
    if not value:raise ValueError(message)


def order_key(label,ident):
    return hashlib.sha256((POLICY+':'+label+':'+ident).encode()).hexdigest(),ident


def qualified_pairs(policy):
    scale.validate_policy(policy)
    pairs={(name,ident) for name,asset in policy['assets'].items()
        for ident,profile in asset['profiles'].items() if scale._qualified(profile)}
    require(1<=len(pairs)<=128,'A finite nonempty current qualified profile set is required')
    return pairs


def profile_minima(pairs):
    """Register rare CargoBox 3x inventory before looking at native outcomes."""
    return {'profile/'+name+'/'+profile:3 if (name,profile)==('CargoBox','iso300') else 1
        for name,profile in pairs}


def profile_evidence_hashes(context):
    return {ref['path']:ref['sha256'] for name,profile in context['pairs']
        for ref in context['policy']['assets'][name]['profiles'][profile]['native_qualification']['evidence']}


def features(meta,pairs,effective_split,unit_assets):
    result=legacy_features.features(meta,pairs)
    if meta.get('training_split_overlay'):result.add('pipeline/layout_overlay')
    if meta.get(unit.KEY):result.add('pipeline/unit_reference')
    if meta.get(shadow.KEY):result.add('pipeline/materialized_shadow')
    if meta['spec'].get('static_mission_extent'):
        result.add('extent_training_split/'+effective_split)
    for actor in meta['runtime_actors']:
        factors=(actor.get('asset_scale') or {}).get('scale_xyz',[1.,1.,1.])
        if (actor['name'] in unit_assets and list(factors)==[1.,1.,1.]
                and actor['source_list']=='objectList' and actor['motion'] in ('static','standing')):
            result.add('unit_control/physical')
    return result


def select(deck,metadata,pairs,overlay,unit_assets,shadow_ids,target=DEFAULT_TARGET,maximum=MAX_FRAMES):
    require(type(maximum) is int and 8<=maximum<=MAX_FRAMES,'Finite 8..64 smoke cap required')
    require(type(target) is int and 8<=target<=maximum,'Explicit target must remain inside smoke cap')
    entries={e['id']:e for e in deck['scenes']}
    require(len(entries)==len(deck['scenes']) and set(entries)==set(metadata),'Unique complete metadata identity required')
    require(len(shadow_ids)==len(set(shadow_ids))==2,'Two explicit materialized shadow controls required')
    effective={ident:layout.resolve(overlay,ident,entries[ident],meta)['split'] for ident,meta in metadata.items()}
    covers={ident:features(meta,pairs,effective[ident],unit_assets) for ident,meta in metadata.items()}
    available=set().union(*covers.values()) if covers else set()
    minima=profile_minima(pairs)
    frequency=Counter(token for value in covers.values() for token in value)
    require(all(frequency[token]>=minimum for token,minimum in minima.items()),
        'Full source lacks preregistered rare profile inventory: '+str(minima))
    required={'profile/'+name+'/'+profile for name,profile in pairs}
    required.update('extent_stratum/'+v for v in ('before','intersection','after'))
    required.update('extent_phase/'+v for v in legacy_features.PHASES)
    required.update(('extent_training_split/train','extent_training_split/val',
        'pipeline/layout_overlay','pipeline/unit_reference','pipeline/materialized_shadow','unit_control/physical'))
    optional={'scaled_context/'+v for v in legacy_features.CONTEXTS}|{'scaled_role/primary','scaled_role/auxiliary'}
    required.update(optional&available)
    require(required<=available,'Full source lacks required smoke coverage: '+str(sorted(required-available)))
    selected=[];labels={}
    def add(ident,label):
        require(ident in entries and ident not in labels,'Missing/duplicate protected control: '+label)
        require(not legacy_features.variants(metadata[ident],pairs),'Protected controls must remain unit geometry')
        labels[ident]=[label];selected.append(ident)
    for ident in sorted(shadow_ids):
        meta=metadata.get(ident,{})
        require(meta.get(shadow.KEY) and meta.get('spec',{}).get('family')=='shadow'
            and any(a['motion'] not in ('static','standing') for a in meta['runtime_actors']),
            'Shadow geometry must already be materialized before registration')
        add(ident,'protected/shadow_materialized/'+ident)
    for crossing in legacy_features.CROSSWALKS:
        choices=[ident for ident,meta in metadata.items() if ident not in labels
            and meta['spec']['family']!='shadow' and not legacy_features.variants(meta,pairs)
            and any(a.get('mission_crossing_id')==crossing and a['motion'] in ('walking','running') for a in meta['runtime_actors'])]
        require(choices,'Missing exact moving crosswalk control: '+crossing)
        add(min(choices,key=lambda ident:order_key('crosswalk/'+crossing,ident)),'protected/crosswalk/'+crossing)
    for lo,hi in ((50,80),(80,100)):
        for role in ('front','rear'):
            label=f'protected/far/{lo}_{hi}m/{role}'
            choices=[ident for ident,meta in metadata.items() if ident not in labels
                and meta['spec']['family']=='highway' and not legacy_features.variants(meta,pairs)
                and any(a['source_list']=='vehicleList' and a.get('actual_ego_role')==role
                    and lo<=a.get('planned_center_distance_m',-1)<hi for a in meta['runtime_actors'])]
            require(choices,'Missing exact far vehicle control: '+label)
            add(min(choices,key=lambda ident:order_key(label,ident)),label)
    def deficits(ids):
        counts=Counter(token for ident in ids for token in covers[ident])
        return (required-set(counts))|{token for token,minimum in minima.items() if counts[token]<minimum}
    uncovered=deficits(selected)
    while uncovered:
        choices=[ident for ident in entries if ident not in labels and covers[ident]&uncovered]
        require(choices and len(selected)<target,'Declared target cannot cover required cells: '+str(sorted(uncovered)))
        ident=min(choices,key=lambda ident:(-len(covers[ident]&uncovered),
            -sum((Fraction(1,frequency[t]) for t in covers[ident]&uncovered),Fraction()),order_key('cover',ident)))
        labels[ident]=['metadata_set_cover'];selected.append(ident);uncovered=deficits(selected)
    for ident in selected[8:][::-1]:
        if not deficits([i for i in selected if i!=ident]):
            selected.remove(ident);del labels[ident]
    covered_count=len(selected)
    extras=sorted((ident for ident in entries if ident not in labels),key=lambda ident:(
        not bool(legacy_features.variants(metadata[ident],pairs)),order_key('fill',ident)))
    require(len(extras)>=target-len(selected),'Insufficient exact source entries for declared target')
    for ident in extras[:target-len(selected)]:labels[ident]=['fixed_target_extra'];selected.append(ident)
    counts=Counter(token for ident in selected for token in covers[ident])
    rows=[dict(id=ident,selection_roles=labels[ident],coverage=sorted(covers[ident]),
        scaled_actors=legacy_features.variants(metadata[ident],pairs),effective_training_split=effective[ident],
        scenario_sha256=entries[ident]['scenario_sha256'],metadata_sha256=sha256(entries[ident]['metadata']),
        source_entry_preserved=True) for ident in selected]
    return rows,dict(required=sorted(required),covered=sorted(set().union(*(covers[i] for i in selected))),
        unavailable_scale_context_or_roles=sorted(optional-available),frames=len(rows),target_frames=target,
        maximum_frames=maximum,protected_frames=8,set_cover_frames=covered_count,
        profile_minima=minima,source_profile_frames={token:frequency[token] for token in sorted(minima)},
        selected_profile_frames={token:counts[token] for token in sorted(minima)},
        native_outcomes_used_for_selection=False,coverage_passed=True)


def runtime_hashes():
    # The normal capture closure stays authoritative. Qualification additionally
    # imports two tools from BEVFusion's tools directory; the legacy single-root
    # closure intentionally cannot resolve those imports. Freeze all current
    # sibling sources plus those explicit tools, without weakening that parser.
    result=pins.collect_runtime()
    scripts=Path(__file__).resolve().parent
    for path in scripts.glob('*.py'):result[str(path)]=sha256(path)
    tools=pins.ROOT/'perception/detection3d/bevfusion/tools'
    for name in ('prepare_base5_portable_dataset.py','prepare_base5_pedestrian_merged.py'):
        path=tools/name;result[str(path)]=sha256(path)
    # The actual CPU loader imports the current MMDet config inheritance and
    # local dataset/transforms. Freeze those sources before observations too.
    bevfusion=pins.ROOT/'perception/detection3d/bevfusion'
    for directory in ('configs','plugin'):
        for path in (bevfusion/directory).rglob('*.py'):result[str(path.resolve())]=sha256(path)
    pins.verify_hashes(result,'Final source and qualification runtime')
    return dict(sorted(result.items()))


def test_hashes():
    return {str(path.resolve()):sha256(path) for path in sorted((pins.ROOT/'dataset/morai_od/tests').rglob('*.py'))}


def cpu_command(xml):
    return [sys.executable,'-m','pytest','-q','-c','/dev/null','--rootdir='+str(pins.ROOT),
        '-o','addopts=','-p','no:cacheprovider',str(pins.ROOT/'dataset/morai_od/tests'),
        *['--deselect='+item['node'] for item in CPU_EXCLUSIONS],'--junitxml='+str(xml)]


def qualify_cpu(output):
    """Execute the complete CPU suite, binding actual XML to current sources."""
    output=Path(output).resolve();require(not output.exists(),'Fresh CPU regression output required')
    before=runtime_hashes();tests=test_hashes();output.mkdir(parents=True)
    xml=output/'tests.xml'
    command=cpu_command(xml);environment=dict(os.environ);environment.update(CPU_ENVIRONMENT)
    result=subprocess.run(command,cwd=pins.ROOT,check=False,env=environment)
    require(result.returncode==0,'Current complete CPU regression failed; no passing receipt is issued')
    require(runtime_hashes()==before and test_hashes()==tests,'Runtime/tests changed during CPU regression')
    from finalize_base5_scale_extent_release import test_results
    results=test_results([xml])
    receipt=dict(schema='base5_final_cpu_regression_v1',helper_sha256=sha256(__file__),
        runtime_hashes_before=before,runtime_hashes_after=before,test_hashes=tests,command=command,
        returncode=0,test_results=results,pytest_receipt_hashes={str(xml):sha256(xml)},
        fixed_exclusions=CPU_EXCLUSIONS,pytest_environment_controls=CPU_ENVIRONMENT,
        gpu_execution=False,training_started=False,capture_started=False)
    atomic_json(output/'receipt.json',receipt)
    return output/'receipt.json'


def inspect_source(manifest):
    manifest=Path(manifest).resolve(strict=True);deck=read(manifest)
    require(deck.get('count')==len(deck.get('scenes',[]))==5000,'Exact full5K source required')
    require(deck['config'].get('environment_tag')=='sunny13','Only current Sunny13 source can qualify')
    require(deck.get('split_overlay') and deck['config'].get(unit.KEY),'Final layout and unit QA policies required')
    layout_audit=layout.validate_manifest_overlay(deck)
    overlay=layout.read_overlay(deck['split_overlay']['path'])
    unit_policy=unit.load(deck['config'][unit.KEY])
    reuse_pins=shadow.validate_manifest(deck)
    require(reuse_pins and shadow.KEY in deck['config'],'Explicit materialized shadow registry required')
    registry=read(deck['config'][shadow.KEY]['registration']['path'])
    shadow_ids=[r['source_id'] for r in registry['entries']]
    view=dict(deck,config=scale.bind_config(deck['config']));pairs=qualified_pairs(view['config'][scale.CONFIG_KEY])
    entries,metadata={},{}
    for entry in deck['scenes']:
        ident=entry['id'];require(ident not in entries,'Duplicate final scene ID')
        require(sha256(entry['scenario'])==entry['scenario_sha256'],'Final scenario checksum differs')
        meta=read(entry['metadata']);require(entry['split']==meta['split'],'Final source capture split differs')
        require(meta.get(unit.KEY)==deck['config'][unit.KEY],'Metadata unit reference differs')
        validate_entry(entry,view)
        entries[ident]=entry;metadata[ident]=meta
    inputs=pins.collect_inputs(manifest);inputs.update(reuse_pins);inputs.update(overlay.get('source_hashes',{}))
    inputs.update(overlay.get('finalization_source_hashes',{}))
    inputs[str(Path(deck['split_overlay']['path']).resolve())]=sha256(deck['split_overlay']['path'])
    scale.verify_frozen_policy(view['config'][scale.CONFIG_KEY]);pins.verify_hashes(inputs)
    return dict(deck=deck,entries=entries,metadata=metadata,overlay=overlay,pairs=pairs,
        policy=view['config'][scale.CONFIG_KEY],unit_assets=set(unit_policy['assets']),shadow_ids=shadow_ids,
        input_hashes=inputs,layout_audit=layout_audit)


def stability_context(manifest,full,source):
    """Reproduce the outcome-free paired plan, including all geometry and IDs."""
    manifest=Path(manifest).resolve(strict=True);deck=read(manifest)
    registered=deck.get('scale_stability_registration',{})
    require(registered.get('source_manifest')==str(Path(full).resolve())
        and registered.get('preparation_script_sha256')==sha256(stability.__file__)
        and registered.get('independent_pairs')==60 and registered.get('first_attempt_captures')==120
        and registered.get('selection_uses_native_outcomes') is False
        and registered.get('raw_files_modified') is False and registered.get('source_unchanged') is True
        and registered.get('balanced_order_counts')==dict(original_scaled=30,scaled_original=30)
        and registered.get('coverage',{}).get('minimum_independent_scenes_per_profile')==3
        and deck.get('count')==len(deck.get('scenes',[]))==120,
        'Exact same-final-source 60-pair minimum3 preregistration required')
    entries={entry['id']:entry for entry in deck['scenes']}
    require(len(entries)==120,'Repeated paired scene IDs')
    metadata={ident:read(entry['metadata']) for ident,entry in entries.items()}
    references=[meta.get('scale_stability',{}).get('unit_reference_sources') for meta in metadata.values()]
    require(references and isinstance(references[0],dict) and references[0]
        and all(ref==references[0] for ref in references),'Exact common native unit reference pins required')
    pins.verify_hashes(references[0],'Paired native unit references')
    inventory=dict(registered.get('source_files',{}));inventory.update(references[0])
    inventory[str(manifest)]=sha256(manifest)
    for entry in deck['scenes']:
        require(sha256(entry['scenario'])==entry['scenario_sha256'],'Paired scenario checksum differs')
        for key in ('scenario','metadata'):inventory[str(Path(entry[key]).resolve())]=sha256(entry[key])
    pins.verify_hashes(inventory,'Paired source/geometry/input pins')
    with tempfile.TemporaryDirectory(prefix='base5-paired-replay-') as directory:
        replay=read(stability.prepare(full,Path(directory)/'replay',sorted(references[0]),pairs=60,minimum_per_profile=3))
        require({key:value for key,value in deck.items() if key!='scenes'}==
            {key:value for key,value in replay.items() if key!='scenes'},'Paired preparation/config/source lineage differs')
        for actual,expected in zip(deck['scenes'],replay['scenes']):
            require({key:value for key,value in actual.items() if key not in ('scenario','metadata')}==
                {key:value for key,value in expected.items() if key not in ('scenario','metadata')}
                and read(actual['scenario'])==read(expected['scenario'])
                and read(actual['metadata'])==read(expected['metadata']),
                'Paired registered identity, UID, geometry, metadata or order changed')
    paired={};profile_counts=Counter();orders=Counter();geometry_groups=set()
    for ident,meta in metadata.items():
        tag=meta['scale_stability'];condition=tag['condition'];pair_id=tag['pair_id']
        require(condition in ('original','scaled') and condition not in paired.setdefault(pair_id,{}),
            'Repeated or foreign paired condition')
        paired[pair_id][condition]=ident
        if condition=='scaled':
            source_id=tag['source_id'];require(source_id in source['metadata'],'Foreign paired source identity')
            group=layout.resolve(source['overlay'],source_id,source['entries'][source_id],source['metadata'][source_id])['group_id']
            require(group not in geometry_groups,'Repeated actual source geometry in paired diagnostics')
            geometry_groups.add(group);profile_counts.update(set(stability.variants(meta)))
            orders.update([tag['pair_order']])
    require(len(paired)==len(geometry_groups)==60 and all(set(pair)=={'original','scaled'} for pair in paired.values())
        and dict(orders)==STABILITY_GATE['balanced_pair_orders']
        and all(profile_counts[profile]>=3 for profile in source['pairs']),
        'Independent geometry/profile minimum or AB balance differs')
    rare={ident for ident,meta in source['metadata'].items() if ('CargoBox','iso300') in stability.variants(meta)}
    observed={meta['scale_stability']['source_id'] for meta in metadata.values()
        if meta['scale_stability']['condition']=='scaled' and ('CargoBox','iso300') in stability.variants(meta)}
    require(not rare or rare==observed,'CargoBox iso300 source inventory must be included in full')
    return dict(manifest=manifest,deck=deck,entries=entries,metadata=metadata,paired=paired,input_hashes=inventory,
        profile_scene_counts={name+'/'+profile:profile_counts[(name,profile)] for name,profile in sorted(source['pairs'])},
        rare_source_ids=sorted(rare),gate=copy.deepcopy(STABILITY_GATE))


def prepare(manifest,output,target=DEFAULT_TARGET,maximum=MAX_FRAMES,support_evidence=(),*,stability_manifest):
    manifest,output=Path(manifest).resolve(strict=True),Path(output).resolve()
    require(not output.exists() and output!=manifest.parent and manifest.parent not in output.parents,'Fresh output outside source required')
    import generate_base5_independent as gen
    runtime_dir=gen.geo.DEFAULT_RUNTIME_ASSET_DIR.resolve()
    require(output!=runtime_dir and runtime_dir not in output.parents,'Smoke output cannot touch MORAI runtime')
    runtime=runtime_hashes();context=inspect_source(manifest)
    paired=stability_context(stability_manifest,manifest,context)
    rows,coverage=select(context['deck'],context['metadata'],context['pairs'],context['overlay'],
        context['unit_assets'],context['shadow_ids'],target,maximum)
    audit=audit_base5_scenarios.audit(manifest)
    require(audit.get('status')=='OFFLINE_PASS' and not audit.get('errors')
        and audit.get('scenes')==audit.get('unique_payloads')==5000,'Current full5K offline audit failed')
    evidence={str(Path(path).resolve(strict=True)):sha256(path) for path in support_evidence}
    profile_evidence=profile_evidence_hashes(context)
    subset=copy.deepcopy(context['deck']);subset.update(count=len(rows),
        scenes=[copy.deepcopy(context['entries'][row['id']]) for row in rows],
        status='PREREGISTERED_FINAL_BOUNDED_SMOKE_ONLY; no bulk or completion',bulk_ready=False)
    atomic_json(output/'manifest.json',subset);atomic_json(output/'source_audit.json',audit)
    registration=dict(schema=SCHEMA,policy=POLICY,helper_sha256=sha256(__file__),
        registered_at_unix=time.time(),native_observations_must_follow_registration=True,
        release_helper_sha256=sha256(Path(__file__).with_name(RELEASE_HELPER)),
        source_manifest=str(manifest),source_manifest_sha256=sha256(manifest),
        subset_manifest_sha256=sha256(output/'manifest.json'),source_audit_sha256=sha256(output/'source_audit.json'),
        runtime_hashes=runtime,input_hashes=context['input_hashes'],qualified_profiles=[list(p) for p in sorted(context['pairs'])],
        policy_sha256=scale.digest(context['policy']),qualification_evidence_hashes=profile_evidence,
        support_evidence_hashes=evidence,support_evidence_role='Policy diagnostics only; zero native smoke approvals counted',
        selected=rows,coverage=coverage,native_qa_unchanged=True,live_qualified=False,
        bulk_started=False,training_started=False,completion_live_qualified=False,
        partial_resume_plan=dict(chunk_size=4,preserved_selected=4),
        paired_stability=dict(manifest=str(paired['manifest']),manifest_sha256=sha256(paired['manifest']),
            input_hashes=paired['input_hashes'],profile_scene_counts=paired['profile_scene_counts'],
            rare_source_ids=paired['rare_source_ids'],gate=paired['gate']),
        source_entry_policy='Exact final source entries/config; no post-selection geometry or metadata mutation')
    atomic_json(output/'registration.json',registration)
    for values in (runtime,context['input_hashes'],profile_evidence):pins.verify_hashes(values)
    if evidence:pins.verify_hashes(evidence)
    return output/'manifest.json'


def registration_context(manifest,registration):
    manifest,registration=Path(manifest).resolve(strict=True),Path(registration).resolve(strict=True)
    doc=read(registration)
    require(doc.get('schema')==SCHEMA and doc.get('policy')==POLICY,'New bounded smoke preregistration required')
    require(doc.get('helper_sha256')==sha256(__file__)
        and doc.get('release_helper_sha256')==sha256(Path(__file__).with_name(RELEASE_HELPER)),
        'Preregistered smoke/release helper changed')
    require(doc.get('source_manifest')==str(manifest) and doc.get('source_manifest_sha256')==sha256(manifest),
        'Preregistered final source changed')
    require(doc.get('live_qualified') is False and doc.get('bulk_started') is False
        and doc.get('training_started') is False and doc.get('completion_live_qualified') is False
        and doc.get('native_qa_unchanged') is True,'Preregistration cannot claim observed approvals')
    require(type(doc.get('registered_at_unix')) in (int,float) and 0<doc['registered_at_unix']<=time.time()
        and doc.get('native_observations_must_follow_registration') is True,
        'Explicit pre-observation registration time required')
    current=runtime_hashes();require(doc.get('runtime_hashes')==current,'Runtime changed after preregistration')
    context=inspect_source(manifest);require(doc['input_hashes']==context['input_hashes'],'Full source input inventory changed')
    paired_doc=doc.get('paired_stability',{})
    require(paired_doc.get('gate')==STABILITY_GATE,'Preregistered paired acceptance/cost thresholds changed')
    paired=stability_context(paired_doc['manifest'],manifest,context)
    require(paired_doc==dict(manifest=str(paired['manifest']),manifest_sha256=sha256(paired['manifest']),
        input_hashes=paired['input_hashes'],profile_scene_counts=paired['profile_scene_counts'],
        rare_source_ids=paired['rare_source_ids'],gate=paired['gate']),'Preregistered paired source/plan changed')
    require(doc['qualified_profiles']==[list(p) for p in sorted(context['pairs'])]
        and doc['policy_sha256']==scale.digest(context['policy']),'Current approved profile policy differs')
    require(doc.get('qualification_evidence_hashes')==profile_evidence_hashes(context),
        'Exact qualified policy evidence inventory changed')
    rows,coverage=select(context['deck'],context['metadata'],context['pairs'],context['overlay'],
        context['unit_assets'],context['shadow_ids'],doc['coverage']['target_frames'],doc['coverage']['maximum_frames'])
    require(rows==doc['selected'] and coverage==doc['coverage'],'Metadata-only smoke selection changed')
    subset_path=registration.parent/'manifest.json';subset=read(subset_path)
    expected=copy.deepcopy(context['deck']);expected.update(count=len(rows),
        scenes=[copy.deepcopy(context['entries'][row['id']]) for row in rows],
        status='PREREGISTERED_FINAL_BOUNDED_SMOKE_ONLY; no bulk or completion',bulk_ready=False)
    require(sha256(subset_path)==doc['subset_manifest_sha256'] and subset==expected,'Exact registered source subset changed')
    require(sha256(registration.parent/'source_audit.json')==doc['source_audit_sha256'],'Registered full offline audit changed')
    for key in ('qualification_evidence_hashes','support_evidence_hashes'):
        if doc[key]:pins.verify_hashes(doc[key])
    require(doc.get('partial_resume_plan')==dict(chunk_size=4,preserved_selected=4),'Partial resume plan changed')
    context.update(full=manifest,registration=registration,doc=doc,runtime=current,subset=subset,
        subset_path=subset_path,selected={row['id']:row for row in rows})
    context['paired_stability']=paired
    return context


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    for name in ('manifest','stability-manifest'):parser.add_argument('--'+name,type=Path)
    parser.add_argument('--cpu-regression-only',action='store_true')
    parser.add_argument('--target-frames',type=int,default=DEFAULT_TARGET)
    parser.add_argument('--maximum-frames',type=int,default=MAX_FRAMES)
    parser.add_argument('--support-evidence',type=Path,action='append',default=[])
    args=parser.parse_args()
    if args.cpu_regression_only:
        require(args.manifest is None and args.stability_manifest is None and not args.support_evidence,
            'CPU-only execution does not register a source or capture')
        print(qualify_cpu(args.output))
    else:
        require(args.manifest is not None and args.stability_manifest is not None,'Full and paired manifests required')
        print(prepare(args.manifest,args.output,args.target_frames,args.maximum_frames,args.support_evidence,
            stability_manifest=args.stability_manifest))
