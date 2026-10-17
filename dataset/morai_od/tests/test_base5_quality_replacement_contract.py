"""CPU-only opt-in unit/layout preservation and canonical nested rebindings."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import base5_layout_split as layout
import base5_unit_geometry_contract as unit
import run_base5_quality_loop as loop
from prepare_base5_layout_split import prepare


def fixture(root):
    reference_path=root/'unit.json';loop.atomic_json(reference_path,dict(test_only=True))
    reference=dict(path=str(reference_path),sha256=loop.sha256(reference_path))
    entries=[]
    for i in range(30):
        meta=dict(retry=0,ego=dict(x=float(i),y=0.,z=0.,yaw=0.,link_idx='one-map-link'),
            split='train',runtime_actors=[],scene_group_id='clip_'+str(i),
            spec=dict(family='background',mission=False,role='front',distance_bin=0,yaw_limit=3,
                topology='route_general',seed=i),**{unit.KEY:reference})
        sp,mp=root/'source'/(str(i)+'.json'),root/'source'/(str(i)+'_metadata.json')
        loop.atomic_json(sp,dict(source=i));loop.atomic_json(mp,meta)
        entries.append(dict(id='scene_'+str(i),split='train',metadata=str(mp),scenario=str(sp),
            scenario_sha256=loop.sha256(sp)))
    source=root/'source/manifest.json';loop.atomic_json(source,dict(count=30,scenes=entries,
        config={unit.KEY:reference},provenance={}))
    result=prepare(source,root/'candidate');deck=loop.gen.geo.load_json(Path(result['manifest']))
    entry=deck['scenes'][5];meta=loop.gen.geo.load_json(Path(entry['metadata']))
    deck.update(count=1,scenes=[entry]);subset=root/'subset.json';loop.atomic_json(subset,deck)
    return subset,deck,entry,meta,reference


class QualityReplacementContractTests(unittest.TestCase):
    def test_generic_known_payload_is_skipped_without_relaxing_layout(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source,_,_,meta,_=fixture(root)
            first={'proposed':1};known=hashlib.sha256((json.dumps(first,ensure_ascii=False,indent=2)+'\n').encode()).hexdigest()
            def build(spec,config,context,retry):
                proposed=copy.deepcopy(meta);proposed.pop('training_split_overlay');proposed.pop(unit.KEY)
                proposed['ego']['x']+=.01*retry;proposed['retry']=retry
                return {'proposed':retry},proposed
            with patch.object(unit,'load',return_value=dict(assets={})),patch.object(loop.gen,'context',return_value={}), \
                    patch.object(loop.gen,'build',side_effect=build) as generate:
                result=loop.replacement(source,root/'out',known_payloads={known})
            self.assertEqual(generate.call_count,2)
            deck=loop.gen.geo.load_json(result);candidate=deck['scenes'][0]
            self.assertNotEqual(candidate['scenario_sha256'],known)
            saved=loop.gen.geo.load_json(Path(candidate['metadata']))
            self.assertIn(known,saved['replacement']['avoided_payload_sha256'])
            counts=loop.gen.geo.load_json(root/'out/offline_proposals.json')['proposal_counts']
            self.assertEqual(counts['known_payload_rejections'],1)
            self.assertEqual(layout.validate_manifest_overlay(deck,allow_subset=True)['errors'],[])

    def test_all_known_proposals_exhaust_512_without_binding_or_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source,_,_,meta,_=fixture(root);scene={'same':1}
            known=hashlib.sha256((json.dumps(scene,ensure_ascii=False,indent=2)+'\n').encode()).hexdigest()
            with patch.object(unit,'load',return_value=dict(assets={})),patch.object(loop.gen,'context',return_value={}), \
                    patch.object(loop.gen,'build',return_value=(scene,meta)) as generate, \
                    patch.object(layout,'register_recovery_overlay') as bind:
                with self.assertRaises(loop.PlacementProposalExhausted):loop.replacement(source,root/'out',known_payloads={known})
            self.assertEqual(generate.call_count,512);bind.assert_not_called()
            self.assertFalse((root/'out/manifest.json').exists())

    def test_native_history_excludes_only_verified_no_capture_target_miss(self):
        from test_base5_arrival_budget_retry import safe_miss
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);raw=root/'captures/one';loop.atomic_json(raw/'scenario.json',{'native':1})
            loop.atomic_json(raw/'LIDAR_5/native_Instance.txt',{})
            digest=loop.sha256(raw/'scenario.json');row=dict(status='review',capture_root=str(raw),scenario_sha256=digest)
            miss=safe_miss();proof=root/'no_capture.json'
            loop.atomic_json(proof,dict(reason='natural_targets_not_simultaneously_observed',
                scenario_sha256=miss['scenario_sha256'],no_capture_safety=miss['no_capture_safety']))
            miss.update(failure_evidence=str(proof),failure_evidence_sha256=loop.sha256(proof))
            loop.atomic_json(root/'shutdown_safety.json',dict(park_verified=True))
            (root/'ledger.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in (row,miss)))
            attempts=[dict(capture_root=str(root))]
            self.assertEqual(loop.native_payloads(attempts),{digest})
            loop.atomic_json(proof,{})
            with self.assertRaises(loop.ReplacementContractError):loop.native_payloads(attempts)
            (root/'ledger.jsonl').write_text(json.dumps(row)+'\n')
            loop.atomic_json(raw/'scenario.json',{'changed':1})
            with self.assertRaises(loop.ReplacementContractError):loop.native_payloads(attempts)

    def test_known_capture_guard_is_fatal_before_any_native_load(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source,_,entry,_,_=fixture(root)
            with patch.object(loop.subprocess,'Popen') as process:
                self.assertEqual(loop.execute(source,[0],root/'out',known_payloads={entry['scenario_sha256']}),1)
            process.assert_not_called();report=loop.gen.geo.load_json(root/'out/report.json')
            self.assertEqual(report['stage'],'halted');self.assertIn('Native payload already observed',report['error'])
            self.assertEqual(report['selected'],[])

    def test_completed_child_reuse_precedes_known_native_guard(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source,_,entry,meta,_=fixture(root)
            out=root/'out';session=out/entry['id']/'initial';raw=session/'captures'/entry['id']
            audit=dict(review_reasons=[]);loop.atomic_json(raw/'audit.json',audit);loop.atomic_json(raw/'metadata.json',meta)
            loop.atomic_json(session/'session_contract.json',dict(runner_options={}))
            loop.atomic_json(session/'shutdown_safety.json',dict(park_verified=True))
            row=dict(id=entry['id'],scenario_sha256=entry['scenario_sha256'],status='accepted_candidate',
                capture_root=str(raw),audit=audit)
            with patch.object(loop,'prepare_session',return_value=([row],{})),patch.object(loop.subprocess,'Popen') as process:
                self.assertEqual(loop.execute(source,[0],out,resume=True,known_payloads={entry['scenario_sha256']}),0)
            process.assert_not_called();report=loop.gen.geo.load_json(out/'report.json')
            self.assertTrue(report['attempts'][0]['reused_completed_child']);self.assertEqual(len(report['selected']),1)

    def test_inner_replacement_shares_inherited_and_current_native_history(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source,_,entry,_,_=fixture(root)
            row=dict(id=entry['id'],status='review',audit=dict(review_reasons=['primary_no_sensor_evidence']))
            inherited='a'*64;current='b'*64
            with patch.object(loop,'validate_seed',return_value=(row,root,dict(park_verified=True))), \
                    patch.object(loop,'native_payloads',return_value={current}),patch.object(loop,'lightweight_static',return_value=True), \
                    patch.object(loop,'replacement',side_effect=loop.PlacementProposalExhausted('bounded')) as replace, \
                    patch.object(loop.subprocess,'Popen') as process:
                self.assertEqual(loop.execute(source,[0],root/'out',seed_capture=root/'seed',known_payloads={inherited}),0)
            self.assertEqual(replace.call_args.kwargs['known_payloads'],{inherited,current});process.assert_not_called()

    def test_unit_no_native_target_miss_still_allows_same_payload_budget_retry(self):
        import base5_calibration_layout as preparation
        from test_base5_arrival_budget_retry import safe_miss
        class Child:
            returncode=0
            def poll(self):return 0
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source,deck,entry,meta,_=fixture(root)
            deck['config']['capture']=dict(natural_setup_budget_sec=6);loop.atomic_json(source,deck)
            seed=root/'seed';miss=safe_miss();miss.update(id=entry['id'],scenario_sha256=entry['scenario_sha256'])
            proof=seed/'failure.json';loop.atomic_json(proof,dict(reason='natural_targets_not_simultaneously_observed',
                scenario_sha256=miss['scenario_sha256'],no_capture_safety=miss['no_capture_safety']))
            miss.update(failure_evidence=str(proof),failure_evidence_sha256=loop.sha256(proof))
            (seed/'ledger.jsonl').write_text(json.dumps(miss)+'\n')
            loop.atomic_json(seed/'shutdown_safety.json',dict(park_verified=True))
            def budget(manifest,indices,folder,seconds):
                changed=loop.gen.geo.load_json(manifest);changed['config']['capture']['natural_setup_budget_sec']=9
                loop.atomic_json(folder/'manifest.json',changed);return folder/'manifest.json'
            def child(command,**kwargs):
                session=Path(command[command.index('--output')+1]);raw=session/'captures'/entry['id']
                loop.atomic_json(session/'shutdown_safety.json',dict(park_verified=True))
                loop.atomic_json(raw/'metadata.json',meta)
                row=dict(id=entry['id'],scenario_sha256=entry['scenario_sha256'],status='accepted_candidate',
                    capture_root=str(raw),audit=dict(review_reasons=[]))
                (session/'ledger.jsonl').write_text(json.dumps(row)+'\n');return Child()
            with patch.object(loop,'validate_seed',return_value=(miss,seed,dict(park_verified=True))), \
                    patch.object(loop,'budget_retry',side_effect=budget),patch.object(loop,'replacement_contract',return_value={}), \
                    patch.object(preparation,'normalize_prepared_deck',side_effect=lambda path,*a,**kw:path), \
                    patch.object(loop.subprocess,'Popen',side_effect=child) as process:
                self.assertEqual(loop.execute(source,[0],root/'out',seed_capture=root/'seed.json'),0)
            self.assertEqual(process.call_count,1)
            self.assertEqual(loop.gen.geo.load_json(root/'out/report.json')['stage'],'complete')

    def test_unit_reference_mismatch_is_fatal_before_generation(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);mp=root/'meta.json';loop.atomic_json(mp,dict(runtime_actors=[]))
            manifest=root/'manifest.json';loop.atomic_json(manifest,dict(config={unit.KEY:dict(path='/x',sha256='x')},
                scenes=[dict(id='source',metadata=str(mp))]))
            with patch.object(loop.gen,'context') as context:
                with self.assertRaises(loop.ReplacementContractError):loop.replacement(manifest,root/'out')
            context.assert_not_called();self.assertFalse((root/'out').exists())

    def test_unit_reference_is_copied_and_legacy_cannot_gain_it(self):
        reference=dict(path='/unit.json',sha256='pinned');source={unit.KEY:reference};proposed=dict(runtime_actors=[])
        with patch.object(unit,'load',return_value=dict(assets={})):
            loop.preserve_replacement_unit(source,proposed,dict(unit_reference=reference))
        self.assertEqual(proposed[unit.KEY],reference);self.assertIsNot(proposed[unit.KEY],reference)
        with self.assertRaises(loop.ReplacementContractError):
            loop.preserve_replacement_unit({},proposed,dict(unit_reference=None))

    def test_nested_replacement_rebinds_the_immutable_canonical_parent(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source,deck,entry,meta,reference=fixture(root)
            pins={p:loop.sha256(p) for p in (source,Path(entry['metadata']),Path(entry['scenario']),
                Path(deck['split_overlay']['path']))}
            first=copy.deepcopy(meta);first.pop('training_split_overlay');first.pop(unit.KEY)
            first['ego']['x']+=.20;first['retry']=1
            second=copy.deepcopy(first);second['ego']['x']+=.20;second['retry']=2
            with patch.object(unit,'load',return_value=dict(assets={})), \
                    patch.object(loop.gen,'context',return_value={}), \
                    patch.object(loop.gen,'build',side_effect=[({'fresh':1},first),({'fresh':2},second)]):
                a=loop.replacement(source,root/'first');b=loop.replacement(a,root/'second')
            final=loop.gen.geo.load_json(b);candidate=final['scenes'][0];actual=loop.gen.geo.load_json(Path(candidate['metadata']))
            self.assertEqual(actual[unit.KEY],reference)
            self.assertEqual(actual['spec'],meta['spec']);self.assertEqual(actual['split'],meta['split'])
            self.assertEqual(actual['quality_replacement_contract']['root_entry'],entry)
            self.assertEqual(actual['quality_replacement_contract']['ordinal'],2)
            self.assertTrue(final['layout_split_finalization_pending']);self.assertFalse(final['bulk_ready'])
            self.assertEqual(candidate['source_cell_id'],candidate['id'])
            overlay=layout.read_overlay(final['split_overlay']['path'])
            self.assertEqual(overlay['recovery_aliases'][candidate['id']],entry['id'])
            self.assertEqual(layout.validate_manifest_overlay(final,allow_subset=True)['errors'],[])
            self.assertFalse(overlay['audit']['native_recovery_qualified'])
            self.assertTrue(all(loop.sha256(p)==digest for p,digest in pins.items()))

    def test_missing_parent_overlay_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source,deck,entry,meta,_=fixture(root)
            meta.pop('training_split_overlay');loop.atomic_json(Path(entry['metadata']),meta)
            with patch.object(unit,'load',return_value=dict(assets={})),patch.object(loop.gen,'context') as context:
                with self.assertRaises(loop.ReplacementContractError):loop.replacement(source,root/'out')
            context.assert_not_called()

    def test_layout_rebind_errors_are_fatal_not_geometry_exhaustion(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source,_,_,meta,_=fixture(root)
            proposed=copy.deepcopy(meta);proposed.pop('training_split_overlay');proposed.pop(unit.KEY)
            proposed['ego']['x']+=.20;proposed['retry']=1
            with patch.object(unit,'load',return_value=dict(assets={})), \
                    patch.object(loop.gen,'context',return_value={}),patch.object(loop.gen,'build',return_value=({'new':1},proposed)), \
                    patch.object(layout,'register_recovery_overlay',side_effect=ValueError('frozen input changed')):
                with self.assertRaises(loop.ReplacementContractError):loop.replacement(source,root/'out')
            self.assertFalse((root/'out/manifest.json').exists())

    def test_valid_source_layout_rejection_tries_next_geometry(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source,_,_,meta,_=fixture(root);real=layout.register_recovery_overlay
            def build(spec,config,ctx,retry):
                proposed=copy.deepcopy(meta);proposed.pop('training_split_overlay');proposed.pop(unit.KEY)
                proposed['ego']['x']+=.01*retry;proposed['retry']=retry
                return {'new':retry},proposed
            observed=[]
            def bind(*args,**kwargs):
                observed.append(args[2]['id'])
                if len(observed)==1:raise ValueError('Recovery duplicates another actual layout; propose distinct geometry')
                return real(*args,**kwargs)
            with patch.object(unit,'load',return_value=dict(assets={})),patch.object(loop.gen,'context',return_value={}), \
                    patch.object(loop.gen,'build',side_effect=build),patch.object(layout,'register_recovery_overlay',side_effect=bind):
                got=loop.replacement(source,root/'out')
            self.assertTrue(got.exists());self.assertEqual(len(observed),2)
            report=loop.gen.geo.load_json(root/'out/offline_proposals.json')
            self.assertEqual(report['proposal_counts']['geometry'],2)
            self.assertEqual(report['layout_attempts'][0]['status'],'layout_geometry_rejected')

    def test_full_layout_binding_work_is_capped_at_eight(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source,_,_,meta,_=fixture(root)
            def build(spec,config,ctx,retry):
                proposed=copy.deepcopy(meta);proposed.pop('training_split_overlay');proposed.pop(unit.KEY)
                proposed['ego']['x']+=.01*retry;proposed['retry']=retry
                return {'new':retry},proposed
            with patch.object(unit,'load',return_value=dict(assets={})),patch.object(loop.gen,'context',return_value={}), \
                    patch.object(loop.gen,'build',side_effect=build) as generate, \
                    patch.object(layout,'register_recovery_overlay',side_effect=ValueError('Recovery layout leakage/coverage changed: conflict')) as bind:
                with self.assertRaises(loop.PlacementProposalExhausted):loop.replacement(source,root/'out')
            self.assertEqual(bind.call_count,8);self.assertEqual(generate.call_count,8)
            self.assertFalse((root/'out/manifest.json').exists())

    def test_frozen_split_is_not_relaxed_after_512_proposals(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source,_,_,meta,_=fixture(root)
            proposed=copy.deepcopy(meta)
            proposed['split']='val'  # every proposal conflicts with frozen capture split
            with patch.object(unit,'load',return_value=dict(assets={})),patch.object(loop.gen,'context',return_value={}), \
                    patch.object(loop.gen,'build',return_value=({},proposed)) as build:
                with self.assertRaises(loop.PlacementProposalExhausted):loop.replacement(source,root/'out')
            self.assertEqual(build.call_count,512)

    def test_secondary_actor_roster_is_not_relaxed_after_512_proposals(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source,deck,entry,meta,reference=fixture(root)
            meta['runtime_actors']=[dict(data_id=42,source_list='objectList',motion='static'),
                dict(data_id=43,source_list='objectList',motion='static')]
            loop.atomic_json(Path(entry['metadata']),meta)
            proposed=copy.deepcopy(meta);proposed['runtime_actors'][1]['data_id']=44
            context=dict(unit_reference=reference,overlay=True,root_entry=entry)
            with patch.object(loop,'replacement_contract',return_value=context), \
                    patch.object(loop.gen,'context',return_value={}), \
                    patch.object(loop.gen,'build',return_value=({},proposed)) as build, \
                    patch.object(layout,'register_recovery_overlay') as bind:
                with self.assertRaises(loop.PlacementProposalExhausted):loop.replacement(source,root/'out')
            self.assertEqual(build.call_count,512);bind.assert_not_called()
            report=loop.gen.geo.load_json(root/'out/offline_proposals.json')
            self.assertEqual(report['proposal_counts']['roster_rejections'],512)

    def test_prepared_contract_failure_halts_before_capture_with_immutable_source(self):
        import base5_calibration_layout as preparation
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source,deck,entry,meta,_=fixture(root)
            row=dict(id=entry['id'],status='review',audit=dict(review_reasons=['actor_annotation_above_ground'],
                ground_support=dict(actors=[dict(reason='actor_annotation_above_ground',plane={'measured':True})])))
            before={p:loop.sha256(p) for p in (source,Path(entry['scenario']),Path(entry['metadata']))}
            with patch.object(unit,'load',return_value=dict(assets={})), \
                    patch.object(loop,'validate_seed',return_value=(row,root,dict(park_verified=True))), \
                    patch.object(loop,'height_retry',return_value=source), \
                    patch.object(preparation,'normalize_prepared_deck',side_effect=preparation.CalibrationLayoutError('unit/overlay changed')) as normalize, \
                    patch.object(loop.subprocess,'Popen') as process:
                self.assertEqual(loop.execute(source,[0],root/'out',seed_capture=root/'seed'),1)
            process.assert_not_called();normalize.assert_called_once()
            args,kwargs=normalize.call_args
            self.assertEqual(args[1],deck);self.assertEqual(args[2],entry)
            self.assertEqual(args[3].name,'height_deck_contract')
            self.assertIs(kwargs['bind_training'],loop.bind_replacement_contract)
            self.assertEqual(kwargs['context']['root_entry'],entry)
            self.assertTrue(all(loop.sha256(path)==digest for path,digest in before.items()))
            self.assertEqual(loop.gen.geo.load_json(root/'out/report.json')['stage'],'halted')

    def test_preparation_hash_value_error_is_fatal_not_a_terrain_deficit(self):
        import base5_calibration_layout as preparation
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source,_,entry,_,_=fixture(root)
            row=dict(id=entry['id'],status='review',audit=dict(review_reasons=['actor_ground_plane_unverified'],
                ground_support=dict(actors=[])))
            with patch.object(unit,'load',return_value=dict(assets={})), \
                    patch.object(loop,'validate_seed',return_value=(row,root,dict(park_verified=True))), \
                    patch.object(loop,'empty_probe',return_value=source), \
                    patch.object(preparation,'normalize_prepared_deck',side_effect=ValueError('source SHA changed')), \
                    patch.object(loop,'replacement') as replacement,patch.object(loop.subprocess,'Popen') as process:
                self.assertEqual(loop.execute(source,[0],root/'out',seed_capture=root/'seed'),1)
            process.assert_not_called();replacement.assert_not_called()
            report=loop.gen.geo.load_json(root/'out/report.json')
            self.assertEqual(report['stage'],'halted');self.assertIn('source SHA changed',report['error'])

    def test_accepted_status_cannot_adopt_calibration_into_training(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source,_,entry,_,_=fixture(root)
            raw=root/'raw';loop.atomic_json(raw/'metadata.json',dict(calibration_only=True))
            row=dict(id=entry['id'],status='accepted_candidate',capture_root=str(raw),audit=dict(review_reasons=[]))
            with patch.object(loop,'validate_seed',return_value=(row,root,dict(park_verified=True))), \
                    patch.object(loop.subprocess,'Popen') as process:
                self.assertEqual(loop.execute(source,[0],root/'out',seed_capture=root/'seed'),1)
            process.assert_not_called();report=loop.gen.geo.load_json(root/'out/report.json')
            self.assertEqual(report['selected'],[]);self.assertIn('Calibration capture',report['error'])


if __name__=='__main__':unittest.main()
