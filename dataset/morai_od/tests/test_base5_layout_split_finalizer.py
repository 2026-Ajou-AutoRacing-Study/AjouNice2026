"""Selected-layout consolidation, lineage and cross-proposal leakage tests."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import base5_layout_split as layout
from base5_capture_safety import atomic_json,sha256
from finalize_base5_layout_split import finalize
from prepare_base5_layout_split import prepare
from run_base5_fresh_completion import finalize_selected_layout
import test_base5_layout_split as fixtures


class FinalizerTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        source=fixtures.OverlayIOTests().source(self.root)
        result=prepare(source,self.root/'candidate')
        self.source=Path(result['manifest']);self.doc=json.loads(self.source.read_text())
        self.base=layout.read_overlay(result['split_overlay'])
        self.adopted=copy.deepcopy(self.doc);self.rows=[]
        for entry in self.adopted['scenes']:
            raw=self.root/'raw'/entry['id'];raw.mkdir(parents=True)
            (raw/'scenario.json').write_bytes(Path(entry['scenario']).read_bytes())
            (raw/'metadata.json').write_bytes(Path(entry['metadata']).read_bytes())
            atomic_json(raw/'audit.json',dict(review_reasons=[],accepted_candidate=True,components=[]))
            self.rows.append(dict(id=entry['id'],source_cell_id=entry['id'],capture_root=str(raw),
                scenario_sha256=entry['scenario_sha256'],status='accepted_candidate'))
            entry.update(scenario=str(raw/'scenario.json'),metadata=str(raw/'metadata.json'))

    def replacement(self, canonical, x=None):
        source=next(e for e in self.doc['scenes'] if e['id']==canonical)
        original=json.loads(Path(source['metadata']).read_text());proposed=copy.deepcopy(original)
        proposed['ego']['x']=proposed['ego']['x']+.2 if x is None else x
        proposed['fresh_completion_source']=dict(source_cell_id=canonical,
            source_scenario_sha256=source['scenario_sha256'],ordinal=1)
        folder=self.root/'proposals'/canonical;folder.mkdir(parents=True)
        entry=copy.deepcopy(source);entry.update(id=canonical+'_fc01',source_cell_id=canonical+'_fc01',
            scenario=str(folder/'scenario.json'),metadata=str(folder/'metadata.json'))
        atomic_json(Path(entry['scenario']),dict(replacement_of=canonical,x=proposed['ego']['x']))
        entry['scenario_sha256']=sha256(entry['scenario'])
        atomic_json(Path(entry['metadata']),proposed)
        reference=layout.register_recovery_overlay(source,original,entry,proposed,folder/'overlay')
        proposed['training_split_overlay']=reference;atomic_json(Path(entry['metadata']),proposed)
        row=next(r for r in self.rows if r['source_cell_id']==canonical);raw=Path(row['capture_root'])
        (raw/'scenario.json').write_bytes(Path(entry['scenario']).read_bytes())
        atomic_json(raw/'metadata.json',dict(proposed,capture_diagnostic=dict(preserved=True)))
        row.update(id=entry['id'],scenario_sha256=entry['scenario_sha256'])
        adopted=next(e for e in self.adopted['scenes'] if e['id']==canonical)
        adopted['scenario_sha256']=entry['scenario_sha256']
        return reference

    def inputs(self):
        adopted=self.root/'adopted.json';atomic_json(adopted,self.adopted)
        selected=self.root/'selected.jsonl'
        selected.write_text(''.join(json.dumps(r)+'\n' for r in self.rows))
        return adopted,selected

    def test_multiple_proposal_overlays_unify_and_raw_ledger_bytes_are_preserved(self):
        parents=[e['id'] for e in self.doc['scenes'] if self.base['assignments'][e['id']]['split']=='train'][:2]
        for parent in parents:self.replacement(parent)
        adopted,selected=self.inputs()
        originals={p:p.read_bytes() for p in (self.root/'raw').rglob('*') if p.is_file()}
        originals[selected]=selected.read_bytes()
        result=finalize(self.source,adopted,selected,self.root/'final')
        self.assertEqual(result['changed_geometry'],2)
        merged=layout.read_overlay(result['split_overlay'])
        self.assertEqual(merged['audit']['errors'],[])
        self.assertEqual(len(merged['authorized_source_overlays']),3)
        self.assertEqual(layout.validate_manifest_overlay(json.loads(Path(result['manifest']).read_text()))['errors'],[])
        for row in self.rows:
            meta=json.loads((Path(row['capture_root'])/'metadata.json').read_text())
            self.assertIs(layout.overlay_for_metadata(meta,merged),merged)
            self.assertEqual(layout.resolve(merged,row['id'],row,meta)['split'],
                self.base['assignments'][row['source_cell_id']]['split'])
        self.assertEqual(originals,{p:p.read_bytes() for p in originals})
        final_doc=json.loads(Path(result['manifest']).read_text())
        for parent in parents:
            entry=next(e for e in final_doc['scenes'] if e['id']==parent)
            self.assertNotIn('capture_diagnostic',json.loads(Path(entry['metadata']).read_text()))
            self.assertIn('raw_metadata_sha256',merged['selected_lineage'][parent])
        derived=[json.loads(line) for line in Path(result['derived_selected_ledger']).read_text().splitlines()]
        self.assertTrue(all(row['training_split_overlay']['overlay_sha256']==merged['overlay_sha256'] for row in derived))

    def test_finalized_shadow_preserves_exact_prepared_core_and_pins_raw_diagnostics(self):
        import test_base5_shadow_ground_reuse as shadow_fixture
        import base5_shadow_ground_reuse as reuse
        # Supply the full initializer fields needed by the layout allocator;
        # all historical/session metadata are written consistently by the
        # existing immutable shadow fixture, before any registration is made.
        def full_geometry(path,value):
            value=copy.deepcopy(value)
            if isinstance(value,dict) and 'spec' in value and 'ego' in value:
                value['ego'].setdefault('z',0.)
                for actor in value.get('runtime_actors',[])+value.get('ground_probe_source',{}).get('targets',[]):
                    actor.setdefault('estimated_size',[1.,1.,1.]);actor.setdefault('placement_yaw_deg',0.)
            if isinstance(value,dict) and 'scenes' in value:
                for entry in value['scenes']:entry['split']='train'
            atomic_json(path,value)
        root=self.root/'shadow'
        with patch.object(shadow_fixture,'atomic_json',side_effect=full_geometry):
            f=shadow_fixture.fixture(root)
        registered=reuse.register(f.source,[f.selection],root/'registered',f.sensor)
        materialized=reuse.adopt(registered,root/'materialized')
        prepared=prepare(materialized,root/'layout')
        source=Path(prepared['manifest']);document=json.loads(source.read_text());entry=document['scenes'][0]
        raw=root/'native';raw.mkdir()
        meta=json.loads(Path(entry['metadata']).read_text())
        meta.update(capture_runtime_ego=dict(x=meta['ego']['x'],y=meta['ego']['y']),
            capture_runtime_objects=[dict(native_diagnostic=True)])
        atomic_json(raw/'metadata.json',meta);atomic_json(raw/'audit.json',dict(review_reasons=[]))
        (raw/'scenario.json').write_bytes(Path(entry['scenario']).read_bytes())
        original_raw=(raw/'metadata.json').read_bytes()
        adopted=copy.deepcopy(document);adopted['scenes'][0].update(metadata=str(raw/'metadata.json'),scenario=str(raw/'scenario.json'))
        adopted_path=root/'adopted.json';atomic_json(adopted_path,adopted)
        selected=root/'selected.jsonl';selected.write_text(json.dumps(dict(id=entry['id'],source_cell_id=entry['id'],
            status='accepted_candidate',capture_root=str(raw),scenario_sha256=entry['scenario_sha256']))+'\n')
        result=finalize(source,adopted_path,selected,root/'final')
        final_doc=json.loads(Path(result['manifest']).read_text())
        self.assertTrue(reuse.validate_manifest(final_doc,f.sensor))
        self.assertEqual((raw/'metadata.json').read_bytes(),original_raw)
        sidecar=json.loads(Path(final_doc['scenes'][0]['metadata']).read_text())
        self.assertNotIn('capture_runtime_ego',sidecar)
        self.assertEqual(sidecar[reuse.KEY],meta[reuse.KEY])
        meta['capture_runtime_ego']['x']+=1;atomic_json(raw/'metadata.json',meta)
        with self.assertRaisesRegex(ValueError,'lineage changed'):layout.validate_manifest_overlay(final_doc)

    def test_independent_proposals_that_collide_across_splits_are_rejected(self):
        candidates=[e for e in self.doc['scenes'] if float(json.loads(Path(e['metadata']).read_text())['ego']['x'])<10]
        train=next(e for e in candidates if e['derived_training_split']=='train')
        val=next(e for e in candidates if e['derived_training_split']=='val')
        self.replacement(train['id'],x=4.4);self.replacement(val['id'],x=4.4)
        adopted,selected=self.inputs()
        with self.assertRaisesRegex(ValueError,'leak'):finalize(self.source,adopted,selected,self.root/'bad')

    def test_alias_collision_missing_selection_and_native_metadata_tamper_fail_closed(self):
        adopted,selected=self.inputs();original=selected.read_text()
        changed=copy.deepcopy(self.rows);changed[1]['id']=changed[0]['id']
        selected.write_text(''.join(json.dumps(r)+'\n' for r in changed))
        with self.assertRaisesRegex(ValueError,'collision'):finalize(self.source,adopted,selected,self.root/'alias_bad')
        selected.write_text(original)
        raw_meta=Path(self.rows[0]['capture_root'])/'metadata.json';meta=json.loads(raw_meta.read_text())
        meta['spec']['distance_bin']='tampered';atomic_json(raw_meta,meta)
        with self.assertRaisesRegex(ValueError,'non-overlay'):finalize(self.source,adopted,selected,self.root/'metadata_bad')

    def test_original_source_pin_tamper_is_not_hidden_by_new_geometry(self):
        path=Path(json.loads(self.source.read_text())['scenes'][0]['metadata'])
        meta=json.loads(path.read_text());meta['support_note']='changed';atomic_json(path,meta)
        adopted,selected=self.inputs()
        with self.assertRaises(ValueError):finalize(self.source,adopted,selected,self.root/'source_bad')

    def test_unit_reference_survives_finalization_and_native_tag_removal_is_rejected(self):
        import base5_unit_geometry_contract as unit
        from build_base5_mr_asset_catalog import DEFAULT_CATALOG
        native=self.root/'unit_native.json'
        atomic_json(native,dict(assets={'RedBarrel':dict(data_id=40100003,bbox_size_m=[.5727,.5724,.9194])}))
        origin=dict(path=str(native),sha256=sha256(native),assets=['RedBarrel'])
        policy=self.root/'unit_policy.json'
        atomic_json(policy,dict(schema=unit.SCHEMA,training_gt_modified=False,sources=[origin],
            catalog_reference=dict(path=str(DEFAULT_CATALOG.resolve()),sha256=sha256(DEFAULT_CATALOG)),
            assets={'RedBarrel':dict(data_id=40100003,source_list='objectList',raw_class='Obstacle',
                native_size_lwh_m=[.5727,.5724,.9194],reference_source=origin)}))
        reference=dict(path=str(policy),sha256=sha256(policy))
        upstream=self.root/'source'/'manifest.json';document=json.loads(upstream.read_text())
        document['config'][unit.KEY]=reference
        for entry in document['scenes']:
            path=Path(entry['metadata']);meta=json.loads(path.read_text());meta[unit.KEY]=reference;atomic_json(path,meta)
        atomic_json(upstream,document)
        result=prepare(upstream,self.root/'unit_candidate')
        self.source=Path(result['manifest']);self.doc=json.loads(self.source.read_text())
        self.adopted['config']=copy.deepcopy(self.doc['config'])
        self.adopted['split_overlay']=copy.deepcopy(self.doc['split_overlay'])
        for entry in self.doc['scenes']:
            raw=Path(next(r for r in self.rows if r['source_cell_id']==entry['id'])['capture_root'])
            (raw/'metadata.json').write_bytes(Path(entry['metadata']).read_bytes())
        adopted,selected=self.inputs()
        final=finalize(self.source,adopted,selected,self.root/'unit_final')
        merged=json.loads(Path(final['manifest']).read_text())
        self.assertTrue(all(json.loads(Path(e['metadata']).read_text())[unit.KEY]==reference for e in merged['scenes']))
        raw_meta=Path(self.rows[0]['capture_root'])/'metadata.json';meta=json.loads(raw_meta.read_text());meta.pop(unit.KEY)
        atomic_json(raw_meta,meta)
        with self.assertRaisesRegex(ValueError,'unit geometry reference'):
            finalize(self.source,adopted,selected,self.root/'unit_missing')

    def test_completion_finalization_cache_is_read_only_and_detects_raw_mutation(self):
        _,selected=self.inputs()
        result=finalize_selected_layout(self.source,self.adopted,selected,self.root/'completion')
        files={p:p.read_bytes() for p in (self.root/'completion').rglob('*') if p.is_file()}
        self.assertEqual(finalize_selected_layout(self.source,self.adopted,selected,self.root/'completion'),result)
        self.assertEqual(files,{p:p.read_bytes() for p in files})
        raw=Path(self.rows[0]['capture_root'])/'audit.json';atomic_json(raw,dict(review_reasons=['changed']))
        with self.assertRaises(ValueError):finalize_selected_layout(self.source,self.adopted,selected,self.root/'completion')


if __name__=='__main__':unittest.main()
