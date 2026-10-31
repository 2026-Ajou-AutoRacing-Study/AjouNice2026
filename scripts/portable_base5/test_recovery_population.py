"""CPU-only disposable fixtures; frozen q.replacement/campaign control is real."""
import contextlib
import copy
import hashlib
import json
from pathlib import Path
import random
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import recovery_population as population

SCRIPTS = Path(__file__).absolute().parents[2]/'dataset/morai_od/scripts'
sys.path.insert(0,str(SCRIPTS))
import run_base5_quality_loop as quality
import run_base5_capture_campaign as campaign


class RecoveryPopulationTests(unittest.TestCase):
    def fixture(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        actors = [dict(name='primary',data_id=1,source_list='objectList',motion='static'),
                  dict(name='secondary',data_id=2,source_list='objectList',motion='static')]
        metadata = dict(spec=dict(actors=2,role='front',seed=17,population_mode='generic_dense',
            population_max_actors=8,population_budget_bounds=[4,6]),
            runtime_actors=actors,retry=0,split='train')
        source = root/'source.json'
        meta = root/'metadata.json'
        scene = root/'scenario.json'
        meta.write_text(json.dumps(metadata))
        scene.write_text('{"source":true}\n')
        entry = dict(id='cell',metadata=str(meta),scenario=str(scene),
            scenario_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
            family='obstacle',role='front',distance_bounds=[0,30],subfamily=None,
            motion='static',split='train')
        deck = dict(count=1,scenes=[entry],config={},split_overlay={'fixture':True})
        source.write_text(json.dumps(deck))
        fixture = dict(root=root,manifest=source,metadata=metadata,entry=entry,deck=deck,
            frozen=dict(overlay=True,unit_reference=None,root_entry=copy.deepcopy(entry),
                root_metadata=copy.deepcopy(metadata),ordinal=1),
            context={'nested':{'cache':'unchanged'}})
        fixture['before'] = {str(p):p.read_bytes() for p in (source,meta,scene)}
        return fixture

    def candidate(self, fixture, count=2, ids=None):
        value = copy.deepcopy(fixture['metadata'])
        value['spec']['actors'] = count
        ids = ids if ids is not None else list(range(1,count+1))
        value['runtime_actors'] = [dict(name='actor'+str(identity),data_id=identity,
            source_list='objectList',motion='static') for identity in ids]
        return value

    @contextlib.contextmanager
    def core_fixture(self, fixture, builder):
        with patch.object(quality,'replacement_contract',return_value=fixture['frozen']) as contract, \
                patch.object(quality,'bind_replacement_contract') as binding, \
                patch.object(quality.gen,'context',return_value=fixture['context']), \
                patch.object(quality.gen,'build',builder):
            yield contract,binding

    def assert_source_unchanged(self, fixture):
        self.assertEqual({p:Path(p).read_bytes() for p in fixture['before']},fixture['before'])

    def save_metadata_fixture(self, fixture):
        path = Path(fixture['entry']['metadata'])
        path.write_text(json.dumps(fixture['metadata']))
        fixture['before'][str(path)] = path.read_bytes()

    def chooser_builder(self, fixture, observed):
        assets = [dict(data_id=i,name='asset'+str(i),source_list='objectList') for i in (1,2,3,4)]
        pools = dict(obstacle=assets,vehicle=[],person=[])
        geometry = lambda asset:([1.,1.,1.],[0.,0.,0.],True)
        def build(row,configuration,ctx,attempt):
            before = copy.deepcopy(row)
            rng = random.Random(123+attempt)
            reference = random.Random(123+attempt)
            quality.gen.population._choose(assets[0],'obstacle',pools,geometry,dict(row),reference)
            selected = quality.gen.population.choose(assets[0],'obstacle',pools,geometry,row,rng)
            observed.append(dict(input_row=copy.deepcopy(row),input_unchanged=before==row,
                rng_legacy_equal=rng.getstate()==reference.getstate(),selected=[a['data_id'] for a in selected]))
            candidate = self.candidate(fixture,len(selected),[a['data_id'] for a in selected])
            candidate['spec']=copy.deepcopy(row)
            return {'attempt':attempt},candidate
        return build

    def test_actual_replacement_rejects_count_then_roster_and_accepts_unchanged_frozen_spec(self):
        f = self.fixture()
        proposals = [self.candidate(f,3),self.candidate(f,2,[1,7]),self.candidate(f)]
        calls = []
        def build(row,configuration,ctx,attempt):
            calls.append((attempt,id(ctx),id(ctx['nested'])))
            return {'attempt':attempt},proposals[attempt-1]
        old_replacement = quality.replacement
        with self.core_fixture(f,build) as (contract,binding),population.installed(quality) as state:
            result = quality.replacement(f['manifest'],f['root']/'replacement')
            self.assertEqual(state['count_only_rejections'],1)
            self.assertEqual(state['events'][0]['original_actors'],2)
            self.assertEqual(state['events'][0]['proposed_actors'],3)
            self.assertTrue(state['forced_roster'])
            self.assertTrue(state['transient_choose_row_only'])
        self.assertIs(quality.replacement,old_replacement)
        report = json.loads((result.parent/'offline_proposals.json').read_text())
        self.assertEqual(report['proposal_counts']['generation_rejections'],1)
        self.assertEqual(report['proposal_counts']['roster_rejections'],1)
        self.assertEqual(report['geometry_proposals'],3)
        self.assertEqual(report['maximum_geometry_proposals'],512)
        self.assertEqual(report['maximum_layout_bindings'],8)
        self.assertEqual(calls,[(i,id(f['context']),id(f['context']['nested'])) for i in (1,2,3)])
        self.assertEqual(binding.call_count,1)
        self.assertEqual(contract.call_count,1)
        child = json.loads(result.read_text())
        generated = json.loads(Path(child['scenes'][0]['metadata']).read_text())
        self.assertEqual(generated['spec'],f['metadata']['spec'])
        self.assertEqual([a['data_id'] for a in generated['runtime_actors']],[1,2])
        self.assert_source_unchanged(f)

    def test_actual_replacement_exhausts_original_512_bound_without_any_binding(self):
        f = self.fixture()
        build = Mock(side_effect=lambda *args:({'candidate':args[-1]},self.candidate(f,3)))
        with self.core_fixture(f,build) as (_,binding),population.installed(quality) as state:
            with self.assertRaises(quality.PlacementProposalExhausted):
                quality.replacement(f['manifest'],f['root']/'replacement')
            self.assertEqual(state['count_only_rejections'],512)
        self.assertEqual(build.call_count,512)
        binding.assert_not_called()
        report = json.loads((f['root']/'replacement/offline_proposals.json').read_text())
        self.assertEqual(report['proposal_counts']['generation_rejections'],512)
        self.assertEqual(report['status'],'bounded_offline_geometry_exhausted')
        self.assertFalse(report['native_capture_started'])
        self.assert_source_unchanged(f)

    def test_actual_campaign_turns_bounded_exhaustion_into_deficit_not_another_capture(self):
        f = self.fixture()
        build = Mock(side_effect=lambda *args:({'candidate':args[-1]},self.candidate(f,3)))
        def prior_safe_review(manifest,indices,output,**kwargs):
            output.mkdir(parents=True,exist_ok=True)
            (output/'report.json').write_text(json.dumps(dict(stage='complete',attempts=[],selected=[],
                unresolved=[dict(cell='cell',reasons=['population_visible_actor_deficit'])])))
            return 0
        with self.core_fixture(f,build),population.installed(quality), \
                patch.object(campaign,'contract',return_value={'fixture':'same-quota'}), \
                patch.object(campaign,'validate_selected'), \
                patch.object(quality,'execute',side_effect=prior_safe_review) as native:
            code = campaign.execute(f['manifest'],[0],f['root']/'campaign',rounds=2)
        self.assertEqual(code,0)
        status = json.loads((f['root']/'campaign/status.json').read_text())
        self.assertEqual(status['stage'],'complete_with_deficit')
        self.assertEqual(status['review'],1)
        self.assertEqual(status['accepted_candidate'],0)
        self.assertEqual(native.call_count,1)
        self.assertEqual(build.call_count,512)
        self.assert_source_unchanged(f)

    def test_non_actor_spec_change_is_hard_stop_in_actual_replacement(self):
        for actor_change in (False,True):
            with self.subTest(actor_change=actor_change):
                f = self.fixture()
                new = self.candidate(f,3 if actor_change else 2)
                new['spec']['role']='rear'
                build = Mock(return_value=({},new))
                with self.core_fixture(f,build),population.installed(quality):
                    with self.assertRaisesRegex(population.PopulationRecoveryContractError,'outside actors'):
                        quality.replacement(f['manifest'],f['root']/'replacement')
                self.assertEqual(build.call_count,1)
                self.assert_source_unchanged(f)

    def test_invalid_count_types_are_never_geometry_rejections(self):
        for value in (True,2.0,'2',None,-1):
            with self.subTest(value=value):
                f = self.fixture()
                new = self.candidate(f)
                new['spec']['actors']=value
                with self.core_fixture(f,Mock(return_value=({},new))),population.installed(quality):
                    with self.assertRaisesRegex(population.PopulationRecoveryContractError,'nonnegative integer'):
                        quality.replacement(f['manifest'],f['root']/'replacement')

    def test_count_change_cannot_hide_primary_identity_motion_or_scale_change(self):
        for key,value in [('data_id',99),('source_list','pedestrianList'),('motion','moving'),
                ('asset_scale',{'profile_id':'foreign','scale':[2,2,2]})]:
            with self.subTest(key=key):
                f = self.fixture()
                new = self.candidate(f,3)
                new['runtime_actors'][0][key]=value
                with self.core_fixture(f,Mock(return_value=({},new))),population.installed(quality):
                    with self.assertRaisesRegex(population.PopulationRecoveryContractError,'primary identity'):
                        quality.replacement(f['manifest'],f['root']/'replacement')

    def test_count_metadata_mismatch_and_far_count_change_fail_closed(self):
        for kind in ('wrong_length','far'):
            with self.subTest(kind=kind):
                f = self.fixture()
                if kind=='far':
                    f['metadata']['spec']['secondary_far_vehicle']={'frozen':True}
                    Path(f['entry']['metadata']).write_text(json.dumps(f['metadata']))
                new = self.candidate(f,3)
                if kind=='wrong_length':new['runtime_actors'].pop()
                with self.core_fixture(f,Mock(return_value=({},new))),population.installed(quality):
                    with self.assertRaises(population.PopulationRecoveryContractError):
                        quality.replacement(f['manifest'],f['root']/'replacement')

    def test_original_runtime_exception_is_not_reclassified(self):
        f = self.fixture()
        original = RuntimeError('transport/contract is fatal')
        with self.core_fixture(f,Mock(side_effect=original)),population.installed(quality):
            with self.assertRaises(RuntimeError) as caught:
                quality.replacement(f['manifest'],f['root']/'replacement')
        self.assertIs(caught.exception,original)

    def test_original_geometry_exception_retains_its_existing_retry_behavior(self):
        f = self.fixture()
        original = ValueError('Footprint clearance failure')
        build = Mock(side_effect=[original,({'second':True},self.candidate(f))])
        with self.core_fixture(f,build),population.installed(quality) as state:
            result = quality.replacement(f['manifest'],f['root']/'replacement')
        self.assertEqual(build.call_count,2)
        self.assertEqual(state['count_only_rejections'],0)
        self.assertTrue(result.is_file())

    def test_original_source_contract_validation_not_bypassed(self):
        f = self.fixture()
        original = quality.ReplacementContractError('real unit/layout mismatch')
        with self.core_fixture(f,Mock()) as (contract,binding),population.installed(quality):
            contract.side_effect=original
            with self.assertRaises(quality.ReplacementContractError) as caught:
                quality.replacement(f['manifest'],f['root']/'replacement')
        self.assertIs(caught.exception,original)
        binding.assert_not_called()

    def test_later_spatial_build_is_captured_at_replacement_call_and_restored(self):
        f = self.fixture()
        outer = Mock(side_effect=AssertionError('Old outer build must not be used'))
        spatial = Mock(return_value=({},self.candidate(f)))
        with self.core_fixture(f,outer),population.installed(quality):
            self.assertIs(quality.gen.build,outer)
            with patch.object(quality.gen,'build',spatial):
                quality.replacement(f['manifest'],f['root']/'replacement')
                self.assertIs(quality.gen.build,spatial)
            self.assertIs(quality.gen.build,outer)
        spatial.assert_called_once()
        outer.assert_not_called()

    def test_original_chooser_preserves_legacy_rng_and_only_transient_row_gets_frozen_ids(self):
        f = self.fixture()
        observed = []
        original_choose = quality.gen.population.choose
        with self.core_fixture(f,self.chooser_builder(f,observed)), \
                patch.object(quality.gen.population,'choose',wraps=original_choose) as chooser, \
                population.installed(quality) as state:
            result = quality.replacement(f['manifest'],f['root']/'replacement')
            self.assertIs(quality.gen.population.choose,chooser)
        self.assertEqual(state['roster_preserving_choose_calls'],1)
        self.assertEqual(observed,[dict(input_row=f['metadata']['spec'],input_unchanged=True,
            rng_legacy_equal=True,selected=[1,2])])
        self.assertNotIn('frozen_population_data_ids',observed[0]['input_row'])
        self.assertEqual(chooser.call_args.args[4]['frozen_population_data_ids'],[1,2])
        self.assertIsNot(chooser.call_args.args[4],f['metadata']['spec'])
        child=json.loads(result.read_text())
        created=json.loads(Path(child['scenes'][0]['metadata']).read_text())
        self.assertNotIn('frozen_population_data_ids',created['spec'])
        self.assertIs(quality.gen.population.choose,original_choose)
        self.assert_source_unchanged(f)

    def test_existing_exact_frozen_list_is_preserved_and_duplicates_keep_order(self):
        f = self.fixture()
        f['metadata']['runtime_actors'].append(copy.deepcopy(f['metadata']['runtime_actors'][1]))
        f['metadata']['spec'].update(actors=3,population_budget_bounds=[6,8],frozen_population_data_ids=[1,2,2])
        self.save_metadata_fixture(f)
        observed=[]
        with self.core_fixture(f,self.chooser_builder(f,observed)),population.installed(quality):
            result=quality.replacement(f['manifest'],f['root']/'replacement')
        child=json.loads(result.read_text())
        created=json.loads(Path(child['scenes'][0]['metadata']).read_text())
        self.assertEqual([a['data_id'] for a in created['runtime_actors']],[1,2,2])
        self.assertEqual(created['spec']['frozen_population_data_ids'],[1,2,2])
        self.assertTrue(observed[0]['rng_legacy_equal'])
        self.assert_source_unchanged(f)

    def test_mismatched_or_mistyped_existing_frozen_list_is_hard_error(self):
        for ids in ([1,7],[2,1],[True,2],'1,2'):
            with self.subTest(ids=ids):
                f=self.fixture()
                f['metadata']['spec']['frozen_population_data_ids']=ids
                self.save_metadata_fixture(f)
                builder=Mock()
                with self.core_fixture(f,builder),population.installed(quality):
                    with self.assertRaisesRegex(population.PopulationRecoveryContractError,'ordered source roster'):
                        quality.replacement(f['manifest'],f['root']/'replacement')
                builder.assert_not_called()

    def test_original_chooser_pool_budget_vehicle_and_required_secondary_validation_is_not_waived(self):
        for changes in [dict(vehicle_only_population=True),dict(required_secondary_pool='vehicle'),
                dict(population_budget_bounds=[0,1])]:
            with self.subTest(changes=changes):
                f=self.fixture()
                f['metadata']['spec'].update(changes)
                self.save_metadata_fixture(f)
                assets=[dict(data_id=i,name='asset'+str(i),source_list='objectList') for i in (1,2)]
                pools=dict(obstacle=assets,vehicle=[],person=[])
                def build(row,configuration,ctx,attempt):
                    selected=quality.gen.population.choose(assets[0],'obstacle',pools,
                        lambda actor:([1.,1.,1.],None),row,random.Random(attempt))
                    return {},self.candidate(f,len(selected),[a['data_id']for a in selected])
                with self.core_fixture(f,build),population.installed(quality) as state:
                    with self.assertRaises(quality.PlacementProposalExhausted):
                        quality.replacement(f['manifest'],f['root']/'replacement')
                self.assertEqual(state['roster_preserving_choose_calls'],0)
                self.assert_source_unchanged(f)

    def test_chooser_ownership_change_is_detected_and_original_is_restored(self):
        f=self.fixture()
        original=quality.gen.population.choose
        def build(*args):
            quality.gen.population.choose=lambda *unused:[]
            return {},self.candidate(f)
        with self.core_fixture(f,build),population.installed(quality):
            with self.assertRaisesRegex(population.PopulationRecoveryContractError,'chooser adapter ownership'):
                quality.replacement(f['manifest'],f['root']/'replacement')
            self.assertIs(quality.gen.population.choose,original)

    def test_original_input_spec_mutation_fails_and_build_restores(self):
        f = self.fixture()
        def mutate(row,*args):
            row['actors']=3
            return {},self.candidate(f,3)
        with self.core_fixture(f,mutate),population.installed(quality):
            with self.assertRaisesRegex(population.PopulationRecoveryContractError,'mutated'):
                quality.replacement(f['manifest'],f['root']/'replacement')
            self.assertIs(quality.gen.build,mutate)
        self.assert_source_unchanged(f)

    def test_restores_replacement_on_exception_and_refuses_double_install(self):
        original = quality.replacement
        with self.assertRaisesRegex(RuntimeError,'fixture'):
            with population.installed(quality):
                with self.assertRaisesRegex(population.PopulationRecoveryContractError,'already installed'):
                    with population.installed(quality):pass
                raise RuntimeError('fixture')
        self.assertIs(quality.replacement,original)

    def test_replacement_ownership_loss_restores_original_and_raises(self):
        original = quality.replacement
        with self.assertRaisesRegex(population.PopulationRecoveryContractError,'ownership changed'):
            with population.installed(quality):quality.replacement=lambda *args:None
        self.assertIs(quality.replacement,original)

    def test_non_overlay_source_keeps_original_replacement_semantics(self):
        f = self.fixture()
        f['deck'].pop('split_overlay')
        f['manifest'].write_text(json.dumps(f['deck']))
        original = Mock(return_value='legacy-result')
        original._population_recovery_policy = None
        build = quality.gen.build
        with patch.object(quality,'replacement',original),population.installed(quality) as state:
            self.assertEqual(quality.replacement(f['manifest'],f['root']/'unused',known_payloads={'frozen'}),'legacy-result')
            self.assertIs(quality.gen.build,build)
            self.assertEqual(state['replacement_calls'],0)
        original.assert_called_once_with(f['manifest'],f['root']/'unused',known_payloads={'frozen'})


if __name__=='__main__':unittest.main()
