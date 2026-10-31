import copy
from pathlib import Path
from unittest import mock

from test_reinforcement_profile import ReinforcementProfileTests
import reinforcement_profile as r
import reinforcement_screen as screen


class ReinforcementScreenTests(ReinforcementProfileTests):
    """Exercise actual prepare/verify loops with bounded stand-in CPU geometry."""

    def run_screen(self, *, reject_first=False, second_rule=False, only_one_candidate=False):
        rows=[self.row(i) for i in range(32)]
        if only_one_candidate:
            for row in rows[1:]:row['meta']['spec']['mission']=True
        p=copy.deepcopy(self.profile)
        if second_rule:
            p['rules'][0]['assets']['Box']=2
            second=copy.deepcopy(p['rules'][0]);second['id']='second';second['assets']['Box']=2
            p['rules'].append(second)
        initial=r.allocate(rows,p,self.sizes,'a'*64)
        failed=screen.key(initial['assignments'][0]) if reject_first else None
        source=self.root/'source.frozen';r.atomic_json(source,{'immutable':'source'})
        build=self.root/'build.frozen';r.atomic_json(build,{'immutable':'build'})
        profile=self.root/'profile.frozen';r.atomic_json(profile,p)
        request=dict(profile=p,source=r.pin(source),source_build=r.pin(build),profile_file=r.pin(profile))
        by_seed={row['meta']['spec']['seed']:row['meta'] for row in rows}
        calls=[]

        def spatial_build(spec,config,context,retry):
            old=by_seed[spec['seed']]
            ident=f"slot{spec['seed']}";rule=spec['distance_bin'].split('/',1)[1]
            calls.append((ident,rule,retry))
            if (ident,rule)==failed:
                raise ValueError('Known CPU layout conflict; no native input involved')
            metadata=copy.deepcopy(old);metadata.update(spec=copy.deepcopy(spec),retry=retry)
            metadata['runtime_actors'][0]['planned_center_distance_m']=sum(spec['distance_bounds'])/2
            return dict(payload=ident,rule=rule,retry=retry),metadata

        native={'assets':{name:{'native_size_lwh_m':size} for name,size in self.sizes.items()}}
        config={r.unit.KEY:{'fixture':'native'}}
        with mock.patch.object(r.unit,'load',return_value=native), \
             mock.patch.object(r.spatial.gen,'context',return_value={}), \
             mock.patch.object(r,'prewarm',return_value={}), \
             mock.patch.object(r.spatial,'build',side_effect=spatial_build):
            path=screen.prepare(request,rows,config,self.root/'screen',driver=r)
            allocation,cached,pins=screen.verify(request,rows,config,path,driver=r)
        return path,allocation,cached,pins,calls,request,rows,config,native,failed

    def test_actual_prepare_and_verify_cache_all_final_selected_geometry(self):
        path,allocation,cached,pins,calls,*_=self.run_screen()
        self.assertEqual(len(allocation['assignments']),4)
        self.assertEqual(len(cached),4)
        self.assertEqual(len(calls),4)
        self.assertEqual(len(r.read(path)['rounds']),1)
        self.assertIn(str(path),pins)
        self.assertTrue(all(row['accepted'] for row in cached.values()))
        self.assertFalse(r.read(path)['native_outcomes_used_for_selection'])

    def test_actual_rejection_is_recorded_and_same_quota_filled_before_registration(self):
        path,allocation,cached,_,calls,_,_,_,_,failed=self.run_screen(reject_first=True)
        report=r.read(path)
        self.assertEqual(len(allocation['assignments']),4)
        self.assertEqual(report['rejected'],1)
        self.assertEqual(report['cpu_rejected_pairs'],[list(failed)])
        self.assertEqual(len([call for call in calls if call[:2]==failed]),4)
        self.assertNotIn(failed[0],cached)
        self.assertEqual(len(report['rounds']),2)

    def test_rule_specific_cache_can_retest_same_slot_under_different_rule(self):
        path,allocation,cached,_,calls,*_=self.run_screen(reject_first=True,second_rule=True)
        self.assertEqual(len(allocation['assignments']),4)
        self.assertEqual(len({row['id'] for row in allocation['assignments']}),4)
        self.assertTrue(all(cached[row['id']]['assignment']==row for row in allocation['assignments']))
        report=r.read(path)
        self.assertEqual(len({tuple(screen.key(row['assignment'])) for row in report['records']}),len(report['records']))

    def test_tampered_cached_geometry_is_not_reused(self):
        path,_,cached,_,_,request,rows,config,native,_=self.run_screen()
        victim=next(iter(cached.values()))['scenario']['path']
        r.atomic_json(victim,{'changed':True})
        with mock.patch.object(r.unit,'load',return_value=native), \
             self.assertRaisesRegex(ValueError,'cached payload'):
            screen.verify(request,rows,config,path,driver=r)

    def test_all_candidate_failures_remain_on_disk_when_target_cannot_be_filled(self):
        with self.assertRaisesRegex(ValueError,'No eligible'):
            self.run_screen(reject_first=True,only_one_candidate=True)
        ledger=r.read(self.root/'screen'/'trial_ledger.frozen')
        self.assertFalse(ledger['complete'])
        self.assertEqual(len(ledger['records']),1)
        row=ledger['records'][0]
        self.assertFalse(row['accepted'])
        self.assertEqual(row['attempts'],4)
        self.assertEqual(sum(row['rejected_reasons'].values()),4)
        self.assertFalse(ledger['native_outcomes_used_for_selection'])

    def test_unselected_cpu_trial_history_cannot_be_added(self):
        path,_,_,_,_,request,rows,config,native,_=self.run_screen()
        report=r.read(path);report['rounds'][0].reverse();r.atomic_json(path,report)
        with mock.patch.object(r.unit,'load',return_value=native), \
             self.assertRaisesRegex(ValueError,'selection history'):
            screen.verify(request,rows,config,path,driver=r)
