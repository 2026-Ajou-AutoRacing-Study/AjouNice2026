"""Pure continuous replay aggregation; no bag, ROS, model or process startup."""
import replay_signal_bag_continuous_callback as replay


def outcome(lamps,reason='observed_lamps',selection='map_head_association',rescued=False):
    return dict(decision=dict(final_tuples=lamps,near_rescue=dict(attempted=rescued,accepted=rescued),
                observation_timing=dict(output_origin='current_observation')),
                classifier_reason=reason,selection_reason=selection)


def test_next_mission_after_crossing_is_not_previous_mission_unknown_failure():
    green=[['green','circle',None]]
    rows=[dict(t=72.,section='A',group_id=201351,outcomes={'current':outcome(green)}),
          dict(t=75.,section='A',group_id=201365,outcomes={'current':outcome([],reason='no_selected_crop')})]
    report=replay.summarize(rows)['current']['A']
    assert report['counts']['window_frames']==2
    assert report['counts']['outside_active_mission']==1
    assert report['counts']['UNKNOWN']==0


def test_unknown_runs_close_on_real_recovery_and_right_censor_at_window_end():
    rows=[dict(t=t,section='D',group_id=201326,outcomes={'current':outcome(lamps)})
          for t,lamps in ((130.,[]),(130.2,[]),(130.3,[['green','arrow',True]]),(131.,[]))]
    report=replay.summarize(rows)['current']['D']
    assert report['counts']['UNKNOWN']==3
    assert len(report['unknown_runs'])==2
    assert abs(report['unknown_runs'][0]['span_sec']-.3)<1e-9
    assert report['unknown_runs'][1]['right_censored']


def test_detector_cache_is_shared_only_for_same_frame_and_scale():
    class Model:
        def __init__(self):self.model=object();self.calls=[]
        def predict(self,**kw):self.calls.append(kw['imgsz']);return ['result',len(self.calls)]
    model=Model();cache=replay.CachedDetector(model);cache.key=1
    first=cache.predict(imgsz=960)
    assert cache.predict(imgsz=960)==first
    cache.predict(imgsz=640)
    cache.key=2;cache.predict(imgsz=960)
    assert model.calls==[960,640,960]
