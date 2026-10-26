"""Pure continuous replay aggregation; no bag, ROS, model or process startup."""
import hashlib
import json

import pytest

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


def test_custom_b_window_uses_straight_mission_and_does_not_reuse_old_times():
    windows=(('B',20.,25.),)
    rows=[dict(t=22.,section='B',group_id=201365,outcomes={'current':outcome([['green','circle',None]])})]
    report=replay.summarize(rows,windows)['current']
    assert set(report)=={'B'}
    assert report['B']['counts']['known']==1
    assert replay.section(22.,windows)=='B'
    assert replay.section(73.,windows) is None
    assert replay.section(73.)=='A'


@pytest.mark.parametrize('windows', [[],[['B',5,5]],[['E',1,2]],[[[],1,2]],[['A',True,2]],
    [['A',1,float('nan')]],[['A',1,3],['B',2,4]],[['A',1,2],['B',2,3]]])
def test_invalid_or_overlapping_mission_windows_are_rejected(windows):
    with pytest.raises(ValueError):replay.normalize_windows(windows)


def write_map(path,version):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text('<osm><MetaInfo map_version="'+version+'"/></osm>')


def test_legacy_no_options_keep_v87_map_and_original_windows(tmp_path,monkeypatch):
    monkeypatch.setattr(replay,'ROOT',tmp_path)
    path=tmp_path/'kcity_map/lanelet_map/lanelet2_map_v87.osm'
    write_map(path,'87')
    windows,metadata,window_source=replay.replay_configuration()
    assert windows==replay.WINDOWS
    assert metadata['file']==str(path)
    assert metadata['embedded_map_version']=='87'
    assert window_source is None


def test_v94_requires_explicit_map_and_new_windows_and_records_bytes(tmp_path,monkeypatch):
    path=tmp_path/'does_not_infer_version_from_filename.osm';write_map(path,'94')
    with pytest.raises(ValueError,match='explicit --map-file and --windows-json'):
        replay.replay_configuration(path)
    windows_path=tmp_path/'windows.json'
    windows_path.write_text(json.dumps([['B',10,12],['A',1,3],['C',20,22],['D',30,32]]))
    windows,metadata,window_source=replay.replay_configuration(path,windows_path)
    assert [row[0] for row in windows]==list('ABCD')
    assert metadata['embedded_map_version']=='94'
    assert metadata['sha256']==hashlib.sha256(path.read_bytes()).hexdigest()
    assert window_source['sha256']==hashlib.sha256(windows_path.read_bytes()).hexdigest()
    monkeypatch.setattr(replay,'ROOT',tmp_path)
    default_map=tmp_path/'kcity_map/lanelet_map/lanelet2_map_v87.osm';write_map(default_map,'94')
    with pytest.raises(ValueError,match='explicit --map-file'):
        replay.replay_configuration(windows_json=windows_path)


def test_missing_map_metadata_is_not_inferred_from_v94_filename(tmp_path):
    path=tmp_path/'lanelet2_map_v94.osm';path.write_text('<osm/>')
    with pytest.raises(ValueError,match='embedded MetaInfo'):
        replay.replay_configuration(path)


def test_changed_input_is_rejected_and_equal_identity_is_read_only(tmp_path):
    path=tmp_path/'closed-input.json';path.write_bytes(b'original')
    identity=replay.file_identity(path)
    replay.require_unchanged(path,identity)
    path.write_bytes(b'changed input bytes')
    with pytest.raises(RuntimeError,match='Replay input changed'):
        replay.require_unchanged(path,identity)


def test_repeated_mission_windows_are_aggregated_once():
    windows=(('A',1.,2.),('A',4.,5.))
    rows=[dict(t=t,section='A',group_id=201351,outcomes={'current':outcome([])}) for t in (1.,2.,4.,5.)]
    report=replay.summarize(rows,windows)['current']
    assert set(report)=={'A'}
    assert report['A']['counts']['frames']==4
    assert len(report['A']['unknown_runs'])==2
    assert [run['span_sec'] for run in report['A']['unknown_runs']]==[1.,1.]
    assert all(run['right_censored'] for run in report['A']['unknown_runs'])


@pytest.mark.parametrize('threads',[0,9,1.5])
def test_invalid_cpu_thread_limit_is_rejected_before_model_or_ros_import(threads):
    with pytest.raises(ValueError,match='CPU threads'):
        replay.infer(None,None,cpu_threads=threads)
