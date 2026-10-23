"""CPU-only environment RGB audit; fixtures never depend on ROS or MORAI."""
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import report_base5_environment_rgb as report


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value) + '\n')
    return path


def capture(root, colour=80, *, moving=False, child=False, ambiguous=False):
    scenario = dict(objectList=[dict(UNIQUEID=8500, name='Box')])
    write(root / 'scenario.json', scenario)
    actor = dict(unique_id=8500, name='Box', motion='moving' if moving else 'static')
    ego = dict(x=1., y=2., z=3., heading=180., vx=0., vy=0., vz=0.)
    write(root / 'metadata.json', dict(runtime_actors=[actor], capture_runtime_ego=ego))
    write(root / 'telemetry.json', dict(ego_before=ego, ego_after=ego))
    write(root / 'sensor_reference.json', dict(cameraList=[dict(m_SensorUniqueID=1, pos={}, rot={}, cc=dict(cameraFOV=90))]))
    for camera in report.CAMERAS:
        folder = root / camera;folder.mkdir()
        rgb = np.full((24, 32, 3), colour, np.uint8);rgb[10:20, 10:20] = colour + 20
        Image.fromarray(rgb).save(folder / 'stamp_Intensity.png')
        Image.fromarray(np.zeros((24, 32), np.uint8)).save(folder / 'stamp_Depth.png')
        instance = np.zeros((24, 32, 4), np.uint8);instance[10:20, 10:20] = [1, 2, 3, 255]
        Image.fromarray(instance).save(folder / 'stamp_Instance.png')
        candidate = -850001 if child else 8500
        (folder / 'stamp_BBox2D.txt').write_text('Obstacle 10 10 20 20 ' + str(candidate) + ' (1, 2, 3, 255)\n')
    components = [dict(parent_uid=8500, raw_class='Obstacle')] * (2 if ambiguous else 1)
    return dict(id=root.name, scenario_sha256=report.sha256(root / 'scenario.json'),
        capture_root=str(root), status='accepted_candidate', audit=dict(components=components))


def selected(path, rows):
    path.write_text(''.join(json.dumps(row) + '\n' for row in rows))
    return path


def test_exact_static_rgb_and_raw_preservation(tmp_path):
    before = capture(tmp_path / 'before', 60)
    after = capture(tmp_path / 'after', 120, ambiguous=True)
    b = selected(tmp_path / 'b.jsonl', [before]);p = selected(tmp_path / 'p.jsonl', [after])
    source = report.snapshot([path for path in tmp_path.rglob('*') if path.is_file()])
    result = report.build(b, p, tmp_path / 'report')
    assert result['compared_static_pairs'] == 1
    assert result['source_snapshot_unchanged']
    assert result['paired_observations_are_independent_training_samples'] is False
    pair = result['pairs'][0]
    assert pair['sensor_geometry_equal']
    assert pair['runtime_ego_pose_delta']['xy_m'] == 0
    assert not pair['photometric_attribution_qualified']
    camera = pair['cameras'][0]
    assert camera['domains']['native_png']['difference']['channel_absolute_error_0_255']['mean'] == 60
    assert camera['foreground'][0]['observations'][0]['native_pixels'] == 100
    assert camera['foreground'][0]['observations'][1]['camera_component_ambiguous']
    assert camera['foreground'][0]['mask_iou'] == 1
    assert camera['foreground'][0]['observations'][0]['model_pixels'] > 0
    for path in result['contact_sheets']:
        with Image.open(path) as image:
            assert image.width == 1920 and image.height > 100
    assert source == report.snapshot(map(Path, source))
    assert json.loads((tmp_path / 'report' / 'report.json').read_text())['raw_modified'] is False


def test_child_id_and_same_geometry_dynamic_excluded(tmp_path):
    before = capture(tmp_path / 'before', child=True)
    after = capture(tmp_path / 'after', child=True, moving=True)
    result = report.build(selected(tmp_path / 'b', [before]), selected(tmp_path / 'p', [after]), tmp_path / 'out')
    assert result['compared_static_pairs'] == 0
    assert result['excluded'][0]['reason'] == 'not_static'
    meta = json.loads((tmp_path / 'before' / 'metadata.json').read_text())
    entries, _ = report.annotations(tmp_path / 'before' / 'CAMERA_1', meta, before)
    assert entries[0]['parent_uid'] == 8500
    assert entries[0]['camera_instance_id'] == -850001


def test_refuse_ambiguous_pair_and_output_overwrite(tmp_path):
    before = capture(tmp_path / 'before');after = capture(tmp_path / 'after')
    b = selected(tmp_path / 'b', [before]);p = selected(tmp_path / 'p', [after])
    duplicate = dict(before, id='duplicate')
    with pytest.raises(ValueError, match='one-to-many'):
        report.ledger(selected(tmp_path / 'duplicates', [before, duplicate]))
    with pytest.raises(ValueError, match='inside a source'):
        report.build(b, p, tmp_path / 'before' / 'bad')
    with pytest.raises(FileExistsError):
        report.build(b, p, tmp_path / 'before')


def test_hash_mismatch_and_pose_wrap(tmp_path):
    before = capture(tmp_path / 'before');after = capture(tmp_path / 'after')
    write(tmp_path / 'after' / 'scenario.json', dict(changed=True))
    with pytest.raises(ValueError, match='payload hash'):
        report.build(selected(tmp_path / 'b', [before]), selected(tmp_path / 'p', [after]), tmp_path / 'out')
    first = dict(x=0, y=0, z=0, heading=179.)
    second = dict(x=.3, y=.4, z=.1, heading=-179.)
    delta = report.pose_delta(first, second)
    assert delta['heading_deg'] == 2
    assert delta['xy_m'] == .5


def test_runtime_speed_excludes_photometric_pair_and_crop(tmp_path):
    before = capture(tmp_path / 'before');after = capture(tmp_path / 'after')
    telemetry = json.loads((tmp_path / 'after' / 'telemetry.json').read_text())
    telemetry['ego_before']['vx'] = .2
    write(tmp_path / 'after' / 'telemetry.json', telemetry)
    result = report.build(selected(tmp_path / 'b', [before]), selected(tmp_path / 'p', [after]), tmp_path / 'out')
    assert result['excluded'][0]['motion'][1]['maximum_observed_speed_mps'] == .2
    assert report.crop_model(np.zeros((720, 1280, 3), np.uint8)).shape == (512, 1056, 3)
    assert report.crop_model(np.zeros((480, 640, 3), np.uint8)).shape == (512, 1056, 3)
    with pytest.raises(ValueError, match='bottom crop'):
        report.crop_model(np.zeros((10, 1280, 3), np.uint8))


def test_annotation_timestamp_and_runtime_objects(tmp_path):
    before = capture(tmp_path / 'before');after = capture(tmp_path / 'after')
    folder = tmp_path / 'after' / 'CAMERA_1'
    (folder / 'stamp_BBox2D.txt').rename(folder / 'wrong_BBox2D.txt')
    with pytest.raises(ValueError, match='Asynchronous'):
        report.build(selected(tmp_path / 'b', [before]), selected(tmp_path / 'p', [after]), tmp_path / 'out')
    first = dict(objects_before=dict(npc_list=[dict(unique_id=3, name='Car', heading=179.,
        position=dict(x=0, y=0, z=0), size={})]))
    second = dict(objects_before=dict(npc_list=[dict(unique_id=3, name='Car', heading=-179.,
        position=dict(x=.3, y=.4, z=0), size={})]))
    result = report.object_pose_audit({}, {}, first, second)
    assert result['objects'][0]['pose_delta']['xy_m'] == .5
    assert result['objects'][0]['pose_delta']['heading_deg'] == 2.


def test_sensor_geometry_tracks_lidar_not_ros_transport():
    first = dict(Lidar3DList=[dict(m_SensorUniqueID=5, pos=dict(x=1), rot={},
        cc=dict(rotationFrequency=15, rosConfig=dict(Topic='/one')))])
    second = json.loads(json.dumps(first))
    second['Lidar3DList'][0]['cc']['rosConfig']['Topic'] = '/two'
    assert report.sensor_geometry(first) == report.sensor_geometry(second)
    second['Lidar3DList'][0]['cc']['rotationFrequency'] = 20
    assert report.sensor_geometry(first) != report.sensor_geometry(second)


def test_background_without_bbox_is_not_missing_gt(tmp_path):
    before = capture(tmp_path / 'before');after = capture(tmp_path / 'after')
    for row in (before, after):
        for camera in report.CAMERAS:
            folder = Path(row['capture_root']) / camera
            (folder / 'stamp_BBox2D.txt').unlink()
            background = np.full((24, 32, 4), 255, np.uint8)
            background[12:] = [0, 0, 0, 255]
            Image.fromarray(background).save(folder / 'stamp_Instance.png')
    result = report.build(selected(tmp_path / 'b', [before]), selected(tmp_path / 'p', [after]), tmp_path / 'out')
    assert result['compared_static_pairs'] == 1
    for camera in result['pairs'][0]['cameras']:
        assert camera['foreground'] == []
        assert camera['bbox2d_files_present'] == [False, False]


def test_missing_bbox_with_foreground_and_missing_depth_fail_closed(tmp_path):
    before = capture(tmp_path / 'before');after = capture(tmp_path / 'after')
    b = selected(tmp_path / 'b', [before]);p = selected(tmp_path / 'p', [after])
    folder = tmp_path / 'after' / 'CAMERA_1'
    (folder / 'stamp_BBox2D.txt').unlink()
    with pytest.raises(ValueError, match='missing despite foreground'):
        report.build(b, p, tmp_path / 'out1')
    (folder / 'stamp_Depth.png').unlink()
    with pytest.raises(ValueError, match='Depth'):
        report.build(b, p, tmp_path / 'out2')


def test_missing_bbox_with_positive_audit_is_not_waived(tmp_path):
    row = capture(tmp_path / 'capture')
    folder = tmp_path / 'capture' / 'CAMERA_1'
    (folder / 'stamp_BBox2D.txt').unlink()
    Image.fromarray(np.full((24, 32, 4), 255, np.uint8)).save(folder / 'stamp_Instance.png')
    row['audit']['components'][0]['cameras'] = [dict(camera='CAMERA_1', native_instance_pixels=5)]
    meta = json.loads((tmp_path / 'capture' / 'metadata.json').read_text())
    with pytest.raises(ValueError, match='missing despite foreground'):
        report.annotations(folder, meta, row)


def test_standing_actor_is_static_but_walking_is_not():
    meta = dict(runtime_actors=[dict(unique_id=8500, motion='standing')])
    assert report.motion(meta, {}, {})['static']
    meta['runtime_actors'][0]['motion'] = 'walking'
    assert not report.motion(meta, {}, {})['static']


def test_output_symlink_parent_cannot_bypass_raw_protection(tmp_path):
    before = capture(tmp_path / 'before');after = capture(tmp_path / 'after')
    alias = tmp_path / 'alias'
    alias.symlink_to(tmp_path / 'before', target_is_directory=True)
    with pytest.raises(ValueError, match='inside a source'):
        report.build(selected(tmp_path / 'b', [before]), selected(tmp_path / 'p', [after]), alias / 'new')
    assert not (tmp_path / 'before' / 'new').exists()


def test_new_source_file_during_analysis_is_detected(tmp_path, monkeypatch):
    before = capture(tmp_path / 'before');after = capture(tmp_path / 'after')
    original = report.camera_report
    def changed(*args, **kwargs):
        value = original(*args, **kwargs)
        (tmp_path / 'before' / 'unexpected.txt').write_text('concurrent writer')
        return value
    monkeypatch.setattr(report, 'camera_report', changed)
    with pytest.raises(RuntimeError, match='inventory changed'):
        report.build(selected(tmp_path / 'b', [before]), selected(tmp_path / 'p', [after]), tmp_path / 'out')
    assert not (tmp_path / 'out' / 'report.json').exists()
