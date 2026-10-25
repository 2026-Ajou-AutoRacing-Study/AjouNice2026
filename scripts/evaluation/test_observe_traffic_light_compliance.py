"""Pure geometry/GT evidence tests. Never imports ROS/grpc or runs a live node."""
import ast
import hashlib
import json
import math
from pathlib import Path
import tempfile
import types

import pytest

import observe_traffic_light_compliance as observer


def native(identifier='D', color=33, start=9.9, end=9.92, valid=True, record_id=1):
    return dict(native_id=identifier, color=color, request_monotonic=start,
                response_monotonic=end, valid=valid, record_id=record_id)


def test_crossing_is_a_directed_finite_segment_not_an_infinite_line():
    result = observer.segment_crossing([0, -2], [0, 2], [-3, 0], [3, 0])
    assert result['point_xy_m'] == [0, 0]
    assert result['motion_fraction'] == .5
    assert result['direction'] == 'negative_to_positive'
    assert observer.segment_crossing([0, -2], [0, 2], [-3, 0], [3, 0], 'positive_to_negative') is None
    assert observer.segment_crossing([4, -2], [4, 2], [-3, 0], [3, 0]) is None
    assert observer.segment_crossing([0, 2], [0, 3], [-3, 0], [3, 0]) is None
    reverse = observer.segment_crossing([0, 2], [0, -2], [-3, 0], [3, 0], 'positive_to_negative')
    assert reverse['direction'] == 'positive_to_negative'


def test_collinear_motion_and_repeated_line_endpoint_do_not_duplicate_crossing():
    assert observer.segment_crossing([-1, 0], [1, 0], [-3, 0], [3, 0]) is None
    assert observer.segment_crossing([0, -1], [0, 0], [-3, 0], [3, 0]) is not None
    assert observer.segment_crossing([0, 0], [0, 1], [-3, 0], [3, 0]) is None
    with pytest.raises(ValueError): observer.segment_crossing([0, -1], [0, 1], [0, 0], [0, 0])


def test_front_point_uses_raw_heading_correction_once_and_existing_transform():
    point = observer.front_point_map([1, 2, 3], 1.347, [10, 20, 0], [0, 0, 0, 1], 3.)
    assert point == [14., 22., 3.]
    q = [0, 0, math.sin(math.pi/4), math.cos(math.pi/4)]
    rotated = observer.front_point_map([1, 2, 3], 1.347, [10, 20, 0], q, 3.)
    assert rotated == pytest.approx([8, 24, 3])
    translation, inverse = observer.inverse_transform([10, 20, 0], q)
    recovered = [a+b for a, b in zip(observer.rotate(rotated, inverse), translation)]
    assert recovered == pytest.approx([4, 2, 3])


def test_native_exact_before_after_agreement_uses_bounded_ages():
    samples = [native(), native(start=10.08, end=10.1, record_id=2)]
    result = observer.native_bracket(samples, 'D', 10., .25)
    assert result['verified'] and result['color'] == 33
    assert result['before_record_id'] == 1 and result['after_record_id'] == 2
    stale = [native(start=9.75, end=9.8), samples[1]]
    assert not observer.native_bracket(stale, 'D', 10., .25)['verified']
    assert observer.native_bracket(stale, 'D', 10., .5)['verified']
    stale = [native(start=9.5, end=9.8), samples[1]]
    assert not observer.native_bracket(stale, 'D', 10., .5)['verified']


def test_bracket_never_extrapolates_one_side_or_state_transition():
    before, after = native(), native(start=10.08, end=10.1, color=16, record_id=2)
    assert not observer.native_bracket([before], 'D', 10., .25)['verified']
    assert not observer.native_bracket([after], 'D', 10., .25)['verified']
    result = observer.native_bracket([before, after], 'D', 10., .25)
    assert not result['verified'] and result['reason'] == 'native_state_transition_crosses_bracket'
    # A request which straddled the crossing is not a verified before/after read.
    overlap = native(start=9.99, end=10.01)
    assert not observer.native_bracket([overlap, after], 'D', 10., .25)['verified']


def test_intervening_native_failure_is_preserved_not_silently_bridged():
    samples = [native(), native(start=9.95, end=9.99, valid=False, record_id=2),
               native(start=10.08, end=10.1, record_id=3)]
    result = observer.native_bracket(samples, 'D', 10., .25)
    assert not result['verified'] and result['failed_record_ids'] == [2]


def test_permission_proxy_is_separate_from_unknown_and_C_head_disagreement():
    event = dict(estimated_source_monotonic=10., pose_time_verified=True)
    group = dict(group_id=201326, native_ids=['D'])
    for color, expected in ((33, True), (16, False), (5, False)):
        samples = [native(color=color), native(color=color, start=10.08, end=10.1, record_id=2)]
        result = observer.evaluate_crossing(event, samples, group)['0.25']
        assert result['verified'] and result['route_permission_proxy'] is expected
        stale_pose = {**event, 'pose_time_verified': False}
        assert observer.evaluate_crossing(stale_pose, samples, group)['0.25']['route_permission_proxy'] is None
    group = dict(group_id=201290, native_ids=['C1', 'C2'])
    samples = [native(identifier=head, color=color, start=start, end=end, record_id=index)
               for index, (head, color, start, end) in enumerate([
                   ('C1', 16, 9.9, 9.92), ('C1', 16, 10.08, 10.1),
                   ('C2', 1, 9.9, 9.92), ('C2', 1, 10.08, 10.1)])]
    assert not observer.evaluate_crossing(event, samples, group)['0.25']['verified']


def test_geometry_requires_exact_four_mission_ids_and_map_coordinates():
    document = dict(frame='map', groups=[dict(group_id=group, native_ids=list(ids),
        stop_line_xy=[[-1, 0], [1, 0]]) for group, ids in observer.GROUP_IDS.items()])
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)/'geometry.json'
        path.write_text(json.dumps(document))
        groups, metadata = observer.load_geometry(path)
        assert len(groups) == 4 and len(metadata['sha256']) == 64
        document['groups'][2]['native_ids'] = ['C1256W000016']
        path.write_text(json.dumps(document))
        with pytest.raises(ValueError, match='Exact mission native IDs'): observer.load_geometry(path)
        document['frame'] = 'morai_map'
        path.write_text(json.dumps(document))
        with pytest.raises(ValueError, match='map-frame'): observer.load_geometry(path)


def test_primitive_and_JSONL_keep_unique_evidence_ids_and_unknown_values():
    assert observer.primitive({'bad': float('nan'), 'known': False}) == {'bad': None, 'known': False}
    assert observer.stamp_ns(types.SimpleNamespace(sec=1792918944, nanosec=123456789)) == 1792918944123456789
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)/'observations.jsonl'
        writer = observer.JsonlWriter(path)
        assert writer.write({'record_id': 999, 'event': 'test'}) == 1
        assert writer.write({'record_id': 1, 'event': 'test2'}) == 2
        writer.close()
        assert [row['record_id'] for row in map(json.loads, path.read_text().splitlines())] == [1, 2]


def test_source_exposes_only_subscriptions_and_read_only_native_transport():
    tree = ast.parse(Path(observer.__file__).read_text())
    names = {node.func.attr for node in ast.walk(tree) if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)}
    assert 'Subscriber' in names
    assert not names.intersection({'Publisher', 'ServiceProxy', 'Popen', 'run_process', 'set_traffic_light'})


def crossing(source=1000000000, index=1):
    return dict(record_id=index, estimated_source_stamp_ns=source,
                estimated_source_monotonic=10., pose_time_verified=True)


def jpeg(ring, source, receipt=10., payload=b'original JPEG fixture bytes'):
    ring.add(payload, source, source+10000000 if source else 1000000000,
             receipt, frame_id='camera_front', format='jpeg', width=1280, height=720)


def test_JPEG_ring_is_bounded_by_both_memory_count_and_receipt_age(tmp_path):
    ring = observer.CrossingJpegRing(tmp_path/'images')
    for index in range(100): jpeg(ring, 1000000000+index*1000000, receipt=index*.001)
    assert len(ring.frames) == 20
    jpeg(ring, 2000000000, receipt=1.)
    assert len(ring.frames) == 1
    assert not (tmp_path/'images').exists()  # No crossing, no persisted camera stream.


def test_crossing_pair_keeps_nearest_original_bytes_hashes_and_source_clock_deltas(tmp_path):
    emitted, ring = [], observer.CrossingJpegRing(tmp_path/'images', lambda row: emitted.append(row))
    jpeg(ring, 920000000, receipt=9.92, payload=b'earlier')
    jpeg(ring, 970000000, receipt=9.97, payload=b'nearest before original')
    event = crossing()
    ring.begin(event)
    jpeg(ring, 1040000000, receipt=10.04, payload=b'first after')
    assert event['crossing_jpeg_bracket']['status'] == 'pending'
    # Include a closer late source image before the .1s watermark closes.
    jpeg(ring, 1020000000, receipt=10.05, payload=b'nearest after original')
    jpeg(ring, 1120000000, receipt=10.12)
    result = event['crossing_jpeg_bracket']
    assert result['status'] == 'complete_original_pair' and result['timing_bracket_verified']
    assert Path(result['before']['path']).read_bytes() == b'nearest before original'
    assert Path(result['after']['path']).read_bytes() == b'nearest after original'
    assert result['before']['sha256'] == hashlib.sha256(b'nearest before original').hexdigest()
    assert result['before']['source_delta_to_crossing_sec'] == -.03
    assert result['after']['source_delta_to_crossing_sec'] == .02
    assert result['native_GT_replaced'] is False and result['independent_pixel_review_required']
    assert len(list((tmp_path/'images').glob('*.jpg'))) == 2 and len(emitted) == 1


def test_JPEG_gap_over_point_one_seconds_is_missing_not_nearest_extrapolation(tmp_path):
    ring = observer.CrossingJpegRing(tmp_path/'images')
    jpeg(ring, 899999999, receipt=9.9)
    event = crossing()
    ring.begin(event)
    jpeg(ring, 1100000001, receipt=10.11)
    result = event['crossing_jpeg_bracket']
    assert result['status'] == 'missing_side'
    assert result['before']['status'] == result['after']['status'] == 'missing'
    assert not result['timing_bracket_verified']
    assert not (tmp_path/'images').exists()


def test_exact_point_one_seconds_boundary_is_allowed_and_close_saves_partial_evidence(tmp_path):
    ring = observer.CrossingJpegRing(tmp_path/'images')
    jpeg(ring, 900000000, receipt=9.9)
    event = crossing()
    ring.begin(event)
    jpeg(ring, 1100000000, receipt=10.1)
    ring.close()
    assert event['crossing_jpeg_bracket']['status'] == 'complete_original_pair'
    assert event['crossing_jpeg_bracket']['selection_closed_reason'] == 'observer_closed'


def test_no_after_frame_on_close_retains_before_and_explicit_missing(tmp_path):
    ring = observer.CrossingJpegRing(tmp_path/'images')
    jpeg(ring, 950000000, receipt=9.95)
    event = crossing()
    ring.begin(event)
    ring.close()
    result = event['crossing_jpeg_bracket']
    assert result['before']['status'] == 'saved_original_JPEG'
    assert result['after']['status'] == 'missing'
    assert not result['timing_bracket_verified']


def test_unstamped_crossing_or_same_instant_single_image_is_not_two_sided_GT(tmp_path):
    ring = observer.CrossingJpegRing(tmp_path/'images')
    event = crossing(source=0)
    ring.begin(event)
    assert event['crossing_jpeg_bracket']['reason'] == 'crossing_source_stamp_unverified'
    jpeg(ring, 1000000000)
    coincident = crossing(index=2)
    ring.begin(coincident)
    ring.close()
    assert not coincident['crossing_jpeg_bracket']['timing_bracket_verified']
    assert coincident['crossing_jpeg_bracket']['before']['status'] == 'missing'


def test_saved_images_do_not_override_native_UNDEFINED_or_bad_pose_time(tmp_path):
    ring = observer.CrossingJpegRing(tmp_path/'images')
    jpeg(ring, 950000000)
    event = crossing()
    event['pose_time_verified'] = False
    ring.begin(event)
    jpeg(ring, 1050000000)
    ring.close()
    assert not event['crossing_jpeg_bracket']['timing_bracket_verified']
    event['group_id'] = 201326
    result = observer.evaluate_crossing(event, [], dict(group_id=201326, native_ids=['D']))
    assert result['0.25']['route_permission_proxy'] is None
    assert not result['0.5']['verified']
