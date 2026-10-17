"""격리 후보는 기존 raw/QA를 바꾸지 않고 좁은 불확실성만 비교한다."""
import copy
import itertools
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import base5_static_surface_attitude_candidate as candidate


def write(path, value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value)+'\n')
    return path


@pytest.fixture
def association():
    map_hashes = dict(source_map_link_sha256='a'*64, source_map_boundary_sha256='b'*64)
    row = dict(uid=8500, verified=False, reason='actor_annotation_deep_below_ground',
        minimum_annotation_ground_gap_m=-6., plane=dict(coefficients=[0.,0.,6.],inliers=22,residual_rms_m=.01),
        surface_observation=dict(actor_center_ego_xy_m=[42.,0.],patch_point_count=46,lower_by_one_m_count=6,
            minimum_signed_plane_residual_m=-6.,native_parent_component_count=1))
    road = dict(policy='mgeo-segment-associated-boundary-v1',usable=True,confidence='road_strip',reason=None,
        height_m=0.,nearest_road_distance_m=.1,query_xy_m=[42.,0.],**map_hashes,
        footprint=dict(all_supported=True,map_height_span_m=.01,
            samples=[dict(usable=True,confidence='road_strip',height_m=0.)]*5))
    actor = dict(name='Box',unique_id=8500,motion='static',source_list='objectList',
        position=dict(x=42.,y=0.,z=0.),geometric_center_estimate=[42.,0.],map_ground_initializer=road)
    meta = dict(ground_support_check=dict(static_policy='static-single-capture-advisory-v115'),runtime_actors=[actor])
    observation = dict(native_parent_component_count=1,raw_class='Obstacle',native_world_center_xy_m=[42.,0.],
        lowest_native_world_z_m=.001,legacy_yaw_only_lowest_world_z_m=.35)
    return row,dict(max_range_m=20.),meta,observation,np.eye(3),dict(z=0.),map_hashes


def classify(fixture, **kwargs):
    row,ground,meta,observation,rotation,ego,map_hashes = fixture
    return candidate.candidate_association(row,ground,meta,observation,rotation,ego,map_hashes=map_hashes,**kwargs)


def test_existing_thresholds_accept_only_uncertainty_not_contact_or_approval(association):
    original = copy.deepcopy(association)
    result = classify(association)
    assert result['eligible']
    assert result['native_lowest_corner_minus_road_level_m'] == .001
    assert result['original_plane_and_gap_preserved']
    assert not result['verified_ground_or_mesh_contact'] and not result['proposed_native_approval']
    for index in (0,1,2,3,5,6):assert association[index] == original[index]


@pytest.mark.parametrize('kind', ['near','above','below','notsevere','dense','too_sparse','no_lower','bad_rms',
    'compound','multi_component','dynamic','bad_map','bad_footprint','world_xy','reference'])
def test_original_severe_near_dynamic_compound_and_provenance_guards_remain(association,kind):
    row,ground,meta,observation,_,_,_ = association
    if kind == 'near':row['surface_observation']['actor_center_ego_xy_m']=[15.,0.]
    if kind == 'above':row['reason']='actor_annotation_above_ground'
    if kind == 'below':observation['lowest_native_world_z_m']=-.5
    if kind == 'notsevere':row['minimum_annotation_ground_gap_m']=-.39
    if kind == 'dense':row['surface_observation']['patch_point_count']=65
    if kind == 'too_sparse':row['surface_observation']['patch_point_count']=17
    if kind == 'no_lower':row['surface_observation']['lower_by_one_m_count']=1
    if kind == 'bad_rms':row['plane']['residual_rms_m']=.026
    if kind == 'compound':meta['runtime_actors'][0]['name']='BabyStroller3'
    if kind == 'multi_component':observation['native_parent_component_count']=2
    if kind == 'dynamic':meta['runtime_actors'][0]['motion']='walking'
    if kind == 'bad_map':meta['runtime_actors'][0]['map_ground_initializer']['source_map_link_sha256']='c'*64
    if kind == 'bad_footprint':meta['runtime_actors'][0]['map_ground_initializer']['footprint']['all_supported']=False
    if kind == 'world_xy':observation['native_world_center_xy_m']=[43.,0.]
    if kind == 'reference':row['used_independent_reference']=True
    assert not classify(association)['eligible']


def test_static_pedestrian_extension_is_separate_explicit_opt_in(association):
    association[2]['runtime_actors'][0].update(name='Woman4',source_list='pedestrianList')
    association[3]['raw_class']='Pedestrian'
    assert not classify(association)['eligible']
    assert classify(association,allow_pedestrians=True)['eligible']


def test_all_eight_corners_rotated_not_mean_xy_plus_low_z():
    corners = np.asarray(list(itertools.product((-1.,1.),(-1.,1.),(0.,2.))))
    record = dict(corners_lidar_m=corners.tolist(),annotation_center_lidar_m=[0.,0.,1.],
        annotation_rotation_rpy=[0.,0.,0.],rotation_rpy=[0.,0.,0.],annotation_size_m=[2.,2.,2.],parent_component_count=1,
        raw_class='Obstacle',source_files={})
    rotation = candidate.rotation_matrix([.1,.2,0.])
    observed = candidate.world_corroboration(record,np.eye(4),rotation,dict(x=0.,y=0.,z=0.))
    assert np.allclose(observed['corners_world_m'],corners@rotation.T)
    assert observed['lowest_native_world_z_m'] == pytest.approx((corners@rotation.T)[:,2].min())
    assert observed['lowest_native_world_z_m'] < -.2
    assert observed['all_eight_measured_corners_transformed']


def test_quaternion_must_be_finite_and_already_near_unit():
    assert np.allclose(candidate.quaternion_matrix([0,0,0,1]),np.eye(3))
    for q in ([0,0,0,0],[0,0,0,2],[0,0,float('nan'),1]):
        with pytest.raises(ValueError):candidate.quaternion_matrix(q)


@pytest.fixture
def orientation(tmp_path):
    root=tmp_path/'capture';(root/'LIDAR_5').mkdir(parents=True);(root/'IMU_7').mkdir()
    (root/'LIDAR_5/stamp_Instance.txt').write_text('native companion selected by basename\n')
    yaw=10.;q=[0,0,math.sin(math.radians(yaw)/2),math.cos(math.radians(yaw)/2)]
    fields=dict(zip(('Orientation '+v for v in 'XYZW'),q))
    fields.update({'Angular Velocity '+v:0. for v in 'XYZ'})
    fields.update({'TimeStamp.Secs':123,'TimeStamp.nSecs':2102888896})
    imu=root/'IMU_7/stamp.txt'
    imu.write_text('\n'.join(k+' : '+str(v) for k,v in fields.items()))
    ego=dict(x=0.,y=0.,z=0.,heading=yaw,vx=0.,vy=0.,vz=0.,brake=1.)
    meta=dict(capture_runtime_ego=ego,lidar_to_ego=np.eye(4).tolist(),
        capture_safety=dict(collision=dict(observed=True,no_collision_verified=True,non_ego_uids=[])))
    telemetry=dict(ego_before=ego,ego_after=copy.deepcopy(ego),park_confirmed_monotonic=1.,
        post_park_stationary_witness=dict(verified=True),pre_capture_stationary_witness=dict(verified=True))
    scenario=dict(egoVehicle=dict(initPosition=dict(gear=1)))
    sensor=lambda uid:dict(m_SensorUniqueID=uid,m_TargetUniqueID=0,
        pos=dict(x=0.,y=0.,z=0.),rot=dict(roll=0.,pitch=0.,yaw=0.))
    sensors=dict(IMUList=[sensor(7)],Lidar3DList=[sensor(5)])
    return root,meta,telemetry,scenario,sensors,imu


def test_native_heading_relation_and_one_yaw_correction_with_bad_timestamp_preserved(orientation):
    root,meta,telemetry,scenario,sensor,_=orientation
    rotation,_,proof=candidate.attitude(root,meta,telemetry,scenario,sensor)
    assert math.degrees(math.atan2(rotation[1,0],rotation[0,0])) == pytest.approx(10.-1.347)
    assert proof['raw_timestamp_fields']['nanoseconds']=='2102888896'
    assert not proof['raw_nanoseconds_in_ros_range']
    assert not proof['exact_source_time_synchronization_claimed']
    assert proof['heading_relationship_verified'] and proof['correction_applied_once']


def test_imu_sensor_mount_removed_before_heading_comparison(orientation):
    root,meta,telemetry,scenario,sensor,_=orientation
    sensor['IMUList'][0]['rot']['yaw']=2.
    telemetry['ego_before']['heading']=8.
    telemetry['ego_after']['heading']=8.
    rotation,_,proof=candidate.attitude(root,meta,telemetry,scenario,sensor)
    assert math.degrees(math.atan2(rotation[1,0],rotation[0,0])) == pytest.approx(8.-1.347)
    assert proof['heading_relationship_verified']


def test_empty_background_has_valid_attitude_without_invented_corner_components(orientation):
    root,meta,telemetry,scenario,sensor,_=orientation
    meta.update(runtime_actors=[],spec=dict(family='background'))
    (root/'LIDAR_5/stamp_Instance.txt').write_text('')
    (root/'LIDAR_5/stamp_Instance_8Points.txt').write_text('')
    for filename,value in [('metadata.json',meta),('telemetry.json',telemetry),('scenario.json',scenario),
        ('sensor_reference.json',sensor),('audit.json',dict(accepted_candidate=True,review_reasons=[],components=[]))]:
        write(root/filename,value)
    result=candidate.analyze(root)
    assert result['attitude_qualified'] and result['empty_background_native_records']
    assert not result['classification_changed'] and result['actors']==[]


@pytest.mark.parametrize('kind',['different_stamp','yaw_offset','not_park','moving_ego','gyro','collision','extrinsics'])
def test_unconfirmed_attitude_or_safety_fails_closed(orientation,kind):
    root,meta,telemetry,scenario,sensors,imu=orientation
    if kind=='different_stamp':imu.rename(imu.with_name('wrong.txt'))
    if kind=='yaw_offset':telemetry['ego_before']['heading']+=1.347
    if kind=='not_park':scenario['egoVehicle']['initPosition']['gear']=4
    if kind=='moving_ego':telemetry['ego_after']['vx']=.1
    if kind=='gyro':imu.write_text(imu.read_text().replace('Angular Velocity X : 0.0','Angular Velocity X : 0.1'))
    if kind=='collision':meta['capture_safety']['collision']['non_ego_uids']=[1]
    if kind=='extrinsics':sensors['Lidar3DList'][0]['pos']['z']=1.
    with pytest.raises(ValueError):candidate.attitude(root,meta,telemetry,scenario,sensors)


def test_output_refuses_symlink_parent_into_raw_and_preserves_source(tmp_path,monkeypatch):
    root=tmp_path/'raw';root.mkdir();write(root/'audit.json',dict(original=True))
    alias=tmp_path/'alias';alias.symlink_to(root,target_is_directory=True)
    before=(root/'audit.json').read_bytes()
    with pytest.raises(ValueError,match='outside source'):
        candidate.build([root],alias/'bad')
    assert (root/'audit.json').read_bytes()==before and not (root/'bad').exists()


def test_source_snapshot_changed_is_not_report_qualified(tmp_path,monkeypatch):
    root=tmp_path/'raw';root.mkdir();source=write(root/'audit.json',dict(original=True))
    def changed(*args,**kwargs):
        write(source,dict(changed=True))
        return dict(candidate_classification='unchanged',classification_changed=False)
    monkeypatch.setattr(candidate,'analyze',changed)
    with pytest.raises(ValueError,match='Source files changed'):
        candidate.build([root],tmp_path/'out')
    assert not (tmp_path/'out').exists()


@pytest.mark.parametrize('collision', ['directory', 'symlink'])
def test_output_created_during_analysis_is_never_overwritten(tmp_path, monkeypatch, collision):
    root = tmp_path/'raw'
    source = write(root/'audit.json', dict(original=True))
    output, protected = tmp_path/'out', tmp_path/'other_report'
    source_before = source.read_bytes()

    def competing_analysis(*args, **kwargs):
        if collision == 'directory':
            output.mkdir()
        else:
            protected.mkdir()
            output.symlink_to(protected, target_is_directory=True)
        write(output/'report.json', dict(owner='other execution'))
        return dict(candidate_classification='unchanged', classification_changed=False)

    monkeypatch.setattr(candidate, 'analyze', competing_analysis)
    with pytest.raises(FileExistsError):
        candidate.build([root], output)
    assert candidate.read(output/'report.json') == dict(owner='other execution')
    assert source.read_bytes() == source_before
