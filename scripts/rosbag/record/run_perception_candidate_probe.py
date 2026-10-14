#!/usr/bin/env python3
"""Bounded MORAI raw-input QA; explicit --run, no training or bulk capture.

Owns UDP/TF/brake/recorder processes only, uses existing validator loader,
and installs uniquely named scene copies without editing validator sources.
Candidate grouping never changes sensor RPM, stamp policy or simulator time.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
VALIDATOR = ROOT/'base_ws/src/planning_addons/planning_scenario_validator'
NETWORK = VALIDATOR/'config/internal/generated/ego_network_validator_local.json'
INSTALL = Path.home()/'MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025'
ARCHIVE = Path.home()/'morai_cap/base5_v2_261008/manual_ground_archive_v116/corrected/1.json'
LOOKUP = Path.home()/'morai_cap/base5_v2_261008/endpoint_linear_release56_v117/height_lookup_live.json'
PET_TEMPLATE = ARCHIVE.with_name('5.json')
POLICIES = {'fixed51': ['lidar_npackets:=51'], 'fixed50': ['lidar_npackets:=50'], 'cut0': ['lidar_cut_angle:=0']}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def install_stem(output, index):
    # MORAI SceneLoad encodes the stem in a fixed 30-byte field.
    token = hashlib.sha256(str(output.resolve()).encode()).hexdigest()[:8]
    stem = f'0_b5p_{token}_{index:02d}'
    if len(stem.encode()) > 30:
        raise ValueError('SceneLoad filename exceeds 30 bytes')
    return stem


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def vector(point):
    return {k: float(point[k]) for k in 'xyz'}


def unity(point):
    return dict(x=-float(point['x']), y=float(point['z']), z=-float(point['y']))


def pedestrian(template, identity, name, data_id, profile, offset=0., reverse=False):
    """Use validated asset endpoints; don't extrapolate heights or terrain."""
    knots = profile['knots']
    points = [vector(knots[0]), vector(knots[-1])]
    dx, dy = points[1]['x']-points[0]['x'], points[1]['y']-points[0]['y']
    length = math.hypot(dx, dy)
    if length < 4. or not profile.get('live_operational_qa_passed'):
        raise ValueError('Require previously live-qualified two-point path')
    # Parallel offsets remain within the lookup's authored transfer envelope.
    if abs(offset) > min(1., profile.get('maximum_path_offset_m', 0.)):
        raise ValueError('Offset exceeds the height profile envelope')
    for point in points:
        point['x'] -= offset*dy/length
        point['y'] += offset*dx/length
    if reverse:
        points.reverse()
    actor = copy.deepcopy(template)
    actor.update(DataID=data_id, UNIQUEID=identity, pos=points[0], activeDistance=1000.,
                 speed=3.96, active=True, loop=False, isWaypointMode=True)
    actor['waypointEventList'] = [dict(index=-1, waypointEventData=[]) for _ in points]
    yaw = math.degrees(math.atan2(points[1]['y']-points[0]['y'], points[1]['x']-points[0]['x']))
    actor['rot'] = dict(roll='0.000', pitch='0.000', yaw=f'{yaw:.3f}')
    for key in ('initPos', 'standardPos'):
        actor[key] = unity(points[0])
    actor['movingDistanceAmount'] = 0.
    path = dict(pathName=f'PedPath - {identity}', waypointData=[
        dict(pos=unity(point), rot=dict(roll='0.000', pitch='0.000', yaw='0.000')) for point in points])
    return actor, path, dict(asset=name, unique_id=identity, points=points,
                             reverse=reverse, lateral_offset_m=offset, nominal_speed_mps=1.1)


def prepare(output, selected_scenes=None):
    if output.exists():
        raise FileExistsError('Fresh output directory required')
    output.mkdir(parents=True)
    sources = {str(p): sha(p) for p in (NETWORK, ARCHIVE, PET_TEMPLATE, LOOKUP,
        ROOT/'configs/scenario/2026_molit_comp_sample_scene.json',
        ROOT/'configs/scenario/perception_benchmark/bench_static_obstacle_base3_assets_raw.json')}
    sample = json.loads((ROOT/'configs/scenario/2026_molit_comp_sample_scene.json').read_text())
    grounded = json.loads(ARCHIVE.read_text())
    profiles = json.loads(LOOKUP.read_text())['profiles']
    pet_template = json.loads(PET_TEMPLATE.read_text())['pedestrianList'][0]
    scenes = {'static': json.loads((ROOT/'configs/scenario/perception_benchmark/bench_static_obstacle_base3_assets_raw.json').read_text())}
    rows = {'static': []}
    if selected_scenes and 'near_cargo' in selected_scenes:
        sys.path.insert(0,str(ROOT/'dataset/morai_od/scripts'))
        from build_static_obstacle_od_benchmark import configure_parked_manual_ego
        from generate_dynamic_object_capture_set import (load_route,build_mgeo_guided_competition_route,
            route_arc_coordinates,continuous_route_pose_at_arc)
        route_path=ROOT/'configs/global_path/2026_molit_comp_global_path.txt'
        link_path=ROOT/'kcity_map/mgeo_map_scenario_runner_2025/link_set.json'
        sources.update({str(p):sha(p) for p in (route_path,link_path)})
        guide,_=load_route(route_path)
        route,_=build_mgeo_guided_competition_route(guide,json.loads(link_path.read_text()))
        pose=continuous_route_pose_at_arc(route,route_arc_coordinates(route),260.,wrap=False)
        near=copy.deepcopy(scenes['static']);configure_parked_manual_ego(near,pose)
        scenes['near_cargo']=near;rows['near_cargo']=[dict(ego_route_arc_m=260.,road_pose=pose,
            scope='Existing benchmark actors unchanged; surveyed route pose and original .34m Ego offset. Actual drift/contact guard remains required.')]
    for name, actors in (
        ('man1', [('Man1', 50100001, 0., False)]),
        ('pair_follow', [('Man1', 50100001, -.8, False), ('Man1', 50100001, .8, False)]),
        ('pair_opposed', [('Man1', 50100001, -.4, False), ('Man1', 50100001, .4, True)]),
        ('pet', [('Ped_Pet1', 50100012, 0., False)])):
        scene = copy.deepcopy(grounded)
        for key in ('vehicleList', 'objectList', 'pedestrianList', 'spawnPointList', 'pedspawnPointList', 'aiPedstrianList', 'waypointDataList'):
            scene[key] = []
        metadata = []
        for identity, (asset, data_id, offset, reverse) in enumerate(actors, 301):
            profile = profiles['CW000002/'+asset]
            # The lookup, not a guessed DataID, is the authoritative binding.
            data_id = int(profile['data_id'])
            template = pet_template if asset == 'Ped_Pet1' else sample['pedestrianList'][0]
            actor, path, row = pedestrian(template, identity, asset,
                                          data_id, profile, offset, reverse)
            initial = scene['egoVehicle']['initPosition']['pos']
            px, py = initial['x'], initial['y']
            a, b = row['points']
            dx, dy = b['x']-a['x'], b['y']-a['y']
            fraction = max(0., min(1., ((px-a['x'])*dx+(py-a['y'])*dy)/(dx*dx+dy*dy)))
            distance = math.hypot(px-a['x']-fraction*dx, py-a['y']-fraction*dy)
            if distance < 8.:
                raise ValueError('Pedestrian corridor approaches parked Ego')
            row['ego_path_distance_m'] = distance
            scene['pedestrianList'].append(actor)
            scene['waypointDataList'].append(path)
            metadata.append(row)
        scenes[name], rows[name] = scene, metadata
    for name, scene in scenes.items():
        initial = scene['egoVehicle']['initPosition']
        initial['gear'] = 1
        scene['egoVehicle']['currentVelocity'] = scene['egoVehicle']['customValue'] = 0.
        saved = initial.get('vehicleSaveData', {})
        saved['velocity'] = 0.
        for container in (saved, saved.get('vehicle', {})):
            if 'lastVelocity' in container:
                container['lastVelocity'] = dict(x=0., y=0., z=0.)
        save(output/'scenes'/(name+'.json'), scene)
    cases=[dict(scene=s, policy=p) for s, p in
               [('static','fixed51'), ('static','fixed50'), ('static','cut0'),
                ('man1','fixed51'), ('man1','fixed50'), ('man1','cut0'),
                ('pair_follow','fixed51'), ('pair_opposed','fixed51'), ('pet','fixed51')]]
    if selected_scenes:
        if 'near_cargo' in selected_scenes:cases.append(dict(scene='near_cargo',policy='fixed51'))
        cases=[case for case in cases if case['scene'] in selected_scenes]
    save(output/'plan.json', dict(source_sha256=sources, scenes=rows,cases=cases,
        scope='Parked Ego, raw scene QA; no bulk dataset, learned closed-loop or production policy promotion.'))


def run(output, duration):
    sys.path[:0] = [str(VALIDATOR/'src'), str(ROOT/'base_ws/src/control/trajectory_batch_runner/src')]
    import rospy
    import rosbag
    from planning_scenario_validator.runtime import Processes, LiveMonitor, BrakeHold, prepare_scenario, check_no_conflicting_publishers
    from planning_scenario_validator.runner import _load_scene
    check_no_conflicting_publishers()
    rospy.init_node('perception_candidate_probe', disable_signals=True)
    if rospy.get_param('/use_sim_time', False):
        raise RuntimeError('Live receipt-time contract requires use_sim_time=false')
    monitor = LiveMonitor()
    hold = BrakeHold()
    processes = Processes(output/'stack_logs', os.environ.copy())
    plan = json.loads((output/'plan.json').read_text())
    completed = []
    hold.start()
    recorder = None
    last_policy = None
    try:
        processes.start('sensor_tf', ['roslaunch','tf_pub','sensor_tf.launch',
            f'sensor_config:={ROOT}/configs/sensors/Sensor_competition_local.json'])
        processes.start('pose', ['roslaunch','bringup','maptf_morai.launch','start_lanelet_map:=false'])
        for index, case in enumerate(plan['cases']):
            policy, name = case['policy'], case['scene']
            if policy != last_policy:
                processes.stop('udp')
                processes.start('udp', ['roslaunch','sensor_interface','morai_udp_interface.launch',
                    f'sensor_config:={ROOT}/configs/sensors/Sensor_competition_local.json',
                    f'network_config:={NETWORK}', *POLICIES[policy]])
                monitor.wait(lambda: time.monotonic()-monitor.stamps.get('ego', 0) < .5, 15, 'fresh Ego UDP')
                last_policy = policy
            folder = output/f'{index:02d}_{name}_{policy}'
            folder.mkdir()
            source = output/'scenes'/(name+'.json')
            installed, xy = prepare_scenario(SimpleNamespace(source=source, source_sha256=sha(source),
                install_stem=install_stem(output, index)), INSTALL, NETWORK)
            save(folder/'scenario.json', json.loads(installed.read_text()))
            _load_scene({'document': dict(scenario_ready_timeout_sec=25,
                morai=dict(scenario_load_host='127.0.0.1',scenario_load_port=9095))}, installed, monitor, xy)
            monitor.wait(lambda: monitor.ego is not None and math.hypot(monitor.ego.velocity.x, monitor.ego.velocity.y)<.1,
                         15, 'loaded Ego stop')
            from morai_msgs.msg import ObjectStatusList
            observed=rospy.wait_for_message('/Object_topic',ObjectStatusList,timeout=3.)
            requested=json.loads(installed.read_text())
            for source_group,status_group in (('pedestrianList','pedestrian_list'),('objectList','obstacle_list'),('vehicleList','npc_list')):
                expected_ids={int(o['UNIQUEID']) for o in requested.get(source_group,[])}
                if expected_ids != {int(o.unique_id) for o in getattr(observed,status_group)}:
                    raise RuntimeError('Loaded actor IDs differ; do not certify a rejected actor placement: '+source_group)
            monitor.arm(folder/'events.jsonl')
            save(folder/'parameters.json', rospy.get_param('/'))
            recorder = processes.start('recorder', ['rosbag','record','--lz4','-O',str(folder/'raw.bag'),
                '/Ego_topic','/Competition_topic','/Object_topic','/CollisionData','/competition/raw_twist',
                '/vehicle/status/gear','/vehicle/status/control_mode','/sensor/lidar1/packets','/sensor/lidar1',
                '/sensor/gps','/sensor/imu','/tf','/tf_static','/rosout',
                *[f'/sensor/camera/{c}/compressed' for c in ('front','front_left','front_right','back')]])
            start = time.monotonic()
            initial = (monitor.ego.position.x, monitor.ego.position.y)
            while time.monotonic()-start < duration:
                if recorder.poll() is not None:
                    raise RuntimeError('Raw recorder exited')
                if time.monotonic()-monitor.stamps.get('ego', 0) > 1.:
                    raise RuntimeError('Ego stream lost')
                if math.hypot(monitor.ego.position.x-initial[0],monitor.ego.position.y-initial[1]) > .3:
                    raise RuntimeError('Parked Ego drift exceeds 30cm')
                if monitor.collision_count or monitor.far_collision:
                    raise RuntimeError('Contact/contaminated CollisionData; stop and preserve raw evidence')
                time.sleep(.1)
            processes.stop('recorder')
            recorder = None
            with rosbag.Bag(str(folder/'raw.bag')) as bag:
                info = bag.get_type_and_topic_info().topics
                required = ['/Ego_topic','/Object_topic','/sensor/lidar1','/sensor/lidar1/packets'] + [
                    f'/sensor/camera/{c}/compressed' for c in ('front','front_left','front_right','back')]
                if any(topic not in info or info[topic].message_count < 10 for topic in required):
                    raise RuntimeError('Incomplete raw inputs')
                sizes = sorted({len(msg.packets) for _,msg,_ in bag.read_messages(topics=['/sensor/lidar1/packets'])})
            expected = 51 if policy == 'fixed51' else 50
            if sizes != [expected]:
                raise RuntimeError(f'Unexpected live packet count {sizes}, expected {expected}')
            row = dict(case=case, folder=str(folder), bag_sha256=sha(folder/'raw.bag'), packets=sizes,
                       duration_sec=duration, ego_final_speed_mps=math.hypot(monitor.ego.velocity.x,monitor.ego.velocity.y))
            completed.append(row)
            save(output/'progress.json', dict(completed=completed))
            print(json.dumps(row), flush=True)
    finally:
        processes.stop('recorder')
        # Keep braking until the observed speed is low, then close only owned processes.
        save(output/'final_stop.json', dict(speed_mps=None if monitor.ego is None else
            math.hypot(monitor.ego.velocity.x,monitor.ego.velocity.y),
            note='Brake hold preserved through cleanup; no extra reload merely to Park.'))
        processes.close()
        hold.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=32.)
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--scene',action='append',choices=('static','man1','pair_follow','pair_opposed','pet','near_cargo'))
    args = parser.parse_args()
    if not 10 <= args.duration <= 60:
        parser.error('Bounded QA duration must be 10..60 seconds')
    prepare(args.output,args.scene)
    if args.run:
        run(args.output, args.duration)


if __name__ == '__main__':
    main()
