#!/usr/bin/env python3
"""Small stationary ROS/UDP replay addon for newly calibrated MR assets."""
import argparse
import copy
from pathlib import Path

from generate_mr_restored_asset_calibration import generate
from generate_base5_reinforcement_smoke import write_json
from generate_dynamic_object_capture_set import load_json


def generate_replay(output, runtime_dir=None):
    output = output.resolve()
    if (output / 'manifest.json').exists():
        raise FileExistsError('Use a new output directory')
    calibration = load_json(generate(output / 'source_calibration_deck', runtime_dir))
    manifest = copy.deepcopy(calibration)
    for index, row in enumerate(manifest['scenes']):
        stem = 'b5rt{:02d}'.format(index + 1)
        scenario = load_json(Path(row['scenario']))
        metadata = load_json(Path(row['metadata']))
        row.update(id=stem, file=stem, file_name=stem,
                   scenario=str(output / (stem + '.json')),
                   metadata=str(output / (stem + '_metadata.json')),
                   split='val' if index % 4 == 3 else 'train',
                   mission_family='restored_stationary_replay')
        write_json(Path(row['scenario']), scenario)
        write_json(Path(row['metadata']), metadata)
    manifest.update(dataset_id='base5_mr_restored_replay_261004',
                    scenario_group='base5_mr_restored_replay')
    manifest['capture_policy'].update(captures_per_scene=3, capture_period_sec=.4,
                                      ego_target_speed_kph=2.)
    manifest['generation'].update(
        purpose='Stationary ROS/UDP addon; not near-pass or generalization benchmark',
        requires_lidar_instance_annotation=False,
        requires_online_sensors=True,
        split_unit='scene',
        asset_offsets='dataset/morai_od/configs/morai_asset_bbox_offsets_mr_restored_261004.json')
    write_json(output / 'manifest.json', manifest)
    return output / 'manifest.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    print(generate_replay(parser.parse_args().output))
