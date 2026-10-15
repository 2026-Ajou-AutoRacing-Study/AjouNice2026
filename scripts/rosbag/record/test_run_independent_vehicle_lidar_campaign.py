import importlib.util
import json
from pathlib import Path
import sys

import pytest
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
spec = importlib.util.spec_from_file_location('independent_campaign', HERE/'run_independent_vehicle_lidar_campaign.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_default_four_configs_preserve_shared_source(tmp_path):
    validator = module.VALIDATOR/'src/planning_scenario_validator/runner.py'
    before = module.digest(validator)
    output = tmp_path/'plan'
    plan = module.prepare_cases(output, None, ['fixed51', 'fixed50'], ['highway', 'roundabout'])
    assert len(plan['cases']) == 4
    assert plan['production_changed'] is False
    for case in plan['cases']:
        cfg = yaml.safe_load(Path(case['config']).read_text())
        assert cfg['stack']['detector'] == 'od_gt'
        assert cfg['upload']['enabled'] is False
        assert cfg['recording']['video']['enabled'] is False
        assert cfg['scenarios']['repeat'] == 1
    assert module.digest(validator) == before


def test_prepared_scene_hash_checked_before_output_creation(tmp_path):
    scene = module.ROOT/'configs/scenario/perception_benchmark/bench_highway_merge_raw.json'
    manifest = tmp_path/'scenes.json'
    manifest.write_text(json.dumps({'scenes': [dict(name='independent', mission='highway', path=str(scene), sha256='bad')]}))
    with pytest.raises(ValueError, match='hash mismatch'):
        module.prepare_cases(tmp_path/'plan', manifest, ['fixed51'], ['highway'])
    assert not (tmp_path/'plan').exists()


@pytest.mark.parametrize('policies', [[], ['cut0'], ['fixed50', 'fixed50']])
def test_unbounded_or_duplicate_policies_rejected(tmp_path, policies):
    with pytest.raises(ValueError):
        module.prepare_cases(tmp_path/'plan', None, policies, ['highway'])


def test_existing_output_not_overwritten(tmp_path):
    with pytest.raises(FileExistsError):
        module.prepare_cases(tmp_path, None, ['fixed51'], ['highway'])


def test_mission_filter_does_not_silently_select_other_scene(tmp_path):
    scene = module.ROOT/'configs/scenario/perception_benchmark/bench_highway_merge_raw.json'
    manifest = tmp_path/'scenes.json'
    manifest.write_text(json.dumps({'scenes': [dict(name='new_highway', mission='highway', path=str(scene))]}))
    with pytest.raises(ValueError, match='one to four'):
        module.prepare_cases(tmp_path/'plan', manifest, ['fixed51'], ['roundabout'])
