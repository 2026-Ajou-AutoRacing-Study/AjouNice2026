"""Trial preparation is not permission to drive; no ROS/GPU/MORAI process."""
import json
from pathlib import Path
import tempfile

import pytest
import yaml

import prepare_traffic_light_v5_trial as trial


def test_trial_preserves_real_full_course_and_excludes_actor_inputs():
    base = yaml.safe_load(trial.BASE.read_text())
    source = trial.ROOT / base['scenarios']['directory'] / base['scenarios']['include'][0]
    original = source.read_bytes()
    with tempfile.TemporaryDirectory(dir=trial.ROOT / 'artifacts/evaluation') as parent:
        output = Path(parent) / 'trial'
        prepared = trial.prepare(output, '/tmp/explicit-models', '/usr/bin/python3')
        doc = yaml.safe_load(prepared.read_text())
        assert doc['default_case'] == base['default_case']
        assert doc['network'] == base['network']
        assert doc['stack']['pose_source'] == 'localization'
        assert doc['stack']['detector'] == 'od_gt'
        assert doc['stack']['traffic_light_profile'] == 'v5_candidate'
        assert doc['validation']['localization_safety']['enabled'] is True
        assert '/sensor/camera/front/compressed' in doc['recording']['selected_topics']
        assert not doc['recording']['video']['enabled']
        assert not doc['upload']['enabled']
        scene = json.loads((output / 'scenarios/no_objects.json').read_text())
        assert scene['egoVehicle']['initPosition']['gear'] == 1
        for key in ('vehicleList', 'pedestrianList', 'obstacleList', 'objectList', 'spawnPointList'):
            assert not scene.get(key)
        with pytest.raises(FileExistsError):
            trial.prepare(output, '/tmp/models', '/usr/bin/python3')
    assert source.read_bytes() == original


def test_trial_cannot_write_outside_evaluation_root():
    with pytest.raises(ValueError):
        trial.prepare('/tmp/not-an-approved-trial', '/tmp/models', '/usr/bin/python3')
