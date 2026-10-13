"""Explicit scale-1 static placement envelope; never substitutes native training GT."""
import json
from pathlib import Path

from base5_capture_safety import sha256

POLICY = 'native-static-parent-envelope-v130'


def configure(ctx, config, row):
    ref = config.get('static_placement_geometry')
    if not ref:
        return ctx
    if ref.get('policy') != POLICY:
        raise ValueError('Unknown static placement geometry policy')
    path = Path(ref['path'])
    if sha256(path) != ref['sha256']:
        raise ValueError('Static placement geometry reference changed')
    # Frozen final scopes describe the *actual* stationary appearance after the
    # regional adapter, not the original walking/running request in generic rows.
    yaw = row.get('static_yaw_policy', {})
    stationary_scope = (yaw.get('policy') == 'static-free-sector-yaw-v130'
                        and yaw.get('scope') in
                        ('generic', 'stationary_avoidance', 'shadow_static'))
    if (row.get('family') in ('highway', 'roundabout', 'background')
            or row.get('dynamic_observation_phase') or row.get('shadow_moving')
            or (row.get('motion') not in ('static', 'standing') and not stationary_scope)):
        return ctx
    cache = ctx.setdefault('_static_geometry_cache', {})
    key = (str(path.resolve()), ref['sha256'])
    if key not in cache:
        data = json.loads(path.read_text())
        if data.get('policy') != POLICY or data.get('training_gt_modified') is not False:
            raise ValueError('Placement reference must not modify training GT')
        geometry = dict(ctx['geometry'])
        for name, asset in data['assets'].items():
            if name in geometry:
                continue
            if asset['observations'] < 2 or asset['stationary_only'] is not True:
                raise ValueError('Static geometry lacks repeated stationary evidence')
            geometry[name] = asset
        cache[key] = geometry
    return dict(ctx, geometry=cache[key], static_placement_geometry_reference=dict(ref))
