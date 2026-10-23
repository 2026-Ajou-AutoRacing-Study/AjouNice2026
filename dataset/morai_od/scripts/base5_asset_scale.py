"""Opt-in, context-natural asset scale plans for placement, not native GT.

The whitelist is intentionally independent of Scenario container or training
class.  Candidate profiles cannot become collection defaults without explicit
native geometry and visual/contact qualification.  There is no module-level
random state, implicit Unity axis convention, continuous jitter, or GT scaling.
"""
import copy
import hashlib
import json
import math
import weakref
from collections import defaultdict
from collections.abc import Mapping
from pathlib import Path
from types import MappingProxyType


POLICY = 'context-natural-asset-scale-v155'
SCHEMA = 'base5_asset_scale_policy_v1'
ASSIGNMENT_SCHEMA = 'base5_asset_scale_assignment_v1'
BASE = 'original'
PLAN_KEY = 'asset_scale_plan'
CONFIG_KEY = 'asset_scale_policy'
WHITELIST = {
    'CargoBox': (40100001, 'box'),
    'WoodBox': (40100002, 'box'),
    'RedBarrel': (40100003, 'drum'),
    'YellowBarrel': (40100004, 'drum'),
    'TrafficBarrel': (40100005, 'drum'),
    'PE_Drum': (40100027, 'drum'),
    'PE_Firewall_Orange': (40100028, 'barrier'),
    'PE_Firewall_White': (40100029, 'barrier'),
    'Steel_Barricade': (40100033, 'barrier'),
    'Concrete_Barricade01': (40100047, 'barrier'),
    'Concrete_Barricade02': (40100052, 'barrier'),
    'concrete-barrier': (40200162, 'barrier'),
}
_PIN_CACHE = {}
_FREEZE_TOKEN = object()
_FROZEN_NODES = {}


class ScaleContractError(RuntimeError):
    """An identity, policy, geometry or resume violation; not a scene deficit."""


def _immutable(*args, **kwargs):
    raise TypeError('Frozen scale policy cannot be mutated; bind a new validated snapshot')


class FrozenList(tuple):
    """JSON-list semantics with tuple storage, including recursive equality."""
    __slots__ = ()

    def __new__(cls, values, *, _token=None):
        if _token is not _FREEZE_TOKEN:
            raise ScaleContractError('Use freeze_policy to create trusted immutable nodes')
        return tuple.__new__(cls, values)

    def __eq__(self, other):
        if not isinstance(other, (list, tuple)):
            return False
        return len(self) == len(other) and all(a == b for a, b in zip(self, other))

    def __ne__(self, other):
        return not self == other

    __hash__ = None
    __setitem__ = __delitem__ = __iadd__ = __imul__ = _immutable
    append = extend = insert = pop = remove = clear = reverse = sort = _immutable
    __setattr__ = __delattr__ = _immutable

    def __copy__(self):
        return self

    def __deepcopy__(self, memo):
        return self


class _FrozenMap(Mapping):
    """Not a dict subclass: dict.__setitem__ cannot bypass the seal."""
    __slots__ = ('__weakref__',)

    def __init__(self, values, *, _token=None, _verified_profile=False):
        if _token is not _FREEZE_TOKEN:
            raise ScaleContractError('Use freeze_policy to create trusted immutable nodes')
        ident = id(self)
        if ident in _FROZEN_NODES:
            raise ScaleContractError('Frozen scale nodes cannot be reinitialized')
        def discard(reference, registry=_FROZEN_NODES):
            saved = registry.get(ident)
            if saved and saved[0] is reference:
                registry.pop(ident, None)
        _FROZEN_NODES[ident] = (weakref.ref(self, discard), MappingProxyType(dict(
            data=MappingProxyType(dict(values)), verified_profile=_verified_profile, kind='mapping')))

    @property
    def _data(self):
        return _frozen_node(self)['data']

    @property
    def _verified_profile(self):
        return _frozen_node(self)['verified_profile']

    def __getitem__(self, key):
        return self._data[key]

    def __iter__(self):
        return iter(self._data)

    def __len__(self):
        return len(self._data)

    __setattr__ = __delattr__ = __setitem__ = __delitem__ = __ior__ = _immutable
    clear = pop = popitem = setdefault = update = _immutable

    def __copy__(self):
        return self

    def __deepcopy__(self, memo):
        return self


class FrozenPolicy(_FrozenMap):
    """Deep immutable, validated snapshot with one canonical digest.

    Evidence files are verified at snapshot creation and explicitly again by
    verify_frozen_policy at operation boundaries, never per actor hot call.
    """
    __slots__ = ()

    def __init__(self, values, *, _token=None, _sha256=None, _evidence=()):
        super().__init__(values, _token=_token)
        ref, state = _FROZEN_NODES[id(self)]
        _FROZEN_NODES[id(self)] = (ref, MappingProxyType(dict(state, kind='policy',
            policy_sha256=_sha256, evidence=tuple(_evidence), evidence_stamps=MappingProxyType({
                path: _file_stamp(Path(path)) for path, _ in _evidence}))))

    @property
    def _policy_sha256(self):
        return _frozen_node(self, policy=True)['policy_sha256']

    @property
    def _evidence(self):
        return _frozen_node(self, policy=True)['evidence']

    @property
    def _evidence_stamps(self):
        return _frozen_node(self, policy=True)['evidence_stamps']


def _frozen_node(value, policy=False):
    saved = _FROZEN_NODES.get(id(value))
    if (not saved or saved[0]() is not value or (policy and saved[1]['kind'] != 'policy')):
        raise ScaleContractError('Uninitialized or forged immutable scale snapshot')
    return saved[1]


def _file_stamp(path):
    try:
        stat = path.stat()
    except OSError as error:
        raise ScaleContractError('Frozen scale evidence disappeared') from error
    return stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns


def _freeze(value, path=()):
    if isinstance(value, dict):
        require_keys = all(isinstance(key, str) for key in value)
        if not require_keys:
            raise ScaleContractError('Frozen policy JSON mapping keys must be strings')
        return _FrozenMap({key: _freeze(item, path+(key,)) for key, item in value.items()},
            _token=_FREEZE_TOKEN, _verified_profile=(len(path) == 4 and path[0] == 'assets' and path[2] == 'profiles'))
    if isinstance(value, list):
        return FrozenList((_freeze(item, path+(str(index),)) for index, item in enumerate(value)), _token=_FREEZE_TOKEN)
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    raise ScaleContractError('Frozen policy must contain only JSON mappings/lists/scalars')


def thaw_policy(value):
    """Explicit JSON boundary copy; no mutable child aliases escape a snapshot."""
    if isinstance(value, Mapping):
        return {key: thaw_policy(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [thaw_policy(item) for item in value]
    return copy.deepcopy(value)


def plain_config(config):
    """Make a normal JSON-compatible config for persisted manifests only."""
    return thaw_policy(config)


def verify_frozen_policy(policy):
    """Begin/end guard, including evidence inode/mtime/ctime drift or replacement."""
    if type(policy) is not FrozenPolicy:
        raise ScaleContractError('Exact trusted FrozenPolicy snapshot required')
    _frozen_node(policy, policy=True)
    evidence = [dict(path=path, sha256=checksum) for path, checksum in policy._evidence]
    if evidence:
        _pins(evidence)
    for path, stamp in policy._evidence_stamps.items():
        if _file_stamp(Path(path)) != stamp:
            raise ScaleContractError('Frozen scale evidence stamp changed during operation')
    # The tree cannot mutate through normal or direct dict/list base methods.
    # A thawed copy is intentionally not cached and retains ordinary validation.
    return policy


def freeze_policy(policy):
    """Copy, validate and hash once at a deliberate operation boundary."""
    if type(policy) is FrozenPolicy:
        return verify_frozen_policy(policy)
    if not isinstance(policy, dict):
        raise ScaleContractError('Mutable policy dict or trusted FrozenPolicy required')
    plain = copy.deepcopy(policy)
    validate_policy(plain)
    checksum = digest(plain)
    evidence = {}
    for record in plain['assets'].values():
        for profile in record['profiles'].values():
            if profile['status'] == 'qualified':
                for pin in profile['native_qualification']['evidence']:
                    evidence[pin['path']] = pin['sha256']
    frozen = _freeze(plain)
    snapshot = FrozenPolicy(dict(frozen.items()), _token=_FREEZE_TOKEN,
        _sha256=checksum, _evidence=sorted(evidence.items()))
    return verify_frozen_policy(snapshot)


def bind_config(config):
    """Return a new outer config, changing only policy to an immutable snapshot."""
    result = dict(config)
    if result.get(CONFIG_KEY) is not None:
        result[CONFIG_KEY] = freeze_policy(result[CONFIG_KEY])
    return result


def _json_mapping(value):
    if isinstance(value, Mapping):
        return dict(value.items())
    raise TypeError('Unsupported non-JSON scale value: '+type(value).__name__)


def digest(value):
    if type(value) is FrozenPolicy:
        return value._policy_sha256
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False, default=_json_mapping).encode()).hexdigest()


def implementation_sha256():
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def _number(value, *, positive=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or (positive and value <= 0)):
        raise ScaleContractError('Finite numeric scale/geometry values required')
    return float(value)


def _vector(value, *, positive=False):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ScaleContractError('Three-axis scale/geometry vector required')
    return [_number(v, positive=positive) for v in value]


def _axes(value):
    if (not isinstance(value, (list, tuple)) or len(value) != 3
            or any(type(v) is not int for v in value) or sorted(value) != [0, 1, 2]):
        raise ScaleContractError('Explicit bbox-to-scenario-scale axis permutation required')
    return list(value)


def _pins(evidence):
    if not isinstance(evidence, list) or not evidence:
        raise ScaleContractError('Pinned native and visual qualification evidence required')
    for pin in evidence:
        if (not isinstance(pin, dict) or set(pin) != {'path', 'sha256'}
                or not isinstance(pin['path'], str) or not Path(pin['path']).is_absolute()
                or not isinstance(pin['sha256'], str)):
            raise ScaleContractError('Absolute evidence path and checksum required')
        path = Path(pin['path'])
        if not path.is_file():
            raise ScaleContractError('Asset scale qualification evidence changed')
        stat = path.stat()
        stamp = (str(path), pin['sha256'], stat.st_dev, stat.st_ino, stat.st_size,
                 stat.st_mtime_ns, stat.st_ctime_ns)
        if stamp in _PIN_CACHE:
            continue
        if hashlib.sha256(path.read_bytes()).hexdigest() != pin['sha256']:
            raise ScaleContractError('Asset scale qualification evidence changed')
        after = path.stat()
        if (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns) != stamp[2:]:
            raise ScaleContractError('Asset scale evidence changed while hashing')
        _PIN_CACHE[stamp] = True


def _qualified(profile):
    if type(profile) is _FrozenMap and profile._verified_profile:
        return profile.get('status') == 'qualified'
    if profile.get('status') != 'qualified':
        return False
    qualification = profile.get('native_qualification', {})
    if (type(qualification.get('observations')) is not int
            or qualification['observations'] < 2
            or qualification.get('semantic_visual_review') != 'accepted'
            or qualification.get('contact_review') != 'passed'
            or qualification.get('sensor_gt_review') != 'passed'
            or qualification.get('axis_mapping_verified') is not True
            or qualification.get('single_component_obstacle_verified') is not True):
        raise ScaleContractError('Qualified scale profile lacks actual native/visual approval')
    geometry = profile.get('native_geometry', {})
    _vector(geometry.get('bbox_size_m'), positive=True)
    _vector(geometry.get('pivot_to_bbox_center_local_m'))
    _number(geometry.get('pivot_height_above_ground_m'))
    _pins(qualification.get('evidence'))
    return True


def validate_policy(policy):
    if type(policy) is FrozenPolicy:
        _frozen_node(policy, policy=True)
        return policy
    if (not isinstance(policy, dict) or policy.get('schema') != SCHEMA
            or policy.get('policy') != POLICY or policy.get('continuous_jitter') is not False
            or policy.get('training_gt_modified') is not False
            or not isinstance(policy.get('assets'), dict)):
        raise ScaleContractError('Explicit discrete asset scale policy required')
    for name, record in policy['assets'].items():
        if (name not in WHITELIST or not isinstance(record, dict)
                or type(record.get('data_id')) is not int
                or (record['data_id'], record.get('group')) != WHITELIST[name]
                or record.get('source_list') != 'objectList'
                or record.get('semantic_class') != 'obstacle'):
            raise ScaleContractError('Only whitelisted physical obstacle identities may scale')
        fraction = _number(record.get('variation_fraction'))
        if not 0 <= fraction <= .5:
            raise ScaleContractError('Scale variation fraction must preserve original majority')
        if (type(record.get('minimum_original')) is not int
                or record['minimum_original'] < 1):
            raise ScaleContractError('At least one original instance must be protected')
        _axes(record.get('bbox_axis_to_scale_axis'))
        profiles = record.get('profiles')
        if not isinstance(profiles, dict) or not profiles:
            raise ScaleContractError('At least one declared discrete candidate profile required')
        for ident, profile in profiles.items():
            if (not isinstance(ident, str) or not ident or ident == BASE
                    or not isinstance(profile, dict)
                    or profile.get('status') not in ('candidate', 'qualified', 'rejected')):
                raise ScaleContractError('Unknown discrete scale profile/status')
            scale = _vector(profile.get('scale_xyz'), positive=True)
            if scale == [1., 1., 1.]:
                raise ScaleContractError('Original scale is implicit, never a variant profile')
            _number(profile.get('weight'), positive=True)
            if record['group'] == 'drum' and max(scale) - min(scale) > 1.e-12:
                raise ScaleContractError('Drum candidates must use isotropic scale')
            _qualified(profile)
    return policy


def candidate_policy(bbox_axis_to_scale_axis):
    """Return unapproved probe candidates; the caller must declare its axis hypothesis.

    This creates no native qualification and is safe to pass to normal selection:
    without ``allow_candidate`` every actor remains original size.
    """
    axes = _axes(bbox_axis_to_scale_axis)
    records = {}
    for name, (data_id, group) in WHITELIST.items():
        if group == 'box':
            scales = {'iso125': ([1.25] * 3, 7), 'iso150': ([1.5] * 3, 3)}
            if name == 'CargoBox':
                scales['sample_2_3_2'] = ([2., 3., 2.], 1)
        elif group == 'drum':
            scales = {'iso120': ([1.2] * 3, 7), 'iso150': ([1.5] * 3, 3)}
        else:
            # All six candidate references currently have local bbox Y as the
            # long horizontal axis. This is a declared hypothesis, not approval.
            scales = {}
            for factor, weight in ((1.25, 7), (1.5, 3)):
                scale = [1., 1., 1.]
                scale[axes[1]] = factor
                scales['length' + str(int(factor * 100))] = (scale, weight)
        records[name] = dict(data_id=data_id, group=group, source_list='objectList',
            semantic_class='obstacle', variation_fraction=.3 if group == 'box' else .2,
            minimum_original=2, bbox_axis_to_scale_axis=axes,
            profiles={ident: dict(scale_xyz=scale, weight=weight, status='candidate')
                      for ident, (scale, weight) in scales.items()})
    return dict(schema=SCHEMA, policy=POLICY, continuous_jitter=False,
                training_gt_modified=False, assets=records)


def _record(asset, policy, motion):
    validate_policy(policy)
    name, ident = asset.get('name'), asset.get('data_id')
    record = policy['assets'].get(name)
    if record is None:
        if ident in {row['data_id'] for row in policy['assets'].values()}:
            raise ScaleContractError('Declared scalable DataID has a different asset name')
        return None
    if (type(ident) is not int or ident != record['data_id']
            or asset.get('source_list') != 'objectList'
            or asset.get('semantic_class') != 'obstacle'
            or asset.get('ontology_class', 'Obstacle') != 'Obstacle'
            or asset.get('collection_role', 'positive') != 'positive'):
        raise ScaleContractError('Scalable asset catalog identity/class/container differs')
    return record if motion in ('static', 'standing') else None


def actor_key(asset, row, ordinal):
    """Stable slot identity; geometry retry numbers and shared RNG are excluded."""
    if (type(row.get('seed')) is not int or type(ordinal) is not int or ordinal < 0):
        raise ScaleContractError('Frozen integer scene seed and actor ordinal required')
    return digest(dict(seed=row['seed'], ordinal=ordinal, data_id=asset['data_id'],
                       name=asset['name'], source_list=asset['source_list']))


def _available(record, allow_candidate):
    return {key: value for key, value in record['profiles'].items()
            if _qualified(value) or (allow_candidate and value['status'] == 'candidate')}


def _weighted(profiles, key):
    total = sum(profile['weight'] for profile in profiles.values())
    target = int(digest([key, 'profile'])[:16], 16) / 2**64 * total
    for ident, profile in sorted(profiles.items()):
        target -= profile['weight']
        if target < 0:
            return ident
    return sorted(profiles)[-1]


def select(asset, row, ordinal, motion, policy=None, *, profile_id=None,
           allow_candidate=False):
    """Select before placement. An explicit frozen profile never falls back on retry."""
    if policy is None:
        if profile_id is not None:
            raise ScaleContractError('Frozen scale profile supplied without its policy')
        return None
    record = _record(asset, policy, motion)
    if record is None:
        if profile_id not in (None, BASE):
            raise ScaleContractError('Cannot scale moving or non-whitelisted actor')
        return None
    key = actor_key(asset, row, ordinal)
    profiles = _available(record, allow_candidate)
    if profile_id is None:
        draw = int(digest([key, 'variant'])[:16], 16) / 2**64
        profile_id = (_weighted(profiles, key)
                      if profiles and draw < record['variation_fraction'] else BASE)
    if profile_id != BASE and profile_id not in profiles:
        raise ScaleContractError('Frozen scale profile is unapproved, rejected, or absent')
    profile = record['profiles'].get(profile_id)
    result = dict(schema=ASSIGNMENT_SCHEMA, policy=POLICY,
        implementation_sha256=implementation_sha256(), policy_sha256=digest(policy),
        actor_key=key, scene_seed=row['seed'], ordinal=ordinal, data_id=asset['data_id'],
        name=asset['name'], source_list='objectList', semantic_class='obstacle', motion=motion,
        profile_id=profile_id, scale_xyz=list(profile['scale_xyz']) if profile else [1., 1., 1.],
        profile_status=profile['status'] if profile else 'original',
        candidate_only=bool(profile and profile['status'] == 'candidate'),
        bbox_axis_to_scale_axis=list(record['bbox_axis_to_scale_axis']),
        training_gt_modified=False, retry_resamples_scale=False)
    result['assignment_sha256'] = digest(result)
    return result


def validate_assignment(assignment, policy, *, allow_candidate=False):
    if not isinstance(assignment, dict):
        raise ScaleContractError('Scale assignment metadata required')
    payload = {k: v for k, v in assignment.items() if k != 'assignment_sha256'}
    if (assignment.get('assignment_sha256') != digest(payload)
            or assignment.get('schema') != ASSIGNMENT_SCHEMA
            or assignment.get('policy') != POLICY
            or assignment.get('implementation_sha256') != implementation_sha256()
            or assignment.get('policy_sha256') != digest(policy)):
        raise ScaleContractError('Frozen scale assignment/helper/policy checksum changed')
    asset = {k: assignment.get(k) for k in
             ('name', 'data_id', 'source_list', 'semantic_class')}
    row = dict(seed=assignment.get('scene_seed'))
    expected = select(asset, row, assignment.get('ordinal'), assignment.get('motion'), policy,
                      profile_id=assignment.get('profile_id'), allow_candidate=allow_candidate)
    if expected != assignment:
        raise ScaleContractError('Frozen scale assignment identity/profile differs')
    return assignment


def placement_geometry(size, pivot, assignment, policy, *, allow_candidate=False):
    """Return a new placement envelope; native annotations are never accepted here.

    Qualified variants use *measured* geometry without multiplying it again.
    Candidates use a clearly marked axis hypothesis to prepare bounded probes.
    """
    original_size = _vector(size, positive=True)
    original_pivot = _vector(pivot)
    if assignment is None:
        return original_size, original_pivot, None
    validate_assignment(assignment, policy, allow_candidate=allow_candidate)
    if assignment['profile_id'] == BASE:
        result_size, result_pivot, mode = original_size, original_pivot, 'scale1_reference'
    elif assignment['candidate_only']:
        factors = [assignment['scale_xyz'][axis]
                   for axis in assignment['bbox_axis_to_scale_axis']]
        result_size = [v * factor for v, factor in zip(original_size, factors)]
        result_pivot = [v * factor for v, factor in zip(original_pivot, factors)]
        mode = 'candidate_scaled_envelope_not_native_certification'
    else:
        profile = policy['assets'][assignment['name']]['profiles'][assignment['profile_id']]
        measured = profile['native_geometry']
        result_size = _vector(measured['bbox_size_m'], positive=True)
        result_pivot = _vector(measured['pivot_to_bbox_center_local_m'])
        mode = 'qualified_native_geometry_not_rescaled'
    return result_size, result_pivot, dict(mode=mode,
        assignment_sha256=assignment['assignment_sha256'],
        scale1_size_m=original_size, scale1_pivot_m=original_pivot,
        placement_size_m=result_size, placement_pivot_m=result_pivot,
        training_gt_modified=False, scale1_height_cache_reusable=assignment['profile_id'] == BASE)


def apply_actor(actor, assignment, policy, *, allow_candidate=False):
    """Return a scenario actor copy after set_pose; never multiply template scale."""
    result = copy.deepcopy(actor)
    if assignment is None:
        return result
    validate_assignment(assignment, policy, allow_candidate=allow_candidate)
    if result.get('DataID') != assignment['data_id']:
        raise ScaleContractError('Scenario actor DataID differs from scale assignment')
    scale = assignment['scale_xyz']
    result['scale'] = dict(zip(('x', 'y', 'z'), map(float, scale)))
    result['scale'].update({'_' + key: f'{float(value):.3f}'
                            for key, value in zip(('x', 'y', 'z'), scale)})
    return result


def verify_actor(actor, assignment, policy, *, asset=None, row=None, ordinal=None,
                 motion=None, allow_candidate=False):
    """Bind serialized scenario scale to frozen metadata, including optional spec identity."""
    validate_assignment(assignment, policy, allow_candidate=allow_candidate)
    expected = apply_actor(actor, assignment, policy, allow_candidate=allow_candidate)
    if actor.get('scale') != expected['scale']:
        raise ScaleContractError('Scenario scale or serialized display scale differs from metadata')
    supplied = (asset, row, ordinal, motion)
    if any(value is not None for value in supplied):
        if any(value is None for value in supplied):
            raise ScaleContractError('Complete current actor/spec identity required')
        if select(asset, row, ordinal, motion, policy, profile_id=assignment['profile_id'],
                  allow_candidate=allow_candidate) != assignment:
            raise ScaleContractError('Scale assignment belongs to another actor/spec')
    return assignment


def stratified_profiles(items, policy, *, allow_candidate=False):
    """Freeze variant counts with original primary protection before regeneration.

    Items contain asset/row/ordinal/motion, with row.split (train or val). Quotas
    are rounded once per asset and split, then apportioned across family,
    mission, distance-bin and role strata. This selects no geometry or new asset.
    The returned actor-key/profile map must be frozen into new specs by caller.
    """
    validate_policy(policy)
    groups = defaultdict(list)
    seen = set()
    for item in items:
        asset, row = item['asset'], item['row']
        record = _record(asset, policy, item['motion'])
        if record is None:
            continue
        key = actor_key(asset, row, item['ordinal'])
        if key in seen:
            raise ScaleContractError('Duplicate actor slot in scale allocation')
        seen.add(key)
        if row.get('split') not in ('train', 'val'):
            raise ScaleContractError('Explicit train/val split required for scale allocation')
        stratum = (row.get('family'), bool(row.get('mission')), row.get('distance_bin'), row.get('role'))
        groups[(asset['name'], row['split'])].append((key, stratum, item['ordinal']))
    mapping, summaries = {}, []
    for (name, split), actors in sorted(groups.items()):
        record = policy['assets'][name]
        profiles = _available(record, allow_candidate)
        ordered = sorted(actors, key=lambda a: digest([a[0], 'allocation']))
        protected = {key for key, _, _ in ordered[:record['minimum_original']]}
        primary = [a for a in ordered if a[2] == 0]
        if primary:
            protected.add(primary[0][0])
        target = min(int(len(actors) * record['variation_fraction'] + .5),
                     len(actors) - len(protected)) if profiles else 0
        buckets = defaultdict(list)
        for key, stratum, _ in ordered:
            mapping[key] = BASE
            if key not in protected:
                buckets[stratum].append(key)
        total = sum(map(len, buckets.values()))
        quotas = {stratum: target * len(keys) / total for stratum, keys in buckets.items()} if total else {}
        counts = {stratum: int(value) for stratum, value in quotas.items()}
        remaining = target - sum(counts.values())
        rank = sorted(quotas, key=lambda s: (-(quotas[s] - counts[s]), digest(list(s))))
        for stratum in rank[:remaining]:
            counts[stratum] += 1
        for stratum, keys in buckets.items():
            for key in keys[:counts[stratum]]:
                mapping[key] = _weighted(profiles, key)
        summaries.append(dict(name=name, split=split, total=len(actors), variants=target,
                              original=len(actors) - target, protected_original_keys=sorted(protected)))
    return mapping, dict(schema='base5_asset_scale_allocation_v1', policy_sha256=digest(policy),
                         groups=summaries, training_gt_modified=False,
                         primary_original_preserved=True, actor_population_preserved=True)


def attach_plan(row, actors, policy, profiles):
    """Create a new spec overlay; source spec/range contracts stay recoverable."""
    validate_policy(policy)
    if PLAN_KEY in row or len(profiles) != len(actors) or row.get('actors') != len(actors):
        raise ScaleContractError('New scale plan requires the complete original actor list')
    if all(ident == BASE for ident in profiles):
        return copy.deepcopy(row)
    for ordinal, (actor, profile) in enumerate(zip(actors, profiles)):
        select(_identity_asset(actor), row, ordinal, actor['motion'], policy, profile_id=profile)
    identities = [{k: actor.get(k) for k in
                   ('data_id', 'name', 'source_list', 'motion', 'unique_id', 'population_pool')}
                  for actor in actors]
    plan = dict(policy=POLICY, implementation_sha256=implementation_sha256(),
        policy_sha256=digest(policy), source_spec=copy.deepcopy(row), source_spec_sha256=digest(row),
        actor_identity=identities, profile_ids=list(profiles), actor_population_preserved=True,
        native_qa_unchanged=True, range_plan_rebind='primary_obstacle_range' in row)
    result = copy.deepcopy(row)
    result['frozen_population_data_ids'] = [actor['data_id'] for actor in actors]
    result[PLAN_KEY] = plan
    if 'primary_obstacle_range' in result:
        # Explicit new-spec binding, never a change to its distance/classifier.
        result['primary_obstacle_range']['frozen_spec_fields_sha256'] = digest(
            {k: value for k, value in result.items() if k != 'primary_obstacle_range'})
    validate_plan(result, {CONFIG_KEY: policy})
    return result


def validate_plan(row, config):
    plan = row.get(PLAN_KEY)
    if plan is None:
        return None
    policy = config.get(CONFIG_KEY)
    validate_policy(policy)
    if (not isinstance(plan, dict) or plan.get('policy') != POLICY
            or plan.get('implementation_sha256') != implementation_sha256()
            or plan.get('policy_sha256') != digest(policy)
            or plan.get('source_spec_sha256') != digest(plan.get('source_spec'))
            or PLAN_KEY in plan.get('source_spec', {})
            or plan.get('actor_population_preserved') is not True
            or plan.get('native_qa_unchanged') is not True):
        raise ScaleContractError('Frozen scale plan source/helper/policy differs')
    source = plan['source_spec']
    actors, profiles = plan.get('actor_identity'), plan.get('profile_ids')
    if (not isinstance(actors, list) or not isinstance(profiles, list)
            or len(actors) != len(profiles) or len(actors) != source.get('actors')
            or all(ident == BASE for ident in profiles)
            or plan.get('range_plan_rebind') != ('primary_obstacle_range' in source)):
        raise ScaleContractError('Frozen scale actor list/profile counts differ')
    expected = copy.deepcopy(source)
    expected['frozen_population_data_ids'] = [actor['data_id'] for actor in actors]
    expected[PLAN_KEY] = copy.deepcopy(plan)
    if 'primary_obstacle_range' in expected:
        expected['primary_obstacle_range']['frozen_spec_fields_sha256'] = digest(
            {k: value for k, value in expected.items() if k != 'primary_obstacle_range'})
    if expected != row:
        raise ScaleContractError('Scale overlay changed unrelated frozen spec/range fields')
    for ordinal, (actor, profile) in enumerate(zip(actors, profiles)):
        select(_identity_asset(actor), row, ordinal, actor['motion'], policy, profile_id=profile)
    return plan


def _identity_asset(actor):
    asset = dict(actor)
    if (actor.get('name') in WHITELIST and actor.get('data_id') == WHITELIST[actor['name']][0]
            and actor.get('source_list') == 'objectList'):
        asset.setdefault('semantic_class', 'obstacle')
    return asset


def planned_geometry(asset, row, ordinal, motion, config, size, pivot):
    """Generator/auditor hook; legacy specs have no changed RNG or geometry."""
    plan = validate_plan(row, config)
    if plan is None:
        return list(size), list(pivot), None, None
    if not 0 <= ordinal < len(plan['actor_identity']):
        raise ScaleContractError('Scale actor ordinal outside the frozen population')
    identity = plan['actor_identity'][ordinal]
    if any(identity.get(key) != asset.get(key) for key in ('data_id', 'name', 'source_list')):
        raise ScaleContractError('Scale plan changed asset identity/order')
    if identity['motion'] != motion:
        raise ScaleContractError('Scale plan changed actor motion')
    assignment = select(asset, row, ordinal, motion, config[CONFIG_KEY],
                        profile_id=plan['profile_ids'][ordinal])
    new_size, new_pivot, proof = placement_geometry(size, pivot, assignment, config[CONFIG_KEY])
    return new_size, new_pivot, assignment, proof


def height_initializer(assignment, policy, size, pivot, *, allow_candidate=False):
    """Variant-specific initializer, never an asset-name scale1 saved-height cache."""
    validate_assignment(assignment, policy, allow_candidate=allow_candidate)
    if assignment['profile_id'] == BASE:
        raise ScaleContractError('Original actors must retain their original height initializer')
    if assignment['candidate_only']:
        height = _vector(size, positive=True)[2] / 2 - _vector(pivot)[2]
        mode = 'candidate_bbox_bottom_hypothesis_not_contact_verified'
    else:
        profile = policy['assets'][assignment['name']]['profiles'][assignment['profile_id']]
        height = _number(profile['native_geometry']['pivot_height_above_ground_m'])
        mode = 'qualified_scale_specific_native_contact_initializer'
    return dict(usable=True, policy=POLICY, mode=mode, reason=None,
        data_id=assignment['data_id'], profile_id=assignment['profile_id'],
        pivot_height_candidate_m=height, assignment_sha256=assignment['assignment_sha256'],
        scale1_height_cache_reused=False, contact_verified=False,
        limitation='Qualified profile initializer plus MGeo surface; not arbitrary-position collider certification')


def verify_metadata(scene, metadata, config, catalog):
    """Check scale, placement geometry, identity and contact initializer on disk."""
    row, actors = metadata['spec'], metadata['runtime_actors']
    plan = validate_plan(row, config)
    if plan is None:
        if any(actor.get('asset_scale') is not None for actor in actors):
            raise ScaleContractError('Foreign scale metadata appeared in a legacy scene')
        return None
    if len(actors) != len(plan['actor_identity']):
        raise ScaleContractError('Scale runtime actor count changed')
    for ordinal, actor in enumerate(actors):
        identity = plan['actor_identity'][ordinal]
        if any(actor.get(key) != identity[key] for key in identity):
            raise ScaleContractError('Scale runtime actor identity/motion changed')
        asset = dict(catalog[actor['data_id']])
        assignment = select(asset, row, ordinal, actor['motion'], config[CONFIG_KEY],
                            profile_id=plan['profile_ids'][ordinal])
        if actor.get('asset_scale') != assignment:
            raise ScaleContractError('Scale runtime assignment differs from frozen profile')
        raw = next((value for value in scene.get(actor['source_list'], [])
                    if value.get('UNIQUEID') == actor['unique_id']), None)
        if raw is None:
            raise ScaleContractError('Scaled actor missing from serialized scenario')
        if assignment is not None:
            verify_actor(raw, assignment, config[CONFIG_KEY], asset=asset,
                         row=row, ordinal=ordinal, motion=actor['motion'])
        if assignment and assignment['profile_id'] != BASE:
            profile = config[CONFIG_KEY]['assets'][actor['name']]['profiles'][assignment['profile_id']]
            if actor['estimated_size'] != profile['native_geometry']['bbox_size_m']:
                raise ScaleContractError('Scale native placement size differs from profile')
            if actor.get('static_height_initializer') != height_initializer(
                    assignment, config[CONFIG_KEY], actor['estimated_size'],
                    profile['native_geometry']['pivot_to_bbox_center_local_m']):
                raise ScaleContractError('Scale native contact initializer differs from profile')
    return plan
