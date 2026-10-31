"""Explicit pre/intersection/post strata for the stationary mission.

The image supplied by the team is not a georeferenced official polygon.  These
are bounded MGeo route intervals selected for coverage, not predicted spawn
probabilities or signal/avoidance constraints.  A frozen scene keeps its split,
asset population, phase, yaw and gap contract on every recovery attempt.
"""
from collections import Counter, defaultdict
import copy
import hashlib
import json
import math
import random
import base5_layout_split as layout_split


POLICY = 'stationary-mission-bounded-route-strata-v155'
CONFIG_KEY = PLAN_KEY = 'static_mission_extent'
STRATA = ('before', 'intersection', 'after')
LINKS = dict(before=('A2256W000213',), intersection=('A2256W000222',),
             after=('A2256W000220', 'A2256W000202'))
DEFAULT_COUNTS = dict(before=300, intersection=120, after=180)
ARC_BOUNDS_M = (250., 390.8)


class ExtentContractError(RuntimeError):
    """A source/plan mismatch; do not treat this as an ordinary missing scene."""


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def eligible(spec):
    return (spec.get('family') == 'obstacle' and spec.get('mission') is True
            and bool(spec.get('avoidance_phase')) and not spec.get('shadow_phase'))


def split_for(link):
    return 'val' if int(hashlib.sha256(link.encode()).hexdigest()[:8], 16) / 2**32 < .15 else 'train'


def identity(actors):
    return [[a.get(k) for k in ('data_id', 'unique_id', 'name', 'source_list', 'motion')]
            for a in actors]


def route_sha256(ctx):
    if len(ctx['route']) != len(ctx['arc']):
        raise ExtentContractError('Route points and arc coordinates differ in length')
    return digest([[p[k] for k in ('x', 'y', 'z', 'link_idx')]+[float(arc)]
                   for p, arc in zip(ctx['route'], ctx['arc'])])


def _append(rows, lo, hi):
    if hi-lo < 1e-8:
        return
    if rows and abs(rows[-1][1]-lo) < 1e-7:
        rows[-1][1] = float(hi)
    else:
        rows.append([float(lo), float(hi)])


def configuration(ctx, counts=None, bounds=ARC_BOUNDS_M, split_policy=None):
    """Resolve exact link ownership, including half-segments at link seams."""
    counts = copy.deepcopy(DEFAULT_COUNTS if counts is None else counts)
    if split_policy not in (None, layout_split.POLICY):
        raise ExtentContractError('Unknown mission extent split policy')
    if (set(counts) != set(STRATA) or any(type(v) is not int or v < 1 for v in counts.values())
            or sum(counts.values()) != 600):
        raise ExtentContractError('Exactly 600 positive stationary-mission stratum slots required')
    if (len(bounds) != 2 or not all(math.isfinite(v) for v in bounds)
            or not 237.9 <= bounds[0] < bounds[1] <= 390.8):
        raise ExtentContractError('Mission extent must stay within the reviewed bounded route')
    owner = {link: name for name, links in LINKS.items() for link in links}
    intervals = {name: defaultdict(list) for name in STRATA}
    for p, q, a, b in zip(ctx['route'][:-1], ctx['route'][1:], ctx['arc'][:-1], ctx['arc'][1:]):
        parts = [(float(a), float(b), p.get('link_idx'))]
        if p.get('link_idx') != q.get('link_idx'):
            mid = float(a+b)/2
            parts = [(float(a), mid, p.get('link_idx')), (mid, float(b), q.get('link_idx'))]
        for a, b, link in parts:
            if link in owner:
                splits = ('train', 'val') if split_policy else (split_for(link),)
                for split in splits:
                    _append(intervals[owner[link]][split], max(a, bounds[0]), min(b, bounds[1]))
    serial = {name: dict(parts) for name, parts in intervals.items()}
    if any(not serial[name] for name in STRATA):
        raise ExtentContractError('Reviewed MGeo mission links are absent from this route')
    result = dict(policy=POLICY, counts=counts, route_bounds_m=list(bounds), intervals_m=serial,
        link_ids={key: list(value) for key, value in LINKS.items()},
        route_sha256=route_sha256(ctx),
        traffic_signal_interaction_constraint=False,
        official_georeferenced_polygon=False,
        limitation='Team-image interpretation on bounded MGeo links; coverage prior, not official spawn probabilities')
    if split_policy:
        result['split_policy'] = split_policy
    return result


def _phase_quotas(rows, amount):
    """Largest-remainder allocation keeps phase proportions within a split."""
    sizes = Counter(row['spec']['avoidance_phase'] for _, row in rows)
    total = sum(sizes.values())
    if not 0 <= amount <= total:
        raise ExtentContractError('Stratum quota is incompatible with frozen train/val allocation')
    result = {phase: amount*n//total for phase, n in sizes.items()}
    missing = amount-sum(result.values())
    ranked = sorted(sizes, key=lambda phase: (-(amount*sizes[phase] % total), phase))
    for phase in ranked[:missing]:
        result[phase] += 1
    return result


def assign(metadata, config):
    """Attach plans only to 600 obstacle-mission rows, preserving all old fields.

    Existing link split makes the entire intersection validation-only.  Split
    preservation therefore takes precedence over a requested 180 inside slots.
    """
    settings = config[CONFIG_KEY]
    if settings.get('policy') != POLICY:
        raise ExtentContractError('Unknown static mission extent policy')
    rows = [(ident, meta) for ident, meta in metadata.items() if eligible(meta['spec'])]
    if len(rows) != 600 or any(PLAN_KEY in meta['spec'] for _, meta in rows):
        raise ExtentContractError('One fresh allocation of exactly 600 stationary mission rows required')
    by_split = defaultdict(list)
    for ident, meta in rows:
        by_split[meta['split']].append((ident, meta))
    if settings.get('split_policy') == layout_split.POLICY:
        if not layout_split.enabled(config):
            raise ExtentContractError('Known-map extent requires matching generator policy')
        return _assign_known_map(rows, by_split, settings)
    if set(settings['intervals_m']['before']) != {'train'} or set(settings['intervals_m']['intersection']) != {'val'}:
        raise ExtentContractError('Reviewed before/intersection split geometry changed')
    quotas = dict(train=_phase_quotas(by_split['train'], settings['counts']['before']),
                  val=_phase_quotas(by_split['val'], settings['counts']['intersection']))
    result = {}
    for split, values in by_split.items():
        per_phase = defaultdict(list)
        for ident, meta in values:
            per_phase[meta['spec']['avoidance_phase']].append((ident, meta))
        first = 'before' if split == 'train' else 'intersection'
        for phase, group in per_phase.items():
            # Hash order is independent of previous capture success/visibility.
            group.sort(key=lambda item: (digest([POLICY, item[0]]), item[0]))
            for index, (ident, meta) in enumerate(group):
                name = first if index < quotas[split][phase] else 'after'
                if not settings['intervals_m'][name].get(split):
                    raise ExtentContractError('Assigned stratum has no route interval in frozen split')
                spec = copy.deepcopy(meta['spec'])
                if 'frozen_population_data_ids' in spec and spec['frozen_population_data_ids'] != [
                        a['data_id'] for a in meta['runtime_actors']]:
                    raise ExtentContractError('Existing frozen actor list differs')
                spec['frozen_population_data_ids'] = [a['data_id'] for a in meta['runtime_actors']]
                spec[PLAN_KEY] = dict(policy=POLICY, stratum=name, split=split,
                    source_id=ident, source_spec_sha256=digest(meta['spec']),
                    source_identity=identity(meta['runtime_actors']),
                    settings_sha256=digest(settings), intervals_m=copy.deepcopy(settings['intervals_m'][name][split]))
                result[ident] = spec
    if Counter(spec[PLAN_KEY]['stratum'] for spec in result.values()) != Counter(settings['counts']):
        raise ExtentContractError('Spatial quota cannot satisfy the preserved split')
    return result


def _assign_known_map(rows, by_split, settings):
    """Both splits can use all three strata; preserve phase/identity/old seeds.

    Initial split is only a generation allocation. The final layout overlay
    performs group deduplication and train-coverage allocation after geometry.
    """
    if any(set(settings['intervals_m'][name]) != {'train', 'val'} for name in STRATA):
        raise ExtentContractError('Known-map extent must expose all strata to both splits')
    remaining_counts = dict(settings['counts'])
    result = {}
    for split_index, (split, values) in enumerate(sorted(by_split.items())):
        total = len(values)
        if split_index == len(by_split)-1:
            counts = remaining_counts.copy()
        else:
            raw = {name: settings['counts'][name]*total/len(rows) for name in STRATA}
            counts = {name: int(value) for name, value in raw.items()}
            for name in sorted(STRATA, key=lambda name: (-(raw[name]-counts[name]), name))[:total-sum(counts.values())]:
                counts[name] += 1
        pending = sorted(values, key=lambda item: (digest([POLICY, item[0]]), item[0]))
        for name in STRATA:
            phase_counts = _phase_quotas(pending, counts[name]) if counts[name] else {}
            chosen, kept = [], []
            for item in pending:
                phase = item[1]['spec']['avoidance_phase']
                if phase_counts.get(phase, 0):
                    chosen.append(item)
                    phase_counts[phase] -= 1
                else:
                    kept.append(item)
            pending = kept
            for ident, meta in chosen:
                spec = copy.deepcopy(meta['spec'])
                population = [a['data_id'] for a in meta['runtime_actors']]
                if spec.get('frozen_population_data_ids', population) != population:
                    raise ExtentContractError('Existing frozen actor list differs')
                spec['frozen_population_data_ids'] = population
                spec[PLAN_KEY] = dict(policy=POLICY, stratum=name, split=split,
                    split_policy=layout_split.POLICY, source_id=ident,
                    source_spec_sha256=digest(meta['spec']), source_identity=identity(meta['runtime_actors']),
                    settings_sha256=digest(settings), intervals_m=copy.deepcopy(settings['intervals_m'][name][split]))
                result[ident] = spec
            remaining_counts[name] -= counts[name]
    if Counter(spec[PLAN_KEY]['stratum'] for spec in result.values()) != Counter(settings['counts']):
        raise ExtentContractError('Known-map extent quota differs')
    return result


def validate_plan(row, config):
    plan = row.get(PLAN_KEY)
    if not plan:
        return None
    settings = config.get(CONFIG_KEY, {})
    known_map = settings.get('split_policy') == layout_split.POLICY
    if known_map != (plan.get('split_policy') == layout_split.POLICY) or known_map != layout_split.enabled(config):
        raise ExtentContractError('Extent plan/config split policy mismatch')
    if (not eligible(row) or plan.get('policy') != POLICY or settings.get('policy') != POLICY
            or plan.get('stratum') not in STRATA or plan.get('settings_sha256') != digest(settings)
            or plan.get('intervals_m') != settings.get('intervals_m', {}).get(plan['stratum'], {}).get(plan.get('split'))
            or not plan.get('source_id') or len(plan.get('source_spec_sha256', '')) != 64
            or [a[0] for a in plan.get('source_identity', [])] != row.get('frozen_population_data_ids')):
        raise ExtentContractError('Static mission extent plan/config/identity mismatch')
    for a, b in plan['intervals_m']:
        if not all(math.isfinite(v) for v in (a, b)) or b-a < .1:
            raise ExtentContractError('Invalid mission sampling interval')
    return plan


def anchor(gen, ctx, row, config, attempt=0):
    plan = validate_plan(row, config)
    if plan is None:
        raise ExtentContractError('Extent anchor requires an explicit frozen plan')
    settings = config[CONFIG_KEY]
    route_hash = route_sha256(ctx)
    if route_hash != settings['route_sha256']:
        raise ExtentContractError('MGeo route changed from the frozen extent')
    if type(attempt) is not int or attempt < 0:
        raise ExtentContractError('Nonnegative finite retry ordinal required')
    rng = random.Random(int(digest([POLICY, row['seed'], attempt])[:16], 16))
    pieces = plan['intervals_m']
    length = sum(b-a-.04 for a, b in pieces)
    offset = rng.uniform(0., length)
    for a, b in pieces:
        available = b-a-.04
        if offset <= available:
            arc = a+.02+offset
            break
        offset -= available
    else:
        raise ExtentContractError('Mission arc sampling exhausted unexpectedly')
    base = gen.geo.continuous_route_pose_at_arc(ctx['route'], ctx['arc'], arc)
    if plan.get('split_policy') != layout_split.POLICY and split_for(base['link_idx']) != plan['split']:
        raise ExtentContractError('Mission anchor crossed frozen physical-link split')
    return ctx['route'], ctx['arc'], base, dict(spatial_stratum='static_obstacle',
        static_mission_extent=dict(policy=POLICY, stratum=plan['stratum'], sampled_base_arc_m=arc,
            intervals_m=copy.deepcopy(pieces), primary_center_verified=False,
            traffic_signal_interaction_constraint=False))


def project_primary(ctx, xy, settings, bounded=True):
    """Exact XY segment projection, restricted to the reviewed route envelope."""
    best = None
    lower, upper = settings['route_bounds_m'] if bounded else (0., float(ctx['arc'][-1]))
    for p, q, a, b in zip(ctx['route'][:-1], ctx['route'][1:], ctx['arc'][:-1], ctx['arc'][1:]):
        if b < lower or a > upper or b-a < 1e-9:
            continue
        dx, dy = q['x']-p['x'], q['y']-p['y']
        square = dx*dx+dy*dy
        if square < 1e-12:
            continue
        t = ((xy[0]-p['x'])*dx+(xy[1]-p['y'])*dy)/square
        t = max(max(0., (lower-a)/(b-a)), min(min(1., (upper-a)/(b-a)), t))
        arc = float(a+t*(b-a))
        distance = math.hypot(xy[0]-p['x']-t*dx, xy[1]-p['y']-t*dy)
        candidate = (distance, arc)
        if best is None or candidate < best:
            best = candidate
    if best is None:
        raise ExtentContractError('Reviewed route has no projectable segments')
    return dict(arc_m=best[1], lateral_distance_m=best[0])


def finish(gen, ctx, meta, config):
    plan = validate_plan(meta['spec'], config)
    if plan is None:
        return meta
    if (meta.get('split') != plan['split'] or identity(meta['runtime_actors']) != plan['source_identity']
            or meta.get('details', {}).get('spatial_stratum') != 'static_obstacle'):
        raise ExtentContractError('Extent build changed actor identity/motion/split/mission')
    settings = config[CONFIG_KEY]
    primary = meta['runtime_actors'][0]
    # Do not clamp a primary outside the reviewed envelope to its nearest
    # boundary, or a scene just before 250m could falsely count as inside it.
    projected = project_primary(ctx, primary['geometric_center_estimate'], settings, bounded=False)
    allowed = settings['intervals_m'][plan['stratum']]
    # A footprint-support translation or noncentral pivot can move the actual
    # primary across a seam. Retry geometry rather than counting Ego or base.
    if not any(a <= projected['arc_m'] < b for pieces in allowed.values() for a, b in pieces):
        raise ValueError('Primary center left frozen stationary-mission stratum')
    if projected['lateral_distance_m'] > 8.:
        raise ValueError('Primary center is too far from reviewed mission route')
    if not any(a <= meta['base']['route_arc_m'] < b for a, b in plan['intervals_m']):
        raise ExtentContractError('Extent base left the frozen sampling interval')
    meta['static_mission_extent'] = dict(policy=POLICY, stratum=plan['stratum'], split=plan['split'],
        source_id=plan['source_id'], sampled_base_arc_m=meta['base']['route_arc_m'],
        primary_center_route=projected,
        final_ego_route=project_primary(ctx, [meta['ego']['x'], meta['ego']['y']], settings, bounded=False),
        primary_center_not_ego_used_for_coverage=True,
        traffic_signal_interaction_constraint=False, native_qa_unchanged=True)
    meta['details']['static_mission_extent']['primary_center_verified'] = True
    return meta


def verify_transition(before, after, expected):
    declared = copy.deepcopy(before['spec'])
    declared['frozen_population_data_ids'] = [a['data_id'] for a in before['runtime_actors']]
    declared[PLAN_KEY] = copy.deepcopy(expected[PLAN_KEY])
    if (after['spec'] != expected or expected != declared or before['split'] != after['split']
            or identity(before['runtime_actors']) != identity(after['runtime_actors'])
            or expected[PLAN_KEY]['source_spec_sha256'] != digest(before['spec'])):
        raise ExtentContractError('Extent preparation changed a frozen nonspatial scene contract')
    return True
