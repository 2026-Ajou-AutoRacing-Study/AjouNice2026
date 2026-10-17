#!/usr/bin/env python3
"""Explicit frozen-roster recovery through the original chooser, no QA waiver.

The frozen quality loop still owns its 512 geometry / 8 layout-binding bounds,
ordered-roster, primary, split, scale, payload, unit and native-QA checks. No
source spec or generated actor list is edited. A transient chooser row freezes
the original ordered DataIDs while the original chooser still consumes legacy
RNG draws and validates pools/budgets. Install around q.replacement, not around
a later spatial context's gen.build assignment.
"""
import copy
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import threading


POLICY = 'frozen-recovery-roster-preserving-v2'


class PopulationRecoveryContractError(RuntimeError):
    """Malformed source/true contract changes must never become geometry misses."""


class PopulationCountProposalRejected(ValueError):
    """A valid generator proposal cannot fill the frozen actor-count cell."""


def require(value, message):
    if not value:
        raise PopulationRecoveryContractError(message)


def exact(left, right):
    """JSON equality which does not silently equate bool/int or int/float."""
    if type(left) is not type(right):
        return False
    if isinstance(left, dict):
        return left.keys() == right.keys() and all(exact(value, right[key]) for key, value in left.items())
    if isinstance(left, list):
        return len(left) == len(right) and all(exact(a, b) for a, b in zip(left, right))
    return left == right


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_context(manifest):
    manifest = Path(manifest)
    deck = json.loads(manifest.read_text())
    # No reinterpretation of old non-overlay recovery population semantics.
    if not deck.get('split_overlay'):
        return None
    require(isinstance(deck.get('scenes'), list) and len(deck['scenes']) == 1,
            'Population recovery requires one explicit frozen quota cell')
    entry = deck['scenes'][0]
    metadata_path = Path(entry['metadata'])
    metadata = json.loads(metadata_path.read_text())
    specification = metadata.get('spec')
    require(type(specification) is dict and type(specification.get('actors')) is int
            and specification['actors'] >= 0, 'Frozen source actor count must be a nonnegative integer')
    actors = metadata.get('runtime_actors')
    require(type(actors) is list and len(actors) == specification['actors'],
            'Frozen source spec/actual actor count mismatch')
    require(all(type(actor) is dict and type(actor.get('data_id')) is int for actor in actors),
            'Frozen source requires ordered integer DataIDs')
    identities = [actor['data_id'] for actor in actors]
    if 'frozen_population_data_ids' in specification:
        require(exact(specification['frozen_population_data_ids'], identities),
                'Declared frozen DataIDs differ from the actual ordered source roster')
    paths = [manifest, metadata_path, Path(entry['scenario'])]
    return dict(spec=copy.deepcopy(specification), metadata=copy.deepcopy(metadata),
        ordered_data_ids=identities,
        files={str(path):sha(path) for path in paths}, source_id=entry['id'])


def verify_sources(context):
    require(all(sha(path) == expected for path, expected in context['files'].items()),
            'Frozen recovery source changed during proposal generation')


def primary_identity(actor):
    require(type(actor) is dict and type(actor.get('data_id')) is int
            and type(actor.get('source_list')) is str and type(actor.get('motion')) is str,
            'Malformed primary identity/motion in count-changing proposal')
    return {key:copy.deepcopy(actor.get(key)) for key in
            ('data_id', 'source_list', 'motion', 'asset_scale')}


def validate_proposal(context, row_before, row_after, candidate, attempt, state):
    require(exact(row_before, row_after) and exact(row_before, context['spec']),
            'Generator mutated or replaced the frozen input spec')
    require(type(candidate) is dict and type(candidate.get('spec')) is dict,
            'Generated metadata/spec mapping required')
    old, new = context['spec'], candidate['spec']
    require(type(new.get('actors')) is int and new['actors'] >= 0,
            'Generated actor count must be a nonnegative integer')
    changed = {key for key in old.keys() | new.keys()
        if key not in old or key not in new or not exact(old[key], new[key])}
    if not changed:
        return
    require(changed == {'actors'}, 'Replacement changed frozen spec outside actors: '+str(sorted(changed)))
    require('secondary_far_vehicle' not in old,
            'Far-secondary source-only recovery cannot change actor count')
    actors = candidate.get('runtime_actors')
    require(type(actors) is list and len(actors) == new['actors'],
            'Generated spec/actual actor count mismatch')
    require(bool(context['metadata']['runtime_actors']) and bool(actors),
            'Count-only rejection cannot hide a missing/introduced primary')
    require(exact(primary_identity(context['metadata']['runtime_actors'][0]), primary_identity(actors[0])),
            'Count-only rejection cannot hide changed primary identity/motion/scale')
    state['count_only_rejections'] += 1
    require(state['count_only_rejections'] <= 512*state['replacement_calls'],
            'Frozen geometry proposal bound unexpectedly exceeded')
    state['events'].append(dict(source_id=context['source_id'], attempt=attempt,
        original_actors=old['actors'], proposed_actors=new['actors'],
        action='reject_proposal_without_capture', spec_fields_changed=['actors']))
    raise PopulationCountProposalRejected(
        'Frozen population candidate count differs: '+str(old['actors'])+' -> '+str(new['actors']))


@contextmanager
def installed(quality_loop=None):
    """Wrap each replacement's currently active build; restore on every exit.

    The yielded in-memory evidence is diagnostic only, never native approval.
    No files, contexts, rosters, metadata, thresholds or original code are edited.
    """
    if quality_loop is None:
        import run_base5_quality_loop as quality_loop
    original = quality_loop.replacement
    require(not getattr(original, '_population_recovery_policy', None),
            'Population recovery adapter is already installed')
    generator = quality_loop.gen
    state = dict(policy=POLICY, replacement_calls=0, count_only_rejections=0, events=[],
        forced_roster=True, source_roster_preserved=True, transient_choose_row_only=True,
        original_choose_rng_and_pool_budget_checks=True, roster_preserving_choose_calls=0,
        source_spec_modified=False, native_qa_unchanged=True,
        original_geometry_limit=512, original_layout_binding_limit=8)
    ownership = threading.Lock()

    def replacement(manifest, output, *args, **kwargs):
        require(ownership.acquire(False), 'Concurrent replacement cannot share a global build adapter')
        try:
            context = source_context(manifest)
            if context is None:
                return original(manifest, output, *args, **kwargs)
            state['replacement_calls'] += 1
            active_build = generator.build
            chooser = generator.population
            active_choose = chooser.choose
            depth = 0

            def choose(primary, poolkey, pools, geometry, row, rng):
                if not depth or not context['ordered_data_ids'] or 'secondary_far_vehicle' in context['spec']:
                    return active_choose(primary, poolkey, pools, geometry, row, rng)
                before = copy.deepcopy(row)
                require(exact(before, context['spec']), 'Scoped population chooser received another frozen spec')
                require(type(primary) is dict and type(primary.get('data_id')) is int
                    and primary['data_id'] == context['ordered_data_ids'][0],
                    'Scoped population chooser changed the primary asset')
                transient = dict(row, frozen_population_data_ids=list(context['ordered_data_ids']))
                try:
                    # The original chooser first consumes its legacy draws,
                    # then validates the exact supplied roster against pools,
                    # size budgets, vehicle-only and required-secondary rules.
                    result = active_choose(primary, poolkey, pools, geometry, transient, rng)
                finally:
                    require(exact(before, row), 'Original chooser mutated the frozen input spec')
                require(type(result) is list and all(type(actor) is dict
                    and type(actor.get('data_id')) is int for actor in result),
                    'Original chooser returned malformed actor identities')
                require(exact([actor['data_id'] for actor in result], context['ordered_data_ids']),
                    'Original chooser did not preserve the explicit ordered source roster')
                state['roster_preserving_choose_calls'] += 1
                return result

            def build(row, configuration, ctx, attempt=0):
                nonlocal depth
                # A generator's private nested construction retains its own
                # semantics; validate only the final outer proposal metadata.
                if depth:
                    return active_build(row, configuration, ctx, attempt)
                before = copy.deepcopy(row)
                depth += 1
                try:
                    result = active_build(row, configuration, ctx, attempt)
                finally:
                    depth -= 1
                    require(exact(before, row), 'Generator mutated the frozen input spec')
                require(type(result) is tuple and len(result) == 2,
                        'Original generator must return scene and metadata')
                validate_proposal(context, before, row, result[1], attempt, state)
                return result

            generator.build = build
            chooser.choose = choose
            try:
                return original(manifest, output, *args, **kwargs)
            finally:
                changed = generator.build is not build
                chooser_changed = chooser.choose is not choose
                generator.build = active_build
                chooser.choose = active_choose
                verify_sources(context)
                require(not changed and not chooser_changed, 'Population generator/chooser adapter ownership changed')
        finally:
            ownership.release()

    replacement._population_recovery_policy = POLICY
    quality_loop.replacement = replacement
    try:
        yield state
    finally:
        changed = quality_loop.replacement is not replacement
        quality_loop.replacement = original
        require(not changed, 'Population replacement adapter ownership changed')
