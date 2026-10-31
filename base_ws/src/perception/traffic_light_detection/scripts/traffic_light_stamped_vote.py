#!/usr/bin/env python3
"""Opt-in, source-preserving 3/2 or 7/4 experiment (not production).

Inputs must be CURRENT, observed-only normalized phases, never the output of a
hold or another smoother. The regulatory group owns the vote: currently
map-qualified physical heads in one group share its program. Physical-head
changes do not reset it, but a missing/currently unqualified head does. Each
vote and winning source retains its actual physical-head provenance.
A matched UNKNOWN is an empty-phase vote. By default startup requires a
majority of agreeing original frames, including before the window fills.
Explicit startup_passthrough instead reproduces SignalSmoother's current-input
passthrough for the first six accepted distinct frames for 7/4. This cold
startup ledger survives age pruning and source errors; only a full group,
identity or clock reset starts it again. R/Y as well as permission changes are
delayed once majority voting applies, even when fewer fresh votes survive.
"""
from collections import Counter, deque
import math


class StampedPhaseVote:
    def __init__(self, window_size=3, max_age_sec=.5, startup_passthrough=False):
        if type(window_size) is not int or window_size not in (3, 7):
            raise ValueError('stamped vote window_size must be 3 or 7')
        if type(startup_passthrough) is not bool:
            raise ValueError('stamped vote startup_passthrough must be an explicit bool')
        max_age_limit = .5 if window_size == 3 else .9
        if (isinstance(max_age_sec, bool) or not isinstance(max_age_sec, (int, float))
                or not math.isfinite(max_age_sec) or not 0 < max_age_sec <= max_age_limit
                or round(max_age_sec * 1e9) < 1):
            raise ValueError(f'stamped vote max_age_sec must be finite in (0,{max_age_limit}] and at least 1ns')
        self.window_size = window_size
        self.majority_count = window_size // 2 + 1
        self.max_age_ns = round(max_age_sec * 1e9)
        self.startup_passthrough = startup_passthrough
        self.reset()

    def reset(self):
        self._votes = deque(maxlen=self.window_size)
        self._identity = None
        self._clock_ns = None
        self._source_ns = None
        self._last_trusted_ns = None
        self._accepted_count = 0

    def update(self, normalized, *, normalized_reason, group_id, head_id,
               source_stamp_ns, now_ns, confidence, identity_valid):
        """Return lamps/confidence/reason and complete source/debug provenance.

        ``identity_valid`` means the current JPEG really selected this housing;
        merely remembering a head during a detector miss is not sufficient.
        All age comparisons use integer original-image nanoseconds. Replaying a
        JPEG never adds a vote, refreshes confidence, or renews its lease.
        """
        debug = dict(enabled=True, candidate_version=4, identity_scope='regulatory_group',
                     window_size=self.window_size, majority_count=self.majority_count,
                     max_age_sec=self.max_age_ns / 1e9,
                     startup_passthrough=self.startup_passthrough,
                     startup_policy=('cold_first_distinct_inputs' if self.startup_passthrough else 'strict_majority'),
                     startup_passthrough_active=False,
                     input_source_stamp_ns=source_stamp_ns,
                     input_normalized_tuples=list(normalized),
                     input_confidence=confidence, input_normalized_reason=normalized_reason,
                     group_id=group_id, head_id=head_id, identity_valid=identity_valid,
                     appended=False, duplicate_source=False, reset_reason=None)

        def result(reason, winner=None, winner_count=0):
            lamps = list(winner['lamps']) if winner else []
            observed = bool(lamps)
            source = winner['source_ns'] if observed else None
            clock = winner['clock_ns'] if observed else None
            winner_head = winner['head_id'] if observed else None
            debug.update(vote_source_stamps_ns=[row['source_ns'] for row in self._votes],
                         available_vote_count=len(self._votes),
                         accepted_count=self._accepted_count,
                         startup_complete=self._accepted_count >= self.window_size,
                         window_full=len(self._votes) == self.window_size,
                         vote_head_ids=[row['head_id'] for row in self._votes],
                         vote_tuples=[list(row['lamps']) for row in self._votes],
                         winner_vote_count=winner_count,
                         winner_source_stamp_ns=source,
                         winner_head_id=winner_head,
                         winner_confidence=winner['confidence'] if observed else 0.)
            return dict(lamps=lamps, confidence=winner['confidence'] if observed else 0.,
                        reason='stamped_vote:' + reason, debug=debug,
                        observation_timing=dict(
                            current_image_source_stamp_ns=source_stamp_ns,
                            adopted_observation_source_stamp_ns=source,
                            adopted_observation_head_id=winner_head,
                            last_trusted_observation_source_stamp_ns=self._last_trusted_ns,
                            adopted_observation_clock_sec=clock / 1e9 if clock is not None else None,
                            guard_clock_sec=now_ns / 1e9 if type(now_ns) is int else None,
                            output_is_observed=observed,
                            output_is_held=bool(observed and (source != source_stamp_ns or clock != now_ns)),
                            output_origin=('stamped_vote_startup_passthrough' if observed and reason == 'startup_passthrough'
                                           else 'stamped_vote_winner' if observed else 'stamped_vote_unknown')))

        if (type(identity_valid) is not bool or not identity_valid
                or type(group_id) is not int or group_id <= 0
                or not isinstance(head_id, str) or not head_id):
            self.reset()
            debug['reset_reason'] = 'identity_loss'
            return result('identity_loss')
        if type(now_ns) is not int or now_ns <= 0:
            self.reset()
            debug['reset_reason'] = 'invalid_clock'
            return result('invalid_clock')
        if self._clock_ns is not None and now_ns < self._clock_ns:
            self.reset()
            debug['reset_reason'] = 'clock_reversal'
        # Map qualification belongs to the caller and is still required on
        # EVERY frame/head. A regulatory group may have several synchronized
        # physical heads; switching between them is not a new signal program.
        identity = group_id
        if self._identity != identity:
            self.reset()
            debug['reset_reason'] = debug['reset_reason'] or 'group_change'
            self._identity = identity
        self._clock_ns = now_ns
        if (type(source_stamp_ns) is not int or source_stamp_ns <= 0
                or not 0 <= now_ns - source_stamp_ns <= self.max_age_ns):
            self._votes.clear()
            debug['reset_reason'] = 'invalid_future_or_stale_source'
            return result('invalid_future_or_stale_source')
        if self._source_ns is not None and source_stamp_ns < self._source_ns:
            # Do not lower the accepted-source high-water mark on an old JPEG.
            self._votes.clear()
            debug['reset_reason'] = 'source_reversal'
            return result('source_reversal')
        while self._votes and now_ns - self._votes[0]['source_ns'] > self.max_age_ns:
            self._votes.popleft()
        duplicate = source_stamp_ns == self._source_ns
        debug['duplicate_source'] = duplicate
        if not duplicate:
            self._source_ns = source_stamp_ns
            # Only the observed-only normalizer may supply lamps. Rejected or
            # missing phases are UNKNOWN votes, never synthetic fallback RED.
            valid_confidence = (not isinstance(confidence, bool)
                                and isinstance(confidence, (int, float))
                                and math.isfinite(confidence) and confidence > 0)
            lamps = (tuple(tuple(lamp) for lamp in normalized)
                     if normalized_reason.endswith(':observed') and valid_confidence else ())
            if lamps:
                self._last_trusted_ns = source_stamp_ns
            self._votes.append(dict(key=frozenset(lamps), lamps=lamps,
                                    source_ns=source_stamp_ns, clock_ns=now_ns, head_id=head_id,
                                    confidence=confidence if lamps else 0.))
            self._accepted_count = min(self.window_size, self._accepted_count + 1)
            debug['appended'] = True
        counts = Counter(row['key'] for row in self._votes)
        if not counts:
            return result('unconfirmed')
        if self.startup_passthrough and self._accepted_count < self.window_size:
            # Use the stored original, not a duplicate caller's altered head,
            # confidence, phase or clock. Age pruning/source errors must not
            # reopen cold startup after its distinct-input budget was consumed.
            current = self._votes[-1]
            debug['startup_passthrough_active'] = True
            return result('startup_passthrough', current, counts[current['key']])
        winner_key, count = counts.most_common(1)[0]
        if count < self.majority_count:
            return result('unconfirmed')
        winner = next(row for row in reversed(self._votes) if row['key'] == winner_key)
        return result('winner' if winner_key else 'unknown_majority', winner, count)
