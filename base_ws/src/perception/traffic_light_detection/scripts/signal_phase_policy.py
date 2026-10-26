#!/usr/bin/env python3
"""Profile-driven normalization for competition traffic signals.

The wire format remains Autoware's elemental color/shape array. This module
only converts detector tuples to a phase allowed by the profile assigned to
the current Lanelet traffic-light regulatory-element ID.
"""

import yaml


TOKEN_TO_RESULT = {
    "RED_CIRCLE": ("red", "circle", None),
    "YELLOW_CIRCLE": ("yellow", "circle", None),
    "GREEN_CIRCLE": ("green", "circle", None),
    "GREEN_LEFT_ARROW": ("green", "arrow", True),
    "GREEN_RIGHT_ARROW": ("green", "arrow", False),
}
RESULT_TO_TOKEN = {value: key for key, value in TOKEN_TO_RESULT.items()}


def _phase_key(tokens):
    return frozenset(tokens)


def _parse_correction_key(value):
    return _phase_key(part.strip() for part in value.split("+") if part.strip())


class SignalPhasePolicy:
    def __init__(self, config):
        self.default_profile = str(config.get("default_profile", ""))
        self.assignments = {
            int(group_id): str(profile)
            for group_id, profile in config.get("signal_assignments", {}).items()
        }
        self.profiles = config.get("profiles", {})
        self.section_labels = {
            int(group_id): str(label)
            for group_id, label in (config.get("section_labels") or {}).items()
        }
        self._observation_state = {}
        self._output_observation = {}
        self._last_trusted_source_stamp_ns = {}
        self._validate()

    @classmethod
    def from_yaml(cls, path):
        with open(path, "r", encoding="utf-8") as stream:
            return cls(yaml.safe_load(stream) or {})

    def _validate_phase(self, phase, location):
        if not isinstance(phase, list):
            raise ValueError(f"{location} must be a list")
        unknown = set(phase) - set(TOKEN_TO_RESULT)
        if unknown:
            raise ValueError(f"{location} has unknown lamps: {sorted(unknown)}")

    def _validate(self):
        if self.default_profile not in self.profiles:
            raise ValueError(f"unknown default_profile: {self.default_profile}")
        for group_id, profile_name in self.assignments.items():
            if profile_name not in self.profiles:
                raise ValueError(f"group {group_id} references unknown profile {profile_name}")

        for profile_name, profile in self.profiles.items():
            supported = profile.get("supported_lamps", [])
            self._validate_phase(supported, f"{profile_name}.supported_lamps")
            supported_set = set(supported)
            for index, phase in enumerate(profile.get("valid_phases", [])):
                self._validate_phase(phase, f"{profile_name}.valid_phases[{index}]")
                if not set(phase).issubset(supported_set):
                    raise ValueError(f"{profile_name}.valid_phases[{index}] uses unsupported lamps")
            fallback = profile.get("fallback_phase", [])
            self._validate_phase(fallback, f"{profile_name}.fallback_phase")
            if not set(fallback).issubset(supported_set):
                raise ValueError(f"{profile_name}.fallback_phase uses unsupported lamps")
            for source, target in profile.get("corrections", {}).items():
                source_tokens = list(_parse_correction_key(str(source)))
                self._validate_phase(source_tokens, f"{profile_name}.corrections source")
                self._validate_phase(target, f"{profile_name}.corrections[{source}]")
                if not set(target).issubset(supported_set):
                    raise ValueError(f"{profile_name}.corrections[{source}] uses unsupported lamps")

    def section_label(self, group_id):
        gid = int(group_id)
        if gid in self.section_labels:
            return self.section_labels[gid]
        return "none" if gid == 0 else str(gid)

    def profile_name(self, group_id):
        return self.assignments.get(int(group_id), self.default_profile)

    def apply(self, color_results, group_id):
        profile_name = self.profile_name(group_id)
        profile = self.profiles[profile_name]
        if profile.get("passthrough", False):
            return list(color_results), f"{profile_name}:passthrough"

        observed_tokens = [
            RESULT_TO_TOKEN[result] for result in color_results if result in RESULT_TO_TOKEN
        ]
        observed_key = _phase_key(observed_tokens)
        corrections = {
            _parse_correction_key(str(source)): target
            for source, target in profile.get("corrections", {}).items()
        }

        if observed_key in corrections:
            return self._results(corrections[observed_key]), f"{profile_name}:corrected"

        valid_phases = {
            _phase_key(phase): phase for phase in profile.get("valid_phases", [])
        }
        if observed_key in valid_phases:
            return self._results(valid_phases[observed_key]), f"{profile_name}:valid"

        supported = set(profile.get("supported_lamps", []))
        filtered_tokens = [token for token in observed_tokens if token in supported]
        filtered_key = _phase_key(filtered_tokens)
        if filtered_key in corrections:
            return self._results(corrections[filtered_key]), f"{profile_name}:filtered_corrected"
        if filtered_key in valid_phases:
            return self._results(valid_phases[filtered_key]), f"{profile_name}:filtered"

        # Preserve the profile-selected fallback and historical reason label.
        # The four mission profiles choose RED: ':fail_open' does not mean GO.
        # Departure/passthrough permission belongs to the downstream planner.
        return self._results(profile.get("fallback_phase", [])), f"{profile_name}:fail_open"

    def apply_transition(self, color_results, group_id):
        """Guard against an empty phase reaching the planner.

        D-2 identifies its two yellows by the red lamp (lone Y is normalized
        to RED, R+Y is preserved). This identifies a phase, NOT permission to
        depart: the downstream planner owns that decision and its history.
        """
        profile_name = self.profile_name(group_id)
        profile = self.profiles[profile_name]
        if profile.get("passthrough", False):
            return list(color_results), f"{profile_name}:transition_passthrough"

        observed_tokens = [
            RESULT_TO_TOKEN[result] for result in color_results if result in RESULT_TO_TOKEN
        ]
        # A split vote from the smoother must not leak an empty phase to BVP,
        # where competition fail-open would authorize motion.
        if not observed_tokens:
            return (
                self._results(profile.get("fallback_phase", [])),
                f"{profile_name}:transition_fallback",
            )
        return list(color_results), f"{profile_name}:transition_valid"

    def reset_transition(self, group_id=None):
        if group_id is None:
            self._observation_state.clear()
            self._output_observation.clear()
            self._last_trusted_source_stamp_ns.clear()
        else:
            self._observation_state.pop(int(group_id), None)
            self._output_observation.pop(int(group_id), None)
            self._last_trusted_source_stamp_ns.pop(int(group_id), None)

    def output_observation_metadata(self, group_id):
        """Snapshot the chosen lamp's provenance, separate from a fresh decision.

        A synthetic fallback RED is a policy decision on the current image, not
        a new observed red lamp. The optional source timestamp is deliberately
        not reconstructed from the guard clock for legacy callers.
        """
        return dict(self._output_observation.get(int(group_id), {}))

    def _record_output_observation(self, group_id, stamp, source_stamp_ns,
                                   observed_source_stamp_ns=None, observed_clock=None,
                                   is_observed=False, origin="policy_fallback"):
        self._output_observation[int(group_id)] = {
            "current_image_source_stamp_ns": source_stamp_ns,
            "adopted_observation_source_stamp_ns": observed_source_stamp_ns if is_observed else None,
            "last_trusted_observation_source_stamp_ns": self._last_trusted_source_stamp_ns.get(int(group_id)),
            "adopted_observation_clock_sec": observed_clock if is_observed else None,
            "guard_clock_sec": stamp,
            "output_is_observed": bool(is_observed),
            "output_is_held": bool(is_observed and (
                observed_source_stamp_ns != source_stamp_ns or observed_clock != stamp)),
            "output_origin": origin,
        }

    def process_normalized(self, normalized, reason, group_id, stamp, confidence, smoother,
                           *, source_stamp_ns=None, use_smoothing=True):
        """Keep missing observations out of the phase vote and history (D-2).

        ``stamp`` remains the advancing guard clock, normally rospy.now(). The
        optional integer nanoseconds belong to the original image. Winner/hold
        payloads retain both independently; the existing three-value return API
        remains unchanged. Lamp permission and yellow duration are not changed.
        ``use_smoothing=False`` is an opt-in observation handoff: genuine
        current normalized lamps are immediate, while D missing observations
        retain their existing bounded source-preserving hold/expiry contract.
        It is not a permission debounce; the downstream compliance owner must
        perform fresh, distinct-observation grant confirmation itself.
        """
        if type(use_smoothing) is not bool:
            raise ValueError("use_smoothing must be an explicit bool")
        source_stamp_ns = (source_stamp_ns if type(source_stamp_ns) is int and source_stamp_ns > 0
                           else None)
        valid = not reason.endswith(":fail_open")
        profile = self.profiles[self.profile_name(group_id)]
        limits = profile.get("observation_guard")
        if not limits:
            if valid and normalized and source_stamp_ns is not None:
                self._last_trusted_source_stamp_ns[int(group_id)] = source_stamp_ns
            payload = (normalized, confidence, stamp, source_stamp_ns, valid)
            result = smoother.update(normalized, payload=payload) if use_smoothing else payload
            lamps, conf = result[:2] if result is not None else ([], 0.0)
            lamps, transition = self.apply_transition(lamps, group_id)
            observed = bool(result and result[0] and result[4])
            self._record_output_observation(
                group_id, stamp, source_stamp_ns, result[3] if observed else None,
                result[2] if observed else None, observed,
                ("current_observation" if result[2] == stamp and result[3] == source_stamp_ns
                 else "smoothed_observation") if observed else "policy_fallback")
            return lamps, conf, transition
        state = self._observation_state.setdefault(group_id, {})
        if stamp < state.get("clock", stamp):
            self.reset_transition(group_id)
            smoother.reset()
            state = self._observation_state.setdefault(group_id, {})
        state["clock"] = stamp
        if valid and normalized and source_stamp_ns is not None:
            self._last_trusted_source_stamp_ns[int(group_id)] = source_stamp_ns
        gap = float(limits["max_observation_gap_sec"])
        if stamp - state.get("input_at", stamp) > gap:
            state.pop("output", None)
            state.pop("yellow_since", None)
            smoother.reset()
        if valid:
            state["input_at"] = stamp
        # Abstain on missing observations, rather than giving synthetic red
        # (or UNKNOWN) a vote against the genuinely observed yellow phase.
        payload = (normalized, confidence, stamp, source_stamp_ns)
        result = (smoother.update(normalized, payload=payload) if use_smoothing else payload) if valid else None
        if result and result[0] and stamp - result[2] <= gap:
            lamps, conf, observed_at, observed_source = result
            if observed_at > state.get("observed_at", float("-inf")):
                state["observed_at"] = observed_at
                is_yellow = TOKEN_TO_RESULT["YELLOW_CIRCLE"] in set(lamps)
                if is_yellow:
                    state.setdefault("yellow_since", observed_at)
                else:
                    state.pop("yellow_since", None)
                output, transition = self.apply_transition(lamps, group_id)
                state.update(output=output, confidence=conf, reason=transition,
                             observed_source_stamp_ns=observed_source)
        yellow_expired = (
            "yellow_since" in state and
            stamp - state["yellow_since"] >= float(limits["max_yellow_sec"])
        )
        if "output" in state and stamp - state["observed_at"] <= gap and not yellow_expired:
            observed_source = state.get("observed_source_stamp_ns")
            self._record_output_observation(
                group_id, stamp, source_stamp_ns, observed_source, state["observed_at"], True,
                "current_observation" if state["observed_at"] == stamp and observed_source == source_stamp_ns
                else "bounded_observation_hold")
            return state["output"], state["confidence"], state["reason"] + ":bounded_observation"
        self._record_output_observation(group_id, stamp, source_stamp_ns)
        return self._results(profile["fallback_phase"]), 0.0, "observation_guard:expired_or_unconfirmed"

    @staticmethod
    def _results(tokens):
        return [TOKEN_TO_RESULT[token] for token in tokens]
