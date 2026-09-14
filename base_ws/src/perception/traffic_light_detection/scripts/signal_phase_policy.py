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
        self._last_non_yellow_phase = {}
        self._observation_state = {}
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
            conditional_yellow = profile.get("conditional_yellow")
            if conditional_yellow is not None:
                for index, phase in enumerate(conditional_yellow.get("go_after_phases", [])):
                    self._validate_phase(
                        phase, f"{profile_name}.conditional_yellow.go_after_phases[{index}]"
                    )
                    if not set(phase).issubset(supported_set):
                        raise ValueError(
                            f"{profile_name}.conditional_yellow.go_after_phases[{index}] "
                            "uses unsupported lamps"
                        )
                stop_phase = conditional_yellow.get("stop_phase", [])
                self._validate_phase(
                    stop_phase, f"{profile_name}.conditional_yellow.stop_phase"
                )
                if not set(stop_phase).issubset(supported_set):
                    raise ValueError(
                        f"{profile_name}.conditional_yellow.stop_phase uses unsupported lamps"
                    )

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

        # Competition policy is intentionally fail-open to avoid DNF. Each
        # profile chooses a route-compatible GO phase rather than publishing a
        # generic green that could authorize the wrong movement.
        return self._results(profile.get("fallback_phase", [])), f"{profile_name}:fail_open"

    def apply_transition(self, color_results, group_id):
        """Resolve phases whose permission depends on the preceding stable phase.

        This is intentionally called after temporal smoothing. In particular,
        D-2 has both straight-green -> yellow (STOP for our left-turn route) and
        protected-left -> yellow (GO), which cannot be distinguished from the
        current yellow lamps alone.
        """
        profile_name = self.profile_name(group_id)
        profile = self.profiles[profile_name]
        if profile.get("passthrough", False):
            return list(color_results), f"{profile_name}:transition_passthrough"

        observed_tokens = [
            RESULT_TO_TOKEN[result] for result in color_results if result in RESULT_TO_TOKEN
        ]
        observed_key = _phase_key(observed_tokens)
        yellow_key = _phase_key(["YELLOW_CIRCLE"])
        conditional_yellow = profile.get("conditional_yellow")

        # A split vote from the smoother must not leak an empty phase to BVP,
        # where competition fail-open would authorize motion. Use the
        # profile's explicit fallback; C and D-2 configure this as red.
        if not observed_key:
            return (
                self._results(profile.get("fallback_phase", [])),
                f"{profile_name}:transition_fallback",
            )

        if conditional_yellow is not None and observed_key == yellow_key:
            allowed_predecessors = {
                _phase_key(phase)
                for phase in conditional_yellow.get("go_after_phases", [])
            }
            if self._last_non_yellow_phase.get(int(group_id)) in allowed_predecessors:
                return list(color_results), f"{profile_name}:yellow_go_after_allowed_phase"
            return (
                self._results(conditional_yellow.get("stop_phase", [])),
                f"{profile_name}:yellow_stop_after_disallowed_phase",
            )

        if observed_key and observed_key != yellow_key:
            self._last_non_yellow_phase[int(group_id)] = observed_key
        return list(color_results), f"{profile_name}:transition_valid"

    def reset_transition(self, group_id=None):
        if group_id is None:
            self._last_non_yellow_phase.clear()
            self._observation_state.clear()
        else:
            self._last_non_yellow_phase.pop(int(group_id), None)
            self._observation_state.pop(int(group_id), None)

    def process_normalized(self, normalized, reason, group_id, stamp, confidence, smoother):
        """Keep missing observations out of the phase vote and history (D-2).

        Payload timestamps are observation times, not repeated winner times.
        This prevents a stale smoothing winner from refreshing a GO lease.
        """
        profile = self.profiles[self.profile_name(group_id)]
        limits = profile.get("observation_guard")
        if not limits:
            result = smoother.update(normalized, payload=(normalized, confidence))
            lamps, conf = result if result is not None else ([], 0.0)
            lamps, transition = self.apply_transition(lamps, group_id)
            return lamps, conf, transition
        state = self._observation_state.setdefault(group_id, {})
        if stamp < state.get("clock", stamp):
            self.reset_transition(group_id)
            smoother.reset()
            state = self._observation_state.setdefault(group_id, {})
        state["clock"] = stamp
        valid = not reason.endswith(":fail_open")
        gap = float(limits["max_observation_gap_sec"])
        if stamp - state.get("input_at", stamp) > gap:
            self._last_non_yellow_phase.pop(group_id, None)
            state.pop("output", None)
            state.pop("yellow_since", None)
            smoother.reset()
        if valid:
            state["input_at"] = stamp
        # Abstain on missing observations, rather than giving synthetic red
        # (or UNKNOWN) a vote against the genuinely observed yellow phase.
        result = smoother.update(normalized, payload=(normalized, confidence, stamp)) if valid else None
        if result and result[0] and stamp - result[2] <= gap:
            lamps, conf, observed_at = result
            if observed_at > state.get("observed_at", float("-inf")):
                state["observed_at"] = observed_at
                is_yellow = set(lamps) == {TOKEN_TO_RESULT["YELLOW_CIRCLE"]}
                if is_yellow:
                    state.setdefault("yellow_since", observed_at)
                else:
                    state.pop("yellow_since", None)
                output, transition = self.apply_transition(lamps, group_id)
                state.update(output=output, confidence=conf, reason=transition)
        yellow_expired = (
            "yellow_since" in state and
            stamp - state["yellow_since"] >= float(limits["max_yellow_sec"])
        )
        if yellow_expired:
            self._last_non_yellow_phase.pop(group_id, None)
        if "output" in state and stamp - state["input_at"] <= gap and not yellow_expired:
            return state["output"], state["confidence"], state["reason"] + ":bounded_observation"
        return self._results(profile["fallback_phase"]), 0.0, "observation_guard:expired_or_unconfirmed"

    @staticmethod
    def _results(tokens):
        return [TOKEN_TO_RESULT[token] for token in tokens]
