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

    @staticmethod
    def _results(tokens):
        return [TOKEN_TO_RESULT[token] for token in tokens]
