"""Explicit nine-actuator settings, independent of Solver geometry.

Null means unknown, never a zero position or an inferred hardware limit.
"""
import copy
import json
from pathlib import Path

DEFAULT_HARDWARE_CONFIG = Path(__file__).resolve().parents[1] / "config/mh6_hardware.json"
FINGERS = ("F1", "F2", "F3", "F4", "F5")  # thumb -> little
PALM = ("M1", "M2", "M3")
CHANNELS = FINGERS + PALM + ("M4",)
COMMON = {"min", "max", "initial_position", "max_position_step"}
OPTIONAL = {"release_position"}


class HardwareConfig:
    def __init__(self, data):
        self.data = copy.deepcopy(data)
        self._validate()

    @classmethod
    def load(cls, path=DEFAULT_HARDWARE_CONFIG):
        return cls(json.loads(Path(path).expanduser().read_text(encoding="utf-8")))

    def to_dict(self):
        return copy.deepcopy(self.data)

    @property
    def fixed_branch(self):
        return self.data["fixed_branch"]

    @property
    def actuators(self):
        return self.data["actuators"]

    def required(self, channel):
        return COMMON | ({"open_position", "closed_position"} if channel in FINGERS else
                         {"neutral_position", "endpoint_position"} if channel == "M4" else set())

    def missing(self, channels=CHANNELS):
        missing = [f"actuators.{name}.{key}" for name in channels
                   for key in sorted(self.required(name)) if self.actuators[name][key] is None]
        return missing

    def palm_diagnostic_limits(self):
        # Missing physical bounds do not redefine the Solver's protocol domain.
        # They still block control acceptance in NineChannelPreview.
        return {i: (self.actuators[name]["min"] if self.actuators[name]["min"] is not None else 0,
                    self.actuators[name]["max"] if self.actuators[name]["max"] is not None else 1000)
                for i, name in enumerate(PALM, 1)}

    def _validate(self):
        d = self.data
        if isinstance(d, dict) and "m4_mapping" in d:
            raise ValueError("M4 human angles now belong to the Mapping calibration file; remove hardware m4_mapping and use schema_version=2")
        if not isinstance(d, dict) or set(d) != {"schema_version", "fixed_branch", "jump_guard", "servo_move_time_ms", "actuators"}:
            raise ValueError("invalid hardware configuration fields")
        if type(d["schema_version"]) is not int or d["schema_version"] != 2:
            raise ValueError("unsupported hardware schema_version")
        if d["fixed_branch"] not in ("plus_acos", "minus_acos"):
            raise ValueError("fixed_branch must be plus_acos or minus_acos")
        if d["jump_guard"] != "position":
            raise ValueError("only position jump protection is supported")
        if type(d["servo_move_time_ms"]) is not int or not 1 <= d["servo_move_time_ms"] <= 65535:
            raise ValueError("servo_move_time_ms must be an integer in [1,65535]")
        if not isinstance(d["actuators"], dict) or set(d["actuators"]) != set(CHANNELS):
            raise ValueError("actuators must contain exactly F1..F5 and M1..M4")
        for name, values in d["actuators"].items():
            if (not isinstance(values, dict) or not self.required(name) <= set(values)
                    or set(values)-self.required(name)-OPTIONAL):
                raise ValueError(f"invalid fields for actuator {name}")
            ceiling = 2000 if name in FINGERS else 1000
            for key, value in values.items():
                if value is None:
                    continue
                if type(value) is not int or not 0 <= value <= ceiling:
                    raise ValueError(f"{name}.{key} must be null or an integer in [0,{ceiling}]")
                if key == "max_position_step" and value <= 0:
                    raise ValueError(f"{name}.max_position_step must be positive")
            lo, hi = values["min"], values["max"]
            if lo is not None and hi is not None and lo >= hi:
                raise ValueError(f"{name}: min must be less than max")
            for key in ("initial_position", "open_position", "closed_position", "neutral_position", "endpoint_position", "release_position"):
                value = values.get(key)
                if value is not None and ((lo is not None and value < lo) or (hi is not None and value > hi)):
                    raise ValueError(f"{name}.{key} lies outside configured bounds")
            endpoints = ("open_position", "closed_position") if name in FINGERS else ("neutral_position", "endpoint_position")
            a, b = (values.get(key) for key in endpoints)
            if a is not None and b is not None and a == b:
                raise ValueError(f"{name}: mapping endpoints must differ")
