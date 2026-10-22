"""Nine-channel offline command preparation. No serial or hardware imports.

M1..M3 accept an entire fixed-root solution or hold the entire previous vector.
Fingers/M4 have independent position guards. Proposing never advances history.
"""
import copy
import math

from mh6_hardware_config import CHANNELS, FINGERS, PALM

FINGER_KEYS = ("u_thumb", "u_index", "u_middle", "u_ring", "u_little")


def finite(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


class NineChannelPreview:
    def __init__(self, config):
        self.config = config
        # These are explicitly user-specified *simulation* starting references.
        # They must not be treated as measured actual positions in future hardware code.
        self.previous = {name: config.actuators[name]["initial_position"] for name in CHANNELS}
        self.generation = 0

    def _axis(self, name, requested, missing=None):
        spec = self.config.actuators[name]
        previous = self.previous[name]
        reason = None
        if missing or self.config.missing((name,)):
            reason = "CONFIG_REQUIRED"
        elif not finite(requested):
            reason = "INVALID_INPUT"
        elif not spec["min"] <= requested <= spec["max"]:
            reason = "OUT_OF_LIMITS"
        else:
            requested_integer = int(round(requested))
            if abs(requested_integer-previous) > spec["max_position_step"]:
                reason = "POSITION_JUMP"
        accepted = reason is None
        return {"requested_position": requested if finite(requested) else None,
                "integer_target": int(round(requested)) if finite(requested) else None,
                "position": int(round(requested)) if accepted else previous,
                "accepted": accepted, "held_previous": not accepted and previous is not None,
                "status": "ACCEPTED" if accepted else ("UNCONFIGURED" if reason == "CONFIG_REQUIRED" else "HELD_"+reason),
                "reason": reason, "position_delta": int(round(requested))-previous if finite(requested) and previous is not None else None,
                "max_position_step": spec["max_position_step"]}

    def propose(self, result, solver_result):
        channels = {}
        for name, key in zip(FINGERS, FINGER_KEYS):
            value = result["low_dim"].get(key)
            spec = self.config.actuators[name]
            a, b = spec["open_position"], spec["closed_position"]
            requested = a + min(max(value, 0), 1)*(b-a) if finite(value) and a is not None and b is not None else None
            channels[name] = self._axis(name, requested)
            channels[name]["normalized_input"] = min(max(value, 0), 1) if finite(value) else None

        candidates = solver_result["candidates"]
        ids = solver_result["candidate_branch_ids"]
        if len(candidates) != len(ids):
            raise ValueError("candidate branch identities must align")
        matches = [(i, c) for i, (branch, c) in enumerate(zip(ids, candidates))
                   if branch == self.config.fixed_branch and len(c) == 3 and all(finite(x) for x in c)]
        reference = [self.previous[name] for name in PALM]
        index, candidate = min(matches, key=lambda item: sum(
            (round(x)-(p if p is not None else 0))**2 for x, p in zip(item[1], reference))) if matches else (None, None)
        missing = self.config.missing(PALM)
        if missing:
            reason = "CONFIG_REQUIRED"
        elif candidate is None:
            reason = "FIXED_BRANCH_UNAVAILABLE"
        elif any(not self.config.actuators[name]["min"] <= x <= self.config.actuators[name]["max"]
                 for name, x in zip(PALM, candidate)):
            reason = "OUT_OF_LIMITS"
        elif any(abs(int(round(x))-self.previous[name]) > self.config.actuators[name]["max_position_step"]
                 for name, x in zip(PALM, candidate)):
            reason = "POSITION_JUMP"
        else:
            reason = None
        for i, name in enumerate(PALM):
            channels[name] = self._axis(name, candidate[i] if candidate is not None else None)
            if reason:
                channels[name].update(position=self.previous[name], accepted=False,
                                      held_previous=self.previous[name] is not None,
                                      status="UNCONFIGURED" if reason == "CONFIG_REQUIRED" else "HELD_"+reason,
                                      reason=reason)

        # A distinct geometric feature and calibration: never reuse Solver's r.
        raw = result.get("intent", {}).get("thumb_abduction_raw")
        raw_deg = math.degrees(raw) if finite(raw) else None
        mapping = result.get("m4_mapping", {})
        normalized = result["low_dim"].get("u_thumb_abduction")
        m4_calibrated = mapping.get("calibrated") is True
        if not m4_calibrated or not finite(normalized) or not 0 <= normalized <= 1 or raw_deg is None:
            normalized = None
        spec = self.config.actuators["M4"]
        start, end = spec["neutral_position"], spec["endpoint_position"]
        requested = start + normalized*(end-start) if normalized is not None and start is not None and end is not None else None
        missing_mapping = [] if m4_calibrated else ["mapping.thumb_abduction_neutral_deg", "mapping.thumb_abduction_endpoint_deg"]
        channels["M4"] = self._axis("M4", requested, missing=missing_mapping)
        return {"mode": "offline_nine_channel", "hardware_enabled": False,
                "servo_move_time_ms": self.config.data["servo_move_time_ms"],
                "generation": self.generation, "fixed_branch": self.config.fixed_branch,
                "jump_guard": "position", "missing_configuration": self.config.missing()+missing_mapping,
                "channels": channels, "positions": {name: channels[name]["position"] for name in CHANNELS},
                "palm": {"accepted": reason is None, "reason": reason,
                         "selected_candidate_index": index,
                         "requested_motor": list(candidate) if candidate is not None else None,
                         "selected_motor": [channels[name]["position"] for name in PALM]},
                "m4_mapping": {"source": "thumb_abduction_raw", "raw_angle_deg": raw_deg,
                               "human_neutral_deg": mapping.get("human_neutral_deg"),
                               "human_endpoint_deg": mapping.get("human_endpoint_deg"),
                               "calibrated": m4_calibrated,
                               "normalized": normalized, "requested_position": requested}}

    def commit_preview(self, proposal):
        """Advance simulated history only, after the caller records the preview.

        Deliberately named preview: not an acknowledgement from a real actuator.
        A future sender must separately handle successful/failed/uncertain writes.
        """
        if proposal["generation"] != self.generation:
            raise ValueError("stale preview cannot be committed")
        self.previous = copy.deepcopy(proposal["positions"])
        self.generation += 1

    def commit_acknowledged(self, proposal):
        """Advance last acknowledged targets after a successful driver write.

        These are command references, never measured actual motor positions.
        """
        if proposal["generation"] != self.generation:
            raise ValueError("stale command cannot be committed")
        if prepare_packet(proposal) is None:
            raise ValueError("incomplete command cannot be acknowledged")
        self.previous = copy.deepcopy(proposal["positions"])
        self.generation += 1


def prepare_packet(proposal):
    """Build a nine-channel driver payload, without opening a serial port.

    It is intentionally unavailable until every channel is configured and known.
    F and M IDs are on different actuator buses; M4 belongs to the servo bus.
    """
    if proposal["missing_configuration"] or any(x is None for x in proposal["positions"].values()):
        return None
    positions = proposal["positions"]
    return {"finger_ids": [1, 2, 3, 4, 5],
            "finger_positions": [positions[name] for name in FINGERS],
            "palm_ids": [1, 2, 3, 4],
            "palm_positions": [positions[name] for name in PALM+("M4",)],
            "palm_times": [proposal["servo_move_time_ms"]]*4}
