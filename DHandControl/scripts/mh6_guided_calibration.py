"""Stage-specific natural-to-grasp calibration with short recorded voice cues.

Only stable windows update calibration. Transition samples and unsuccessful
attempts remain in the NPZ, identified by metadata, for later inspection.
"""

import json
import copy
import hashlib
import math
from pathlib import Path
import time

import numpy as np

from mh6_hand_session import (HandSessionRecorder, ReplayHandStream, STAGED_CALIBRATION_PHASES,
                             M4_SEPARATE_CALIBRATION_PHASES)
from visionpro_stream import VisionProHandFrame
from mh6_mapping import MH6HandMapper, FINGER_NAMES, LONG_FINGERS, thumb_rotation_angle, thumb_abduction_angle

PROTOCOL = "staged_v3"
STAGED_PROTOCOLS = ("staged_v1", "staged_v2", PROTOCOL)


def stages_for_protocol(protocol):
    if protocol not in STAGED_PROTOCOLS:
        raise ValueError("expected staged_v1, staged_v2 or staged_v3 calibration")
    return M4_SEPARATE_CALIBRATION_PHASES if protocol == "staged_v2" else STAGED_CALIBRATION_PHASES
# Human-feature quality thresholds; unrelated to robot geometry or motor limits.
MIN_SPAN = {"curl": .08, "rotation": .06, "abduction": .06, "distance": .005}
STABLE_FLOOR = {"curl": .10, "rotation": .08, "abduction": .08, "distance": .005}


def add_guided_arguments(parser):
    parser.add_argument("--calibration-flow", choices=("staged", "legacy"), default="staged",
                        help="Live calibration: separate grasp/rotation/opposition stages (default), or historical neutral/range capture.")
    parser.add_argument("--calibration-hold-seconds", type=float, default=1.5,
                        help="Stable endpoint sampling time after each voice and movement interval.")
    parser.add_argument("--calibration-move-seconds", type=float, default=3.0,
                        help="Movement time after the voice finishes, before endpoint sampling.")


def validate_timing(rate, hold, move, neutral_seconds):
    for name, value in (("rate", rate), ("hold", hold), ("move", move), ("neutral", neutral_seconds)):
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"calibration {name} time/rate must be finite and positive")
    if rate * min(hold, neutral_seconds) < 5:
        raise ValueError("calibration stable windows must allow at least five samples")


def feature_rows(samples, *, include_m4=True):
    mapper = MH6HandMapper()
    rows = []
    for points in samples:
        rows.append({**{f"curl:{f}": x for f, x in mapper.compute_finger_curls(points).items()},
                     **{f"distance:{f}": x for f, x in mapper.compute_opposition_distances(points).items()},
                     "rotation:thumb": thumb_rotation_angle(points)})
        if include_m4:
            rows[-1]["abduction:thumb"] = thumb_abduction_angle(points, strict=True)
    return rows


def metrics(samples, *, include_m4=True):
    if len(samples) < 5:
        raise ValueError("有效采样不足，请保持手部处于追踪范围内")
    rows = feature_rows(samples, include_m4=include_m4)
    result = {}
    for key in rows[0]:
        values = np.array([row[key] for row in rows])
        center = float(np.median(values))
        result[key] = {"median": center,
                       "noise": float(1.4826 * np.median(np.abs(values-center))),
                       "spread": float(np.percentile(values, 95)-np.percentile(values, 5))}
    return result


def apply_stage(mapper, phase, rounds, neutral_metrics=None, *, include_m4=True, combine_thumb=False):
    """Validate only the features assigned to this gesture; update no other endpoints."""
    measured = [metrics(samples, include_m4=include_m4) for samples in rounds]
    if phase == "neutral":
        for key, value in measured[0].items():
            if value["spread"] > 2 * STABLE_FLOOR[key.split(":")[0]]:
                raise ValueError(f"自然位不稳定：{key}")
        mapper.calibrate_neutral(rounds[0], include_thumb_abduction=include_m4)
        # The outward half is unused in this protocol; preserve the JSON schema.
        c = mapper.calibration
        c.curl_outward = c.curl_open.copy()
        c.opposition_outward_dist = c.opposition_open_dist.copy()
        c.thumb_rotation_outward = c.thumb_rotation_open
        return measured[0]
    if phase == "grasp":
        keys = [f"curl:{finger}" for finger in FINGER_NAMES]
    elif phase == "thumb_rotate":
        keys = ["rotation:thumb", "abduction:thumb"] if include_m4 and combine_thumb else ["rotation:thumb"]
    elif phase == "thumb_abduct":
        keys = ["abduction:thumb"]
    else:
        finger = phase.removeprefix("opp_")
        if finger not in LONG_FINGERS:
            raise ValueError(f"unknown calibration phase: {phase}")
        keys = [f"distance:{finger}"]
    updates, report = {}, {}
    for key in keys:
        kind, finger = key.split(":")
        zero = neutral_metrics[key]
        centers = [m[key]["median"] for m in measured]
        endpoint = float(np.median(centers))
        span = (zero["median"]-endpoint if kind == "distance" else endpoint-zero["median"])
        if kind == "abduction":
            span = abs(span)  # Either operator-selected deviation direction is valid.
        required = max(MIN_SPAN[kind], 3*zero["noise"],
                       *(3*m[key]["noise"] for m in measured))
        label = "M4偏转（拇指向内旋转动作）" if kind == "abduction" and combine_thumb else key
        if span <= required:
            raise ValueError(f"向内运动范围不足：{label}，请重新完成动作")
        if any(m[key]["spread"] > max(STABLE_FLOOR[kind], .30*span) for m in measured):
            raise ValueError(f"到位后保持不稳定：{label}")
        if max(centers)-min(centers) > max(STABLE_FLOOR[kind], .25*span):
            raise ValueError(f"两次动作端点不一致：{label}")
        updates[key] = endpoint
        report[key] = {"neutral": zero["median"], "endpoint": endpoint, "span": span,
                       "required_span": required, "rounds": [m[key] for m in measured]}
        if kind == "abduction":
            report[key].update(unit="radians", neutral_deg=float(np.degrees(zero["median"])),
                               endpoint_deg=float(np.degrees(endpoint)))
    # Update only after every feature of the stage passes.
    for key, endpoint in updates.items():
        kind, finger = key.split(":")
        if kind == "curl": mapper.calibration.curl_closed[finger] = endpoint
        elif kind == "rotation": mapper.calibration.thumb_rotation_closed = endpoint
        elif kind == "abduction": mapper.calibration.thumb_abduction_endpoint_deg = float(np.degrees(endpoint))
        else: mapper.calibration.opposition_closed_dist[finger] = endpoint
    return report


class NeutralVerificationError(ValueError):
    def __init__(self, diagnostics):
        self.diagnostics = diagnostics
        h, v, r = diagnostics["median_h_v_r"]
        m4 = f", m4={diagnostics['m4_return_error']:.4f}" if "m4_return_error" in diagnostics else ""
        super().__init__(f"返回自然位验证未通过：h={h:.4f}, v={v:.4f}, r={r:.4f}{m4}，阈值={diagnostics['threshold']:.2f}")


def verify_neutral(mapper, samples):
    metrics(samples, include_m4=mapper.calibration.has_thumb_abduction_calibration)
    values = np.array([[row["palm_command"][key] for key in
                        ("vertical", "lateral", "thumb_rotation_command")]
                       for row in (mapper.step(points) for points in samples)])
    medians = np.median(values, axis=0)
    diagnostics = {"median_h_v_r": medians.tolist(), "threshold": .15,
                   "passed": bool(np.all(medians <= .15)),
                   "failed_components": [name for name, value in zip(("h", "v", "r"), medians) if value > .15]}
    if mapper.calibration.has_thumb_abduction_calibration:
        zero = mapper.calibration.thumb_abduction_neutral_deg
        span = abs(mapper.calibration.thumb_abduction_endpoint_deg-zero)
        error = float(np.median([abs(float(np.degrees(thumb_abduction_angle(p)))-zero)/span for p in samples]))
        diagnostics["m4_return_error"] = error
        if error > diagnostics["threshold"]:
            diagnostics["passed"] = False
            diagnostics["failed_components"].append("m4")
    if not diagnostics["passed"]:
        raise NeutralVerificationError(diagnostics)
    return diagnostics


def capture_window(stream, recorder, duration, rate, phase):
    start_index = len(recorder.timestamps)
    points = []
    deadline = time.monotonic()+duration
    while time.monotonic() < deadline:
        started = time.monotonic()
        frame = stream.get_latest_frame()
        if frame is not None:
            recorder.record(frame, phase)
            points.append(frame.points.copy())
        delay = min(1/rate-(time.monotonic()-started), deadline-time.monotonic())
        if delay > 0: time.sleep(delay)
    return points, start_index, len(recorder.timestamps)


def finish_neutral_verification(stream, voice, mapper, recorder, *, rate, move_seconds, neutral_seconds):
    """Retry only the final natural pose, retaining successful gesture endpoints."""
    if recorder.metadata.get("calibration_protocol") in ("staged_v2", PROTOCOL) and not mapper.calibration.has_thumb_abduction_calibration:
        raise ValueError("incomplete M4 calibration; capture the thumb motion before final verification")
    report = recorder.metadata["calibration_report"]
    segments = recorder.metadata["calibration_segments"]
    previous_attempts = [s["attempt"] for s in segments if s["phase"] == "verify_neutral"]
    first_attempt = max(previous_attempts, default=0)+1
    for attempt in range(first_attempt, first_attempt+2):
        voice.play("neutral")
        capture_window(stream, recorder, move_seconds, rate, "transition")
        print("[verify_neutral] 请恢复与起始一致的自然姿态，保持采样中…", flush=True)
        data, first, last = capture_window(stream, recorder, neutral_seconds, rate, "verify_neutral")
        segment = {"phase": "verify_neutral", "round": 1, "attempt": attempt,
                   "start": first, "end": last, "accepted": False}
        # Persist the failed window too, before evaluating it.
        segments.append(segment)
        try:
            result = verify_neutral(mapper, data)
        except ValueError as exc:
            result = getattr(exc, "diagnostics", {"passed": False}) | {"error": str(exc)}
            report["verification"] = result
            report.setdefault("verification_attempts", []).append(result)
            print(f"[verify_neutral] {exc}", flush=True)
            if attempt == first_attempt:
                print("只重试返回自然位，已通过的握拳、旋转和对指无需重做。", flush=True)
                continue
            raise
        segment["accepted"] = True
        report["verification"] = result
        report.setdefault("verification_attempts", []).append(result)
        mapper.calibration.validate()
        report["passed"] = True
        recorder.metadata["calibration_complete"] = True
        return


def run_guided_calibration(stream, voice, *, recorder=None, rate=20., neutral_seconds=3.,
                           move_seconds=3., hold_seconds=1.5, connect_timeout=15.):
    validate_timing(rate, hold_seconds, move_seconds, neutral_seconds)
    if getattr(stream, "is_replay", False):
        raise ValueError("guided prompts are for live capture; use saved phase labels for replay")
    recorder = recorder if recorder is not None else HandSessionRecorder("unused_calibration.npz")
    recorder.metadata.update(calibration_protocol=PROTOCOL, calibration_complete=False)
    recorder.metadata["calibration_timing"] = {
        "rate_hz": rate, "neutral_seconds": neutral_seconds,
        "move_seconds": move_seconds, "hold_seconds": hold_seconds,
        "relax_seconds": 2.0,
    }
    segments = recorder.metadata.setdefault("calibration_segments", [])
    report = {"protocol": PROTOCOL, "passed": False, "stages": {}, "attempt_errors": []}
    recorder.metadata["calibration_report"] = report
    print("每个动作做到位后保持，听到自然张开再放松。", flush=True)
    deadline = time.monotonic()+connect_timeout
    while stream.get_latest_frame() is None:
        if time.monotonic() >= deadline:
            raise RuntimeError("no valid hand frame received before --connect-timeout")
        time.sleep(1/rate)
    mapper = MH6HandMapper()
    neutral = None
    for phase in STAGED_CALIBRATION_PHASES:
        repetitions = 2 if phase in ("grasp", "thumb_rotate", "thumb_abduct") else 1
        for attempt in (1, 2):
            samples, stage_segments = [], []
            for repetition in range(repetitions):
                if phase != "neutral":
                    voice.play("neutral")
                    capture_window(stream, recorder, 2., rate, "transition")
                voice.play(phase)
                capture_window(stream, recorder, move_seconds, rate, "transition")
                print(f"[{phase}] 保持姿态，采样中…", flush=True)
                data, first, last = capture_window(stream, recorder,
                                                  neutral_seconds if phase == "neutral" else hold_seconds,
                                                  rate, phase)
                samples.append(data)
                entry = {"phase": phase, "round": repetition+1, "attempt": attempt,
                         "start": first, "end": last, "accepted": False}
                segments.append(entry); stage_segments.append(entry)
            try:
                stage_report = apply_stage(mapper, phase, samples, neutral, combine_thumb=True)
            except ValueError as exc:
                report["attempt_errors"].append({"phase": phase, "attempt": attempt, "error": str(exc)})
                print(f"[{phase}] {exc}" + ("；重试当前阶段。" if attempt == 1 else "；标定停止。"), flush=True)
                if attempt == 2: raise
                continue
            for entry in stage_segments: entry["accepted"] = True
            report["stages"][phase] = stage_report
            if phase == "neutral": neutral = stage_report
            break
    finish_neutral_verification(stream, voice, mapper, recorder, rate=rate,
                                move_seconds=move_seconds, neutral_seconds=neutral_seconds)
    return mapper.calibration, report


def staged_endpoint_mapper(stream):
    """Rebuild only fully accepted gesture endpoints; never declare verification passed."""
    metadata = stream.metadata
    phases = stages_for_protocol(metadata.get("calibration_protocol"))
    include_m4 = metadata["calibration_protocol"] != "staged_v1"
    groups = {phase: [] for phase in (*phases, "verify_neutral")}
    previous_end = 0
    for segment in metadata.get("calibration_segments", []):
        if not segment.get("accepted"): continue
        phase, first, last = segment["phase"], segment["start"], segment["end"]
        if (phase not in groups or not isinstance(first, int) or not isinstance(last, int)
                or not previous_end <= first < last <= len(stream.transforms)
                or not np.all(stream.phases[first:last] == phase)):
            raise ValueError("invalid staged calibration segment")
        groups[phase].append([t[:, :3, 3].copy() for t in stream.transforms[first:last]])
        previous_end = last
    mapper = MH6HandMapper(); neutral = None
    for phase in phases:
        expected = 2 if phase in ("grasp", "thumb_rotate", "thumb_abduct") else 1
        if len(groups[phase]) != expected:
            raise ValueError(f"incomplete staged calibration: {phase}")
        result = apply_stage(mapper, phase, groups[phase], neutral, include_m4=include_m4,
                             combine_thumb=metadata["calibration_protocol"] == PROTOCOL)
        if phase == "neutral": neutral = result
    mapper.calibration.validate()
    return mapper, groups


def calibration_from_staged_stream(stream):
    if not stream.metadata.get("calibration_complete"):
        raise ValueError("incomplete staged calibration; no default endpoints will be substituted")
    mapper, groups = staged_endpoint_mapper(stream)
    if len(groups["verify_neutral"]) != 1:
        raise ValueError("missing neutral verification")
    verify_neutral(mapper, groups["verify_neutral"][0])
    mapper.calibration.validate()
    return mapper.calibration


def load_pending_calibration(path):
    source = ReplayHandStream(path, no_wait=True, hand="right")
    source.start()
    if source.metadata.get("calibration_complete") or np.any(source.phases == "teleop"):
        raise ValueError("resume requires a pending calibration-only recording")
    staged_endpoint_mapper(source)  # Reject incomplete gesture stages before AVP starts.
    return source


def resume_neutral_verification(stream, voice, source, recorder, *, rate, move_seconds,
                               neutral_seconds, connect_timeout=15.):
    mapper, _ = staged_endpoint_mapper(source)
    previous_metadata = copy.deepcopy(source.metadata)
    new_context = copy.deepcopy(recorder.metadata)
    recorder.metadata.update(previous_metadata)
    for key in ("source", "created_at", "hand", "origin", "rate", "calibrate_seconds",
                "teleop_duration", "prepare_seconds", "recording_mode", "calibration_flow"):
        if key in new_context:
            recorder.metadata[key] = new_context[key]
    recorder.metadata.update(calibration_complete=False, calibration_resume={
        "source": str(source.path.resolve()),
        "source_sha256": hashlib.sha256(source.path.read_bytes()).hexdigest(),
        "original_frames": len(source.timestamps),
        "resumed_at": time.time(),
        "rate_hz": rate, "move_seconds": move_seconds, "neutral_seconds": neutral_seconds,
    })
    report = recorder.metadata["calibration_report"]
    report["passed"] = False
    # Preserve original relative times, with the old last frame before new captures.
    base = time.monotonic()-float(source.timestamps[-1])
    for timestamp, transform, phase in zip(source.timestamps, source.transforms, source.phases):
        recorder.record(VisionProHandFrame(transform[:, :3, 3], transform,
                                          base+float(timestamp), "right"), str(phase))
    # Early staged_v1 files omitted the failed final verification's segment label.
    indices = np.flatnonzero(source.phases == "verify_neutral")
    segments = recorder.metadata["calibration_segments"]
    if len(indices) and not any(s["phase"] == "verify_neutral" for s in segments):
        first, last = int(indices[0]), int(indices[-1])+1
        if not np.all(source.phases[first:last] == "verify_neutral"):
            raise ValueError("cannot infer old verification window")
        segments.append({"phase": "verify_neutral", "round": 1, "attempt": 1,
                         "start": first, "end": last, "accepted": False})
        try:
            verify_neutral(mapper, list(source.transforms[first:last, :, :3, 3]))
        except NeutralVerificationError as exc:
            report.setdefault("verification_attempts", []).append(exc.diagnostics | {"error": str(exc)})
    deadline = time.monotonic()+connect_timeout
    while stream.get_latest_frame() is None:
        if time.monotonic() >= deadline:
            raise RuntimeError("no valid hand frame received before --connect-timeout")
        time.sleep(1/rate)
    print("已恢复通过的标定端点，只补做返回自然位验证。", flush=True)
    finish_neutral_verification(stream, voice, mapper, recorder, rate=rate,
                                move_seconds=move_seconds, neutral_seconds=neutral_seconds)
    return mapper.calibration, report


def save_report(calibration_path, report):
    path = Path(calibration_path).expanduser().with_suffix(".report.json")
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
