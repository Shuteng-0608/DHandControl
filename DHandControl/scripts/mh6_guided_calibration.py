"""Stage-specific natural-to-grasp calibration with short recorded voice cues.

Only stable windows update calibration. Transition samples and unsuccessful
attempts remain in the NPZ, identified by metadata, for later inspection.
"""

import json
import math
from pathlib import Path
import time

import numpy as np

from mh6_hand_session import HandSessionRecorder, STAGED_CALIBRATION_PHASES
from mh6_mapping import MH6HandMapper, FINGER_NAMES, LONG_FINGERS, thumb_rotation_angle

PROTOCOL = "staged_v1"
# Human-feature quality thresholds; unrelated to robot geometry or motor limits.
MIN_SPAN = {"curl": .08, "rotation": .06, "distance": .005}
STABLE_FLOOR = {"curl": .10, "rotation": .08, "distance": .005}


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


def feature_rows(samples):
    mapper = MH6HandMapper()
    rows = []
    for points in samples:
        rows.append({**{f"curl:{f}": x for f, x in mapper.compute_finger_curls(points).items()},
                     **{f"distance:{f}": x for f, x in mapper.compute_opposition_distances(points).items()},
                     "rotation:thumb": thumb_rotation_angle(points)})
    return rows


def metrics(samples):
    if len(samples) < 5:
        raise ValueError("有效采样不足，请保持手部处于追踪范围内")
    rows = feature_rows(samples)
    result = {}
    for key in rows[0]:
        values = np.array([row[key] for row in rows])
        center = float(np.median(values))
        result[key] = {"median": center,
                       "noise": float(1.4826 * np.median(np.abs(values-center))),
                       "spread": float(np.percentile(values, 95)-np.percentile(values, 5))}
    return result


def apply_stage(mapper, phase, rounds, neutral_metrics=None):
    """Validate only the features assigned to this gesture; update no other endpoints."""
    measured = [metrics(samples) for samples in rounds]
    if phase == "neutral":
        for key, value in measured[0].items():
            if value["spread"] > 2 * STABLE_FLOOR[key.split(":")[0]]:
                raise ValueError(f"自然位不稳定：{key}")
        mapper.calibrate_neutral(rounds[0])
        # The outward half is unused in this protocol; preserve the JSON schema.
        c = mapper.calibration
        c.curl_outward = c.curl_open.copy()
        c.opposition_outward_dist = c.opposition_open_dist.copy()
        c.thumb_rotation_outward = c.thumb_rotation_open
        return measured[0]
    if phase == "grasp":
        keys = [f"curl:{finger}" for finger in FINGER_NAMES]
    elif phase == "thumb_rotate":
        keys = ["rotation:thumb"]
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
        required = max(MIN_SPAN[kind], 3*zero["noise"],
                       *(3*m[key]["noise"] for m in measured))
        if span <= required:
            raise ValueError(f"向内运动范围不足：{key}，请重新完成动作")
        if any(m[key]["spread"] > max(STABLE_FLOOR[kind], .30*span) for m in measured):
            raise ValueError(f"到位后保持不稳定：{key}")
        if max(centers)-min(centers) > max(STABLE_FLOOR[kind], .25*span):
            raise ValueError(f"两次动作端点不一致：{key}")
        updates[key] = endpoint
        report[key] = {"neutral": zero["median"], "endpoint": endpoint, "span": span,
                       "required_span": required, "rounds": [m[key] for m in measured]}
    # Update only after every feature of the stage passes.
    for key, endpoint in updates.items():
        kind, finger = key.split(":")
        if kind == "curl": mapper.calibration.curl_closed[finger] = endpoint
        elif kind == "rotation": mapper.calibration.thumb_rotation_closed = endpoint
        else: mapper.calibration.opposition_closed_dist[finger] = endpoint
    return report


def verify_neutral(mapper, samples):
    metrics(samples)  # Validate count, even if the command list would be empty.
    values = np.array([[row["palm_command"][key] for key in
                        ("vertical", "lateral", "thumb_rotation_command")]
                       for row in (mapper.step(points) for points in samples)])
    medians = np.median(values, axis=0)
    if np.any(medians > .15):
        raise ValueError("返回自然位验证未通过，请重新标定自然位")
    return {"median_h_v_r": medians.tolist(), "threshold": .15}


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
        repetitions = 2 if phase in ("grasp", "thumb_rotate") else 1
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
                stage_report = apply_stage(mapper, phase, samples, neutral)
            except ValueError as exc:
                report["attempt_errors"].append({"phase": phase, "attempt": attempt, "error": str(exc)})
                print(f"[{phase}] {exc}" + ("；重试当前阶段。" if attempt == 1 else "；标定停止。"), flush=True)
                if attempt == 2: raise
                continue
            for entry in stage_segments: entry["accepted"] = True
            report["stages"][phase] = stage_report
            if phase == "neutral": neutral = stage_report
            break
    voice.play("neutral")
    capture_window(stream, recorder, move_seconds, rate, "transition")
    data, first, last = capture_window(stream, recorder, neutral_seconds, rate, "verify_neutral")
    try:
        report["verification"] = verify_neutral(mapper, data)
    except ValueError as exc:
        report["verification"] = {"passed": False, "error": str(exc)}
        raise
    segments.append({"phase": "verify_neutral", "round": 1, "attempt": 1,
                     "start": first, "end": last, "accepted": True})
    mapper.calibration.validate()
    report["passed"] = True
    recorder.metadata["calibration_complete"] = True
    return mapper.calibration, report


def calibration_from_staged_stream(stream):
    metadata = stream.metadata
    if metadata.get("calibration_protocol") != PROTOCOL or not metadata.get("calibration_complete"):
        raise ValueError("incomplete staged calibration; no default endpoints will be substituted")
    groups = {phase: [] for phase in (*STAGED_CALIBRATION_PHASES, "verify_neutral")}
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
    for phase in STAGED_CALIBRATION_PHASES:
        expected = 2 if phase in ("grasp", "thumb_rotate") else 1
        if len(groups[phase]) != expected:
            raise ValueError(f"incomplete staged calibration: {phase}")
        result = apply_stage(mapper, phase, groups[phase], neutral)
        if phase == "neutral": neutral = result
    if len(groups["verify_neutral"]) != 1:
        raise ValueError("missing neutral verification")
    verify_neutral(mapper, groups["verify_neutral"][0])
    mapper.calibration.validate()
    return mapper.calibration


def save_report(calibration_path, report):
    path = Path(calibration_path).expanduser().with_suffix(".report.json")
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
