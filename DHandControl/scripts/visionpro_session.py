#!/usr/bin/env python3
"""AVP hand capture, staged human calibration and playback, without motor drivers."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from typing import Optional, Sequence

import numpy as np

from mh6_hand_session import HandSessionRecorder, ReplayHandStream, SESSION_PHASES, STAGED_CALIBRATION_PHASES
from visionpro_stream import VisionProHandStream
from mh6_calibration_voice import CalibrationVoice, add_voice_arguments
from mh6_guided_calibration import (
    add_guided_arguments, run_guided_calibration, validate_timing,
    load_pending_calibration, resume_neutral_verification,
)


RECORDING_MODES = {
    "teleop": ("teleop",),
    "calibration": ("neutral", "range"),
    "full": ("neutral", "range", "teleop"),
}


def positive_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return number


def nonnegative_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise argparse.ArgumentTypeError("must be finite and nonnegative")
    return number


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    record = commands.add_parser("record", help="Record AVP hand transforms only")
    record.add_argument("--avp-ip", required=True, help="Vision Pro IP address or room code")
    record.add_argument("--output", required=True, help="Output compressed NPZ session")
    record.add_argument("--hand", choices=("left", "right"), default="right")
    record.add_argument("--origin", choices=("avp", "sim"), default="avp")
    record.add_argument(
        "--mode", choices=tuple(RECORDING_MODES), default="teleop",
        help="Record teleop actions (default), calibration only, or all phases",
    )
    record.add_argument("--rate", type=positive_float, default=20.0, help="Sampling rate in Hz")
    record.add_argument("--calibrate-seconds", type=positive_float, default=3.0)
    record.add_argument("--range-calibrate-seconds", type=positive_float, default=8.0,
                        help="Historical neutral/range flow only: motion range sampling time.")
    record.add_argument(
        "--duration", type=positive_float,
        help="Seconds of teleop actions; omit to stop with Ctrl+C",
    )
    record.add_argument(
        "--prepare-seconds", type=nonnegative_float, default=3.0,
        help="Legacy/teleop capture preparation time; staged gestures use --calibration-move-seconds.",
    )
    record.add_argument(
        "--connect-timeout", type=positive_float, default=15.0,
        help="Maximum wait for the first valid hand frame",
    )
    record.add_argument("--overwrite", action="store_true", help="Replace an existing recording")
    record.add_argument("--mapping-output", help="Also save staged human mapping JSON and a quality report.")
    record.add_argument("--resume-calibration", help="Pending staged NPZ whose gesture stages passed; capture only final neutral verification into a new output file.")
    add_voice_arguments(record)
    add_guided_arguments(record)

    replay = commands.add_parser("replay", help="Preview recorded hand points without AVP")
    replay.add_argument("--input", required=True, help="Recorded NPZ session")
    replay.add_argument("--phase", choices=SESSION_PHASES, default="teleop")
    replay.add_argument("--speed", type=positive_float, default=1.0)
    replay.add_argument("--loop", action="store_true", help="Loop the teleop phase")
    replay.add_argument("--no-wait", action="store_true", help="Read every frame without sleeping")
    replay.add_argument("--print-every-frame", action="store_true")

    info = commands.add_parser("info", help="Validate a session and show its contents")
    info.add_argument("--input", required=True, help="Recorded NPZ session")

    calibrate = commands.add_parser("calibrate", help="Export fixed mapping parameters from calibration frames")
    calibrate.add_argument("--input", required=True, help="NPZ containing neutral and range phases")
    calibrate.add_argument("--output", required=True, help="Output MappingCalibration JSON")
    calibrate.add_argument("--overwrite", action="store_true")
    return parser.parse_args(argv)


def record_hand_session(
    stream,
    recorder: HandSessionRecorder,
    *,
    rate_hz: float = 20.0,
    calibrate_seconds: float = 2.0,
    range_calibrate_seconds: float = 8.0,
    duration: Optional[float] = None,
    prepare_seconds: float = 3.0,
    connect_timeout: float = 15.0,
    mode: str = "teleop",
) -> None:
    """Sample a started hand source; retain raw frames for the requested phases.

    The caller owns source start/stop and recorder saving, including interruption.
    Calibration phases only label samples here; no calibration algorithm runs.
    """
    for value in (rate_hz, calibrate_seconds, range_calibrate_seconds, connect_timeout):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("rate, calibration durations and connection timeout must be positive")
    if not math.isfinite(prepare_seconds) or prepare_seconds < 0.0:
        raise ValueError("preparation time must be finite and nonnegative")
    if duration is not None and (not math.isfinite(duration) or duration <= 0.0):
        raise ValueError("teleop duration must be finite and positive")
    if mode not in RECORDING_MODES:
        raise ValueError(f"unknown recording mode: {mode}")

    period = 1.0 / rate_hz
    print("Waiting for a valid hand frame...", flush=True)
    deadline = time.monotonic() + connect_timeout
    while stream.get_latest_frame() is None:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise RuntimeError("no valid hand frame received before --connect-timeout")
        time.sleep(min(period, remaining))

    phases = (
        ("neutral", calibrate_seconds, "Keep the hand in a relaxed natural pose."),
        ("range", range_calibrate_seconds,
         "Perform TWO cycles: relax naturally, then close/grasp fully."),
        ("teleop", duration, "Perform the actions to replay in later experiments."),
    )
    for phase, seconds, instruction in phases:
        if phase not in RECORDING_MODES[mode]:
            continue
        print(f"[{phase}] {instruction}", flush=True)
        if prepare_seconds > 0.0:
            print(f"Starting in {prepare_seconds:g}s...", flush=True)
            time.sleep(prepare_seconds)
        print(f"Recording {phase}; press Ctrl+C to save and stop.", flush=True)
        started = time.monotonic()
        deadline = started + seconds if seconds is not None else None
        first_index = len(recorder.timestamps)
        next_report = started + 1.0
        while deadline is None or time.monotonic() < deadline:
            loop_start = time.monotonic()
            frame = stream.get_latest_frame()
            if frame is not None:
                recorder.record(frame, phase)
            now = time.monotonic()
            if now >= next_report:
                print(
                    f"[{phase}] elapsed={now - started:.1f}s "
                    f"frames={len(recorder.timestamps) - first_index}",
                    flush=True,
                )
                next_report = now + 1.0
            sleep_time = period - (time.monotonic() - loop_start)
            if deadline is not None:
                sleep_time = min(sleep_time, deadline - time.monotonic())
            if sleep_time > 0.0:
                time.sleep(sleep_time)
        count = len(recorder.timestamps) - first_index
        if count == 0:
            raise RuntimeError(f"no valid frames recorded in '{phase}'; check hand tracking")
        print(f"[{phase}] captured {count} frames", flush=True)


def record_session(args: argparse.Namespace) -> int:
    path = Path(args.output).expanduser()
    if path.exists() and not args.overwrite:
        print(f"ERROR: recording already exists: {path}; use another name or --overwrite")
        return 2
    staged = args.calibration_flow == "staged" and args.mode != "teleop"
    try:
        voice = None
        resume_source = None
        if args.resume_calibration:
            if not staged:
                raise ValueError("--resume-calibration requires staged calibration/full recording")
            resume_source = load_pending_calibration(args.resume_calibration)
            if resume_source.path.resolve() == path.resolve():
                raise ValueError("resume output must differ from the source recording")
        if args.mapping_output:
            if not staged:
                raise ValueError("--mapping-output requires staged calibration/full recording")
            mapping_path = Path(args.mapping_output).expanduser()
            report_path = mapping_path.with_suffix(".report.json")
            if len({p.resolve() for p in (path, mapping_path, report_path)}) != 3:
                raise ValueError("recording, mapping, and report must have distinct paths")
            if resume_source is not None and resume_source.path.resolve() in (mapping_path.resolve(), report_path.resolve()):
                raise ValueError("resume outputs must not overwrite the source recording")
            if not args.overwrite and any(p.exists() for p in (mapping_path, report_path)):
                raise ValueError("mapping/report already exists; use another name or --overwrite")
        if staged:
            if args.hand != "right":
                raise ValueError("staged MH6 calibration requires --hand right")
            validate_timing(args.rate, args.calibration_hold_seconds,
                            args.calibration_move_seconds, args.calibrate_seconds)
            voice = CalibrationVoice(args.calibration_voice_dir, enabled=not args.no_calibration_voice)
    except (RuntimeError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}")
        return 2
    stream = VisionProHandStream(args.avp_ip, hand=args.hand, origin=args.origin)
    recorder = HandSessionRecorder(str(path), metadata={
        "source": "visionpro_session",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "hand": args.hand,
        "origin": args.origin,
        "rate": args.rate,
        "calibrate_seconds": args.calibrate_seconds,
        "range_calibrate_seconds": args.range_calibrate_seconds,
        "teleop_duration": args.duration,
        "prepare_seconds": args.prepare_seconds,
        "recording_mode": args.mode,
        "calibration_flow": args.calibration_flow,
    })
    status = 0
    completion_played = False
    try:
        stream.start()
        if staged:
            if resume_source is not None:
                calibration, report = resume_neutral_verification(
                    stream, voice, resume_source, recorder, rate=args.rate,
                    neutral_seconds=args.calibrate_seconds,
                    move_seconds=args.calibration_move_seconds,
                    connect_timeout=args.connect_timeout,
                )
            else:
                calibration, report = run_guided_calibration(
                    stream, voice, recorder=recorder, rate=args.rate,
                    neutral_seconds=args.calibrate_seconds,
                    move_seconds=args.calibration_move_seconds,
                    hold_seconds=args.calibration_hold_seconds,
                    connect_timeout=args.connect_timeout,
                )
            if args.mapping_output:
                from mh6_guided_calibration import save_report
                calibration.save_json(args.mapping_output)
                save_report(args.mapping_output, report)
                print(f"Saved mapping calibration: {args.mapping_output}")
            if args.mode == "full":
                recorder.save(finalize=False)
                voice.play("complete")
                completion_played = True
        if not staged or args.mode == "full":
            record_hand_session(
                stream, recorder, rate_hz=args.rate,
                calibrate_seconds=args.calibrate_seconds,
                range_calibrate_seconds=args.range_calibrate_seconds,
                duration=args.duration, prepare_seconds=args.prepare_seconds,
                connect_timeout=args.connect_timeout,
                mode="teleop" if staged else args.mode,
            )
    except KeyboardInterrupt:
        print("Recording stopped; saving captured frames.")
        recorder.metadata["interrupted"] = True
    except (RuntimeError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}")
        status = 2
    finally:
        try:
            saved_path = recorder.save()
            if saved_path is None:
                print("No valid frames captured; no recording saved.")
                status = status or 1
            else:
                print(f"Saved {len(recorder.timestamps)} frames: {saved_path}")
                required = set(STAGED_CALIBRATION_PHASES) if staged else set(RECORDING_MODES[args.mode])
                if staged and args.mode == "full": required.add("teleop")
                missing = sorted(required - set(recorder.phases))
                if missing:
                    print(f"Incomplete session; missing phases: {', '.join(missing)}")
                    status = status or 1
                if staged and not recorder.metadata.get("calibration_complete"):
                    status = status or 1
                if staged and status == 0 and not completion_played:
                    voice.play("complete")
        except (OSError, ValueError) as exc:
            print(f"ERROR: failed to save recording: {exc}")
            status = 2
        finally:
            stream.stop()
    return status


def replay_session(args: argparse.Namespace) -> int:
    if args.loop and args.phase != "teleop":
        print("ERROR: --loop is supported only for --phase teleop")
        return 2
    stream = ReplayHandStream(args.input, speed=args.speed, loop=args.loop, no_wait=args.no_wait)
    count = 0
    try:
        stream.start()
        stream.set_phase(args.phase)
        next_print = 0.0
        first_timestamp = None
        while True:
            frame = stream.get_latest_frame()
            if frame is not None:
                count += 1
                if first_timestamp is None:
                    first_timestamp = frame.timestamp
                now = time.monotonic()
                if args.print_every_frame or now >= next_print:
                    print(
                        f"frames={count} hand={frame.hand} "
                        f"time={frame.timestamp - first_timestamp:.3f}s "
                        f"wrist={np.array2string(frame.points[0], precision=3)} "
                        f"index_tip={np.array2string(frame.points[9], precision=3)}",
                        flush=True,
                    )
                    next_print = now + 0.2
            if stream.phase_finished and not args.loop:
                break
            if frame is None and not args.no_wait:
                time.sleep(min(0.01, 0.01 / args.speed))
        print(f"Replay completed: {count} frames.")
    except KeyboardInterrupt:
        print(f"Replay stopped: {count} frames.")
    except (RuntimeError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}")
        return 2
    finally:
        stream.stop()
    return 0


def session_info(path: str) -> int:
    stream = ReplayHandStream(path)
    try:
        stream.start()
        print(f"Session: {stream.path}")
        print(f"Frames: {len(stream.timestamps)}; hands: {', '.join(np.unique(stream.hands))}")
        for phase in SESSION_PHASES:
            timestamps = stream.timestamps[stream.phases == phase]
            span = float(timestamps[-1] - timestamps[0]) if len(timestamps) else 0.0
            print(f"{phase}: {len(timestamps)} frames, span={span:.3f}s")
        missing = sorted({"neutral", "range", "teleop"} - set(stream.phases))
        present = set(stream.phases)
        if stream.metadata.get("calibration_protocol") == "staged_v1":
            print("Staged calibration:", "complete" if stream.metadata.get("calibration_complete") else "INCOMPLETE")
            print("Export JSON with 'calibrate'; stable windows and attempts are stored in metadata.")
        elif present == {"teleop"}:
            print("Teleop-only session: use --use-default-calibration, --mapping-calibration, "
                  "or --calibration-session in the MH6 runner.")
        elif present == {"neutral", "range"}:
            print("Calibration-only session: export JSON with 'calibrate', "
                  "or use --calibration-session in the MH6 runner.")
        elif missing:
            print(f"Incomplete for MH6 teleop replay; missing phases: {', '.join(missing)}")
        if np.any(stream.hands != "right"):
            print("MH6 mapping runner currently accepts right-hand sessions only.")
        print("Metadata:", json.dumps(stream.metadata, ensure_ascii=False, indent=2))
    except (RuntimeError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}")
        return 2
    finally:
        stream.stop()
    return 0


def export_calibration(args: argparse.Namespace) -> int:
    # Mapping is needed only for this offline export, never for raw capture.
    from mh6_mapping_calibration import calibration_from_session

    path = Path(args.output).expanduser()
    if (path.exists() or path.with_suffix(".report.json").exists()) and not args.overwrite:
        print(f"ERROR: calibration already exists: {path}; use another name or --overwrite")
        return 2
    try:
        calibration_from_session(args.input).save_json(str(path))
        with np.load(args.input, allow_pickle=False) as archive:
            metadata = json.loads(str(archive["metadata_json"].item()))
        if metadata.get("calibration_protocol") == "staged_v1":
            from mh6_guided_calibration import save_report
            save_report(path, metadata["calibration_report"])
        print(f"Saved fixed mapping calibration: {path}")
    except (RuntimeError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}")
        return 2
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    if args.command == "record":
        return record_session(args)
    if args.command == "replay":
        return replay_session(args)
    if args.command == "calibrate":
        return export_calibration(args)
    return session_info(args.input)


if __name__ == "__main__":
    raise SystemExit(main())
