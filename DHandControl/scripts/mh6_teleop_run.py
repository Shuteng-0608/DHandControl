#!/usr/bin/env python3
"""
Plain Vision Pro to MH6 mapping runner.

Flow:
VisionProHandStream -> neutral/range calibration -> MH6HandMapper -> printed intent.

Hardware output remains explicitly locked while signed commands are validated.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
import time
from numbers import Real
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from mh6_mapping import MH6HandMapper
from mh6_hand_session import HandSessionRecorder, ReplayHandStream
from mh6_palm_fallback import PalmFallbackController
from mh6_palm_solver import MH6PalmSolver
from mh6_palm_solution_selector import PalmInputSlewLimiter, PalmSolutionSelector
from visionpro_stream import VisionProHandStream


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Vision Pro to MH6 mapping runner")
    parser.add_argument("--avp-ip", default="192.168.8.145", help="IP address of the Apple Vision Pro device")
    parser.add_argument("--hand", choices=("right",), default="right")
    parser.add_argument("--origin", choices=("avp", "sim"), default="avp")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--calibrate-seconds", type=float, default=2.0)
    parser.add_argument(
        "--range-calibrate-seconds",
        type=float,
        default=8.0,
        help="Time for two over-extension and grasp cycles used to capture motion bounds",
    )
    parser.add_argument("--enable-hardware", action="store_true")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Modbus serial port, required with --enable-hardware")
    parser.add_argument("--baudrate", type=int, default=115200)
    session_group = parser.add_mutually_exclusive_group()
    session_group.add_argument(
        "--record-session",
        help="Record validated raw hand transforms to a compressed NPZ session.",
    )
    session_group.add_argument(
        "--replay-session",
        help="Replay a previously recorded NPZ session instead of Apple Vision Pro.",
    )
    parser.add_argument(
        "--replay-speed",
        type=float,
        default=1.0,
        help="Replay speed multiplier; timestamps and filters follow playback time.",
    )
    parser.add_argument(
        "--replay-loop",
        action="store_true",
        help="Loop only the teleoperation phase of a replay session.",
    )
    parser.add_argument(
        "--replay-no-wait",
        action="store_true",
        help="Process replay frames deterministically without wall-clock sleeping.",
    )
    parser.add_argument(
        "--debug-log",
        help="Write every solver and final control result as JSON Lines.",
    )
    parser.add_argument(
        "--print-every-frame",
        action="store_true",
        help="Print solver and control previews every processed frame instead of at 5 Hz.",
    )
    parser.add_argument(
        "--filter-tau",
        type=float,
        default=0.24,
        help=(
            "First-order low-pass filter time constant in seconds for hardware "
            "commands. Use 0 to disable filtering."
        ),
    )
    parser.add_argument(
        "--max-filter-dt",
        type=float,
        default=0.10,
        help="Maximum dt used by filters and slew guards after a frame gap.",
    )
    parser.add_argument(
        "--tracking-timeout",
        type=float,
        default=0.25,
        help="Seconds without a valid frame before tracking is marked lost.",
    )
    parser.add_argument(
        "--palm-input-speed",
        type=float,
        default=2.0,
        help="Maximum signed palm-input change per second before solving.",
    )
    parser.add_argument(
        "--palm-fallback-delay",
        type=float,
        default=0.5,
        help="Continuous solver no-solution time before preview fallback activates.",
    )
    parser.add_argument(
        "--palm-fallback-speed",
        type=float,
        default=1.0,
        help="Maximum fallback whole-palm closure change per second.",
    )
    parser.add_argument(
        "--palm-fallback-vertical-weight",
        type=float,
        default=0.8,
        help="Vertical contribution to fallback closure; lateral uses one minus this value.",
    )
    parser.add_argument(
        "--print-filtered",
        action="store_true",
        help="Also print the explicitly labeled filtered command layer.",
    )
    return parser.parse_args(argv)


def send_to_hardware_placeholder(result: Dict[str, Dict[str, float]]) -> None:
    """Intentionally no-op when hardware output is disabled."""
    _ = result


class HardwareSender:
    """Prepared actual-position output to the MH6 hardware driver."""

    def __init__(self, port: str, baudrate: int) -> None:
        self.port = port
        self.baudrate = baudrate
        self.hand = None

    def start(self) -> None:
        try:
            from modbus_dev import DexHandControl
        except ImportError as exc:
            raise RuntimeError(
                "DexHandControl could not be imported. Install the Modbus dependencies "
                "and ensure modbus_dev.py is available."
            ) from exc

        self.hand = DexHandControl(port=self.port, baudrate=self.baudrate)
        if not self.hand.start_persistent_connection():
            self.hand = None
            raise RuntimeError("failed to start persistent Modbus connection")

    def stop(self) -> None:
        if self.hand is not None:
            self.hand.stop_persistent_connection()
            self.hand = None

    def send(self, result: Dict[str, Dict[str, float]], palm_selection) -> bool:
        if self.hand is None:
            return False
        finger_values = [
            max(float(result["low_dim"][key]), 0.0)
            for key in ("u_thumb", "u_index", "u_middle", "u_ring", "u_little")
        ]
        finger_positions = self.hand.map_finger_positions(finger_values)
        palm_positions = self.hand.validate_palm_motor_positions(
            palm_selection.selected_motor
        )
        return self.hand.move_hand(
            finger_ids=[1, 2, 3, 4, 5],
            finger_positions=[finger_positions[motor_id] for motor_id in (1, 2, 3, 4, 5)],
            palm_ids=[1, 2, 3],
            palm_positions=[palm_positions[motor_id] for motor_id in (1, 2, 3)],
            palm_times=[80, 80, 80],
            wait_status=False,
        )


class CommandLowPassFilter:
    """Time-aware first-order low-pass filter for normalized command sections."""

    def __init__(
        self,
        tau: float,
        nominal_dt: float = 0.05,
        max_dt: float = 0.10,
        initial_value: float = 0.0,
    ) -> None:
        if tau <= 0.0:
            raise ValueError("tau must be positive")
        if nominal_dt <= 0.0 or max_dt <= 0.0:
            raise ValueError("nominal_dt and max_dt must be positive")
        self.tau = float(tau)
        self.nominal_dt = float(nominal_dt)
        self.max_dt = float(max_dt)
        self.initial_value = float(initial_value)
        self.previous: Dict[tuple, float] = {}
        self.previous_timestamp: Optional[float] = None

    def reset(self) -> None:
        self.previous.clear()
        self.previous_timestamp = None

    def apply(
        self,
        result: Dict[str, Dict[str, float]],
        now: float,
    ) -> Dict[str, Dict[str, float]]:
        filtered_result = copy.deepcopy(result)

        dt = min(self.nominal_dt, self.max_dt)
        if self.previous_timestamp is not None:
            dt = min(max(now - self.previous_timestamp, 0.0), self.max_dt)
        alpha = 1.0 - math.exp(-dt / self.tau) if dt > 0.0 else 0.0
        alpha = min(max(alpha, 0.0), 1.0)

        for section_name in ("low_dim", "palm_command"):
            raw_section = result.get(section_name, {})
            filtered_section = filtered_result.get(section_name, {})
            for key, value in raw_section.items():
                if not isinstance(value, Real) or isinstance(value, bool):
                    continue

                state_key = (section_name, key)
                current = float(value)
                previous = self.previous.get(state_key, self.initial_value)
                filtered = previous + alpha * (current - previous)
                filtered_section[key] = filtered
                self.previous[state_key] = filtered

        self.previous_timestamp = now
        if "palm_command" in filtered_result:
            filtered_result["palm"] = copy.deepcopy(filtered_result["palm_command"])
            filtered_result["palm_fold"] = copy.deepcopy(
                filtered_result["palm_command"]
            )
        return filtered_result


class PalmDebugLogger:
    """Write every solver observation and final preview output as JSON Lines."""

    def __init__(self, path: str) -> None:
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("w", encoding="utf-8", buffering=1)
        self.first_timestamp: Optional[float] = None

    def write(self, timestamp: float, preview) -> None:
        solver_preview, fallback = preview
        requested, applied, candidates, selection = solver_preview
        if self.first_timestamp is None:
            self.first_timestamp = float(timestamp)
        record = {
            "timestamp": float(timestamp) - self.first_timestamp,
            "solver": {
                "requested": requested,
                "applied": applied,
                "candidates": [[float(value) for value in row] for row in candidates],
                "candidate_count": len(candidates),
                "selected_motor": [float(value) for value in selection.selected_motor],
                "status": selection.status,
                "held_previous": bool(selection.held_previous),
                "normalized_jump": selection.normalized_jump,
            },
            "control": {
                "mode": fallback.mode,
                "status": fallback.status,
                "signed_closure": fallback.signed_closure,
                "requested_closure": fallback.requested_closure,
                "applied_closure": fallback.applied_closure,
                "selected_motor": [float(value) for value in fallback.selected_motor],
                "no_solution_duration": fallback.no_solution_duration,
                "entry_distance": fallback.entry_distance,
            },
        }
        self.file.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")

    def close(self) -> None:
        if not self.file.closed:
            self.file.close()


def set_stream_phase(stream, phase: str) -> None:
    setter = getattr(stream, "set_phase", None)
    if callable(setter):
        setter(phase)


def collect_hand_samples(
    stream,
    duration: float,
    rate_hz: float,
    *,
    recorder: Optional[HandSessionRecorder] = None,
    phase: str = "neutral",
) -> List[np.ndarray]:
    set_stream_phase(stream, phase)
    period = 1.0 / rate_hz
    playback_period = period / float(getattr(stream, "speed", 1.0))
    deadline = time.monotonic() + duration
    samples: List[np.ndarray] = []
    is_replay = bool(getattr(stream, "is_replay", False))
    no_wait = bool(getattr(stream, "no_wait", False))

    while (not is_replay and time.monotonic() < deadline) or (
        is_replay and not stream.phase_finished
    ):
        loop_start = time.monotonic()
        frame = stream.get_latest_frame()
        if frame is not None:
            samples.append(frame.points)
            if recorder is not None:
                recorder.record(frame, phase)

        sleep_time = playback_period - (time.monotonic() - loop_start)
        if sleep_time > 0.0 and not no_wait:
            time.sleep(sleep_time)

    return samples


def collect_open_hand_samples(
    stream: VisionProHandStream,
    duration: float,
    rate_hz: float,
) -> List[np.ndarray]:
    """Compatibility alias for callers using the previous function name."""

    return collect_hand_samples(stream, duration, rate_hz)


def print_mapping_line(
    result: Dict[str, Dict[str, float]],
    label: str,
    include_features: bool = True,
) -> None:
    low_dim = result["low_dim"]
    opposition = result["opposition"]
    opposition_signed = result["opposition_signed"]
    grasp_intent = result["grasp_intent"]
    palm_command = result["palm_command"]
    fingers = (
        f"T={low_dim['u_thumb']:.2f} "
        f"TR={low_dim['u_thumb_rotation']:.2f} "
        f"I={low_dim['u_index']:.2f} "
        f"M={low_dim['u_middle']:.2f} "
        f"R={low_dim['u_ring']:.2f} "
        f"L={low_dim['u_little']:.2f}"
    )
    opposition_text = (
        f"I={opposition_signed['p_I']:.2f}/{opposition['p_I']:.2f} "
        f"M={opposition_signed['p_M']:.2f}/{opposition['p_M']:.2f} "
        f"R={opposition_signed['p_R']:.2f}/{opposition['p_R']:.2f} "
        f"L={opposition_signed['p_L']:.2f}/{opposition['p_L']:.2f}"
    )
    grasp_text = (
        f"power={grasp_intent['power_grasp']:.2f} "
        f"tripodFlex={grasp_intent['tripod_flexion']:.2f} "
        f"tripodNear={grasp_intent['tripod_proximity']:.2f} "
        f"tripod={grasp_intent['tripod_precision']:.2f} "
        f"crossOpp={grasp_intent['opposition_cross']:.2f}"
    )
    palm_text = (
        f"vertical={palm_command['vertical']:.2f} "
        f"lateral={palm_command['lateral']:.2f} "
        f"thumbMeasured={palm_command['thumb_rotation_measured']:.2f} "
        f"thumbComp={palm_command['thumb_rotation_compensation']:.2f} "
        f"thumbCommand={palm_command['thumb_rotation_command']:.2f}"
    )
    if include_features:
        print(
            f"{label} features: opposition signed/intent: {opposition_text} | "
            f"grasp: {grasp_text}"
        )
    print(f"{label} commands: signedBending: {fingers} | palmFold: {palm_text}")


def extract_palm_normalized_inputs(
    result: Dict[str, Dict[str, float]],
) -> Dict[str, float]:
    """Extract the three named, signed inputs expected by the palm solver."""
    palm_command = result["palm_command"]
    return {
        "palm_flexion": float(palm_command["vertical"]),
        "palm_cross": float(palm_command["lateral"]),
        "thumb_inward": float(palm_command["thumb_rotation_command"]),
    }


PALM_SOLVER_INPUT_NAMES = (
    "palm_flexion",
    "thumb_inward",
    "palm_cross",
)


def palm_solver_input_values(inputs: Dict[str, float]) -> Tuple[float, float, float]:
    """Return named palm inputs in the positional order required by the solver."""
    return tuple(float(inputs[name]) for name in PALM_SOLVER_INPUT_NAMES)


def named_palm_inputs(values: Sequence[float]) -> Dict[str, float]:
    """Restore solver-ordered values to the stable semantic log field order."""
    if len(values) != len(PALM_SOLVER_INPUT_NAMES):
        raise ValueError("palm solver input must contain exactly three values")
    solver_inputs = dict(zip(PALM_SOLVER_INPUT_NAMES, values))
    return {
        "palm_flexion": float(solver_inputs["palm_flexion"]),
        "palm_cross": float(solver_inputs["palm_cross"]),
        "thumb_inward": float(solver_inputs["thumb_inward"]),
    }


def solve_palm_motor_preview(
    result: Dict[str, Dict[str, float]],
    solver: MH6PalmSolver,
):
    """Solve palm motor candidates from filtered named palm commands."""

    normalized_inputs = extract_palm_normalized_inputs(result)
    motor_solutions = solver.solve_motor_from_normalized(
        *palm_solver_input_values(normalized_inputs)
    )
    return normalized_inputs, motor_solutions


def select_palm_motor_preview(
    result: Dict[str, Dict[str, float]],
    solver: MH6PalmSolver,
    selector: PalmSolutionSelector,
    input_limiter: Optional[PalmInputSlewLimiter] = None,
    timestamp: Optional[float] = None,
):
    """Solve all branches and select the safest continuous preview target."""

    requested_inputs = extract_palm_normalized_inputs(result)
    requested_values = palm_solver_input_values(requested_inputs)
    applied_values = (
        input_limiter.apply(requested_values, timestamp=timestamp)
        if input_limiter is not None
        else requested_values
    )
    applied_inputs = named_palm_inputs(applied_values)
    motor_solutions = solver.solve_motor_from_normalized(*applied_values)
    selection = selector.select(
        applied_values,
        motor_solutions,
        timestamp=timestamp,
    )
    if selection.held_previous and input_limiter is not None:
        input_limiter.hold(selector.previous_valid_input or (0.0, 0.0, 0.0))
    return requested_inputs, applied_inputs, motor_solutions, selection


def select_palm_control_preview(
    result: Dict[str, Dict[str, float]],
    solver: MH6PalmSolver,
    selector: PalmSolutionSelector,
    fallback: PalmFallbackController,
    input_limiter: Optional[PalmInputSlewLimiter] = None,
    timestamp: Optional[float] = None,
):
    """Run the normal solver preview and the print-only degraded controller."""

    solver_preview = select_palm_motor_preview(
        result,
        solver,
        selector,
        input_limiter=input_limiter,
        timestamp=timestamp,
    )
    requested_inputs, _, _, selection = solver_preview
    fallback_result = fallback.update(
        requested_inputs["palm_flexion"],
        requested_inputs["palm_cross"],
        selection,
        timestamp=timestamp,
    )
    return solver_preview, fallback_result


def print_palm_motor_preview(
    preview,
) -> None:
    requested_inputs, applied_inputs, motor_solutions, selection = preview
    requested_text = " ".join(
        f"{name}={value:.3f}" for name, value in requested_inputs.items()
    )
    applied_text = " ".join(
        f"{name}={value:.3f}" for name, value in applied_inputs.items()
    )
    candidate_text = " | ".join(
        f"candidate_{index}=[M1={motors[0]:.2f}, M2={motors[1]:.2f}, "
        f"M3={motors[2]:.2f}]"
        for index, motors in enumerate(motor_solutions, start=1)
    )
    selected = selection.selected_motor
    selected_text = (
        f"selected=[M1={selected[0]:.2f}, M2={selected[1]:.2f}, "
        f"M3={selected[2]:.2f}] status={selection.status}"
    )
    if selection.normalized_jump is not None:
        selected_text += f" jump={selection.normalized_jump:.4f}"
    if candidate_text:
        print(
            f"palm solver preview: requested=({requested_text}) "
            f"applied=({applied_text}) -> {candidate_text} | {selected_text}"
        )
    else:
        print(
            f"palm solver preview: requested=({requested_text}) "
            f"applied=({applied_text}) -> {selected_text}"
        )


def print_palm_control_preview(preview) -> None:
    solver_preview, fallback = preview
    print_palm_motor_preview(solver_preview)
    selected = fallback.selected_motor
    print(
        "palm control preview: "
        f"mode={fallback.mode} status={fallback.status} "
        f"signedClosure={fallback.signed_closure:.3f} "
        f"closure={fallback.requested_closure:.3f}/"
        f"{fallback.applied_closure:.3f} "
        f"noSolution={fallback.no_solution_duration:.3f}s "
        f"entryDistance={fallback.entry_distance:.4f} "
        f"output=[M1={selected[0]:.2f}, M2={selected[1]:.2f}, "
        f"M3={selected[2]:.2f}]"
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    if args.rate <= 0.0:
        print("ERROR: --rate must be greater than 0")
        return 2
    if args.calibrate_seconds <= 0.0:
        print("ERROR: --calibrate-seconds must be greater than 0")
        return 2
    if args.range_calibrate_seconds <= 0.0:
        print("ERROR: --range-calibrate-seconds must be greater than 0")
        return 2
    if args.filter_tau < 0.0:
        print("ERROR: --filter-tau must be greater than or equal to 0")
        return 2
    if args.max_filter_dt <= 0.0:
        print("ERROR: --max-filter-dt must be greater than 0")
        return 2
    if args.tracking_timeout <= 0.0:
        print("ERROR: --tracking-timeout must be greater than 0")
        return 2
    if args.palm_input_speed <= 0.0:
        print("ERROR: --palm-input-speed must be greater than 0")
        return 2
    if args.palm_fallback_delay < 0.0:
        print("ERROR: --palm-fallback-delay must be greater than or equal to 0")
        return 2
    if args.palm_fallback_speed <= 0.0:
        print("ERROR: --palm-fallback-speed must be greater than 0")
        return 2
    if not 0.0 <= args.palm_fallback_vertical_weight <= 1.0:
        print("ERROR: --palm-fallback-vertical-weight must be within [0,1]")
        return 2
    if args.replay_speed <= 0.0:
        print("ERROR: --replay-speed must be greater than 0")
        return 2
    if (args.replay_loop or args.replay_no_wait) and not args.replay_session:
        print("ERROR: replay options require --replay-session")
        return 2
    if args.enable_hardware and not args.port:
        print("ERROR: --port is required with --enable-hardware")
        return 2
    if args.enable_hardware:
        print(
            "ERROR: signed mapping is currently print-test only; hardware output "
            "is intentionally blocked until signed motor commands are validated"
        )
        return 2

    stream = (
        ReplayHandStream(
            args.replay_session,
            speed=args.replay_speed,
            loop=args.replay_loop,
            no_wait=args.replay_no_wait,
        )
        if args.replay_session
        else VisionProHandStream(
            avp_ip=args.avp_ip,
            hand=args.hand,
            origin=args.origin,
        )
    )
    recorder = (
        HandSessionRecorder(
            args.record_session,
            metadata={
                "hand": args.hand,
                "origin": args.origin,
                "rate": args.rate,
                "calibrate_seconds": args.calibrate_seconds,
                "range_calibrate_seconds": args.range_calibrate_seconds,
            },
        )
        if args.record_session
        else None
    )
    debug_logger = PalmDebugLogger(args.debug_log) if args.debug_log else None
    mapper = MH6HandMapper()
    palm_solver = MH6PalmSolver()
    period = 1.0 / args.rate
    playback_period = period / float(getattr(stream, "speed", 1.0))
    palm_solution_selector = PalmSolutionSelector(
        nominal_dt=period,
        max_dt=args.max_filter_dt,
    )
    palm_input_limiter = PalmInputSlewLimiter(
        max_speed_per_sec=(args.palm_input_speed,) * 3,
        nominal_dt=period,
        max_dt=args.max_filter_dt,
    )
    palm_fallback = PalmFallbackController(
        vertical_weight=args.palm_fallback_vertical_weight,
        activation_delay=args.palm_fallback_delay,
        max_closure_speed_per_sec=args.palm_fallback_speed,
        nominal_dt=period,
        max_dt=args.max_filter_dt,
    )
    next_print = 0.0
    command_filter = (
        CommandLowPassFilter(
            args.filter_tau,
            nominal_dt=period,
            max_dt=args.max_filter_dt,
            initial_value=0.0,
        )
        if args.filter_tau > 0.0
        else None
    )
    hardware_sender = (
        HardwareSender(port=args.port, baudrate=args.baudrate)
        if args.enable_hardware
        else None
    )

    try:
        stream.start()

        print("Keep the right hand in a relaxed natural pose for neutral calibration...")
        neutral_samples = collect_hand_samples(
            stream,
            args.calibrate_seconds,
            args.rate,
            recorder=recorder,
            phase="neutral",
        )
        if not neutral_samples:
            print("ERROR: no valid samples collected during neutral calibration")
            return 1

        mapper.calibrate_neutral(neutral_samples)
        print(f"Collected {len(neutral_samples)} neutral-pose samples")

        print(
            "Perform TWO quick cycles now: over-extend all fingers, then close/grasp "
            "the hand through its comfortable full range..."
        )
        range_samples = collect_hand_samples(
            stream,
            args.range_calibrate_seconds,
            args.rate,
            recorder=recorder,
            phase="range",
        )
        if not range_samples:
            print("ERROR: no valid samples collected during motion-range calibration")
            return 1
        mapper.calibrate_motion_range(range_samples)
        print(f"Collected {len(range_samples)} motion-range samples")
        print("curl outward:", mapper.calibration.curl_outward)
        print("curl neutral:", mapper.calibration.curl_open)
        print("curl inward:", mapper.calibration.curl_closed)
        print("distance outward:", mapper.calibration.opposition_outward_dist)
        print("distance neutral:", mapper.calibration.opposition_open_dist)
        print("distance inward:", mapper.calibration.opposition_closed_dist)
        print(
            "thumb rotation outward/neutral/inward:",
            mapper.calibration.thumb_rotation_outward,
            mapper.calibration.thumb_rotation_open,
            mapper.calibration.thumb_rotation_closed,
        )

        if hardware_sender is not None:
            print("WARNING: HARDWARE OUTPUT ENABLED. The MH6 hand will move.")
            hardware_sender.start()

        print("Entering mapping loop. Press Ctrl-C to stop.")
        print("NOTE: signed palm mapping and solver preview use the full -1..1 range.")
        if args.record_session:
            print(f"Recording raw hand session to: {args.record_session}")
        if args.replay_session:
            print(f"Replaying hand session from: {args.replay_session}")
        set_stream_phase(stream, "teleop")
        last_valid_frame_time: Optional[float] = None
        tracking_lost = False

        while True:
            loop_start = time.monotonic()
            frame = stream.get_latest_frame()
            if frame is not None:
                if tracking_lost:
                    print("Tracking recovered; commands will ramp from the held state.")
                    tracking_lost = False
                last_valid_frame_time = loop_start
                if recorder is not None:
                    recorder.record(frame, "teleop")
                raw_result = mapper.step(frame.points)
                output_result = (
                    command_filter.apply(raw_result, frame.timestamp)
                    if command_filter is not None
                    else raw_result
                )
                palm_preview = select_palm_control_preview(
                    output_result,
                    palm_solver,
                    palm_solution_selector,
                    palm_fallback,
                    input_limiter=palm_input_limiter,
                    timestamp=frame.timestamp,
                )
                if debug_logger is not None:
                    debug_logger.write(frame.timestamp, palm_preview)
                if args.print_every_frame or loop_start >= next_print:
                    print_mapping_line(raw_result, "raw", include_features=True)
                    if command_filter is not None and args.print_filtered:
                        print_mapping_line(
                            output_result,
                            "filtered",
                            include_features=False,
                        )
                    print_palm_control_preview(palm_preview)
                    next_print = loop_start + 0.2
                if hardware_sender is not None:
                    # Hardware mode remains blocked above.  Do not route fallback
                    # preview values to physical motors before trajectory validation.
                    if not hardware_sender.send(output_result, palm_preview[0][3]):

                        print("WARNING: hardware command failed")
                else:
                    send_to_hardware_placeholder(output_result)
            elif (
                bool(getattr(stream, "is_replay", False))
                and stream.phase_finished
                and not args.replay_loop
            ):
                print("Replay teleoperation phase completed.")
                break
            elif (
                last_valid_frame_time is not None
                and not tracking_lost
                and loop_start - last_valid_frame_time >= args.tracking_timeout
                and not bool(getattr(stream, "is_replay", False))
            ):
                tracking_lost = True
                palm_fallback.pause()
                print("Tracking lost; holding the last valid command.")

            sleep_time = playback_period - (time.monotonic() - loop_start)
            if sleep_time > 0.0 and not bool(getattr(stream, "no_wait", False)):
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("KeyboardInterrupt: stopping MH6 teleop mapping runner")
    except RuntimeError as exc:
        print(f"ERROR: {exc}")
        return 2
    finally:
        if debug_logger is not None:
            debug_logger.close()
        if recorder is not None:
            saved_path = recorder.save()
            if saved_path is not None:
                print(f"Saved hand session: {saved_path}")
        if hardware_sender is not None:
            hardware_sender.stop()
        stream.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
