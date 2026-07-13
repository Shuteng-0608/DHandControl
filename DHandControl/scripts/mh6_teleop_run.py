#!/usr/bin/env python3
"""
Plain Vision Pro to MH6 mapping runner.

Flow:
VisionProHandStream -> neutral/range calibration -> MH6HandMapper -> printed intent.

Hardware output is disabled by default and requires --enable-hardware.
"""

from __future__ import annotations

import argparse
import copy
import math
import time
from numbers import Real
from typing import Dict, List, Optional, Sequence

import numpy as np

from mh6_mapping import MH6HandMapper
from mh6_palm_solver import MH6PalmSolver
from mh6_palm_solution_selector import PalmSolutionSelector
from visionpro_stream import VisionProHandStream


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Vision Pro to MH6 mapping runner")
    parser.add_argument("--avp-ip", default="192.168.8.145", help="IP address of the Apple Vision Pro device")
    parser.add_argument("--hand", choices=("left", "right"), default="right")
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
        "--print-filtered",
        action="store_true",
        help="Print filtered command values instead of raw mapping values.",
    )
    return parser.parse_args(argv)


def send_to_hardware_placeholder(result: Dict[str, Dict[str, float]]) -> None:
    """Intentionally no-op when hardware output is disabled."""
    _ = result


class HardwareSender:
    """Persistent normalized-command output to the MH6 hardware driver."""

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

    def send(self, result: Dict[str, Dict[str, float]]) -> bool:
        if self.hand is None:
            return False
        print(result)
        return self.hand.move_hand_normalized(
            finger_values=  [
                            result["low_dim"]["u_thumb"],
                            result["low_dim"]["u_index"],
                            result["low_dim"]["u_middle"],
                            result["low_dim"]["u_ring"],
                            result["low_dim"]["u_little"],],
            palm_values=[
                        result["low_dim"]["u_h"], 
                        result["low_dim"]["u_h"], 
                        result["low_dim"]["u_h"]], 
            palm_times=[50, 50, 50],
            wait_status=False,
        )


class CommandLowPassFilter:
    """Time-aware first-order low-pass filter for normalized command sections."""

    def __init__(self, tau: float) -> None:
        self.tau = float(tau)
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

        if self.previous_timestamp is None:
            for section_name in ("low_dim", "palm_command"):
                for key, value in result.get(section_name, {}).items():
                    if isinstance(value, Real) and not isinstance(value, bool):
                        self.previous[(section_name, key)] = float(value)
            self.previous_timestamp = now
            return filtered_result

        dt = now - self.previous_timestamp
        alpha = 0.0 if dt <= 0.0 else 1.0 - math.exp(-dt / self.tau)
        alpha = min(max(alpha, 0.0), 1.0)

        for section_name in ("low_dim", "palm_command"):
            raw_section = result.get(section_name, {})
            filtered_section = filtered_result.get(section_name, {})
            for key, value in raw_section.items():
                if not isinstance(value, Real) or isinstance(value, bool):
                    continue

                state_key = (section_name, key)
                current = float(value)
                previous = self.previous.get(state_key, current)
                filtered = previous + alpha * (current - previous)
                filtered_section[key] = filtered
                self.previous[state_key] = filtered

        if dt > 0.0:
            self.previous_timestamp = now
        if "palm_command" in filtered_result:
            filtered_result["palm"] = copy.deepcopy(filtered_result["palm_command"])
            filtered_result["palm_fold"] = copy.deepcopy(
                filtered_result["palm_command"]
            )
        return filtered_result


def collect_hand_samples(
    stream: VisionProHandStream,
    duration: float,
    rate_hz: float,
) -> List[np.ndarray]:
    period = 1.0 / rate_hz
    deadline = time.monotonic() + duration
    samples: List[np.ndarray] = []

    while time.monotonic() < deadline:
        loop_start = time.monotonic()
        frame = stream.get_latest_frame()
        if frame is not None:
            samples.append(frame.points)

        sleep_time = period - (time.monotonic() - loop_start)
        if sleep_time > 0.0:
            time.sleep(sleep_time)

    return samples


def collect_open_hand_samples(
    stream: VisionProHandStream,
    duration: float,
    rate_hz: float,
) -> List[np.ndarray]:
    """Compatibility alias for callers using the previous function name."""

    return collect_hand_samples(stream, duration, rate_hz)


def print_mapping_line(result: Dict[str, Dict[str, float]]) -> None:
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
    print(
        f"signedBending: {fingers} | opposition signed/intent: {opposition_text} | "
        f"grasp: {grasp_text} | palmFold: {palm_text}"
    )


def solve_palm_motor_preview(
    result: Dict[str, Dict[str, float]],
    solver: MH6PalmSolver,
):
    """Solve palm motor candidates from filtered named palm commands.

    The three values use the solver's full signed -1..1 convention.
    """

    palm_command = result["palm_command"]
    normalized_inputs = {
        "palm_flexion": float(palm_command["vertical"]),
        "palm_cross": float(palm_command["lateral"]),
        "thumb_inward": float(palm_command["thumb_rotation_command"]),
    }
    motor_solutions = solver.solve_motor_from_normalized(
        normalized_inputs["palm_flexion"],
        normalized_inputs["palm_cross"],
        normalized_inputs["thumb_inward"],
    )
    return normalized_inputs, motor_solutions


def select_palm_motor_preview(
    result: Dict[str, Dict[str, float]],
    solver: MH6PalmSolver,
    selector: PalmSolutionSelector,
    timestamp: Optional[float] = None,
):
    """Solve all branches and select the safest continuous preview target."""

    normalized_inputs, motor_solutions = solve_palm_motor_preview(result, solver)
    selection = selector.select(
        tuple(normalized_inputs.values()),
        motor_solutions,
        timestamp=timestamp,
    )
    return normalized_inputs, motor_solutions, selection


def print_palm_motor_preview(
    preview,
) -> None:
    normalized_inputs, motor_solutions, selection = preview
    input_text = " ".join(
        f"{name}={value:.3f}" for name, value in normalized_inputs.items()
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
        print(f"palm solver preview: {input_text} -> {candidate_text} | {selected_text}")
    else:
        print(f"palm solver preview: {input_text} -> {selected_text}")


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
    if args.enable_hardware and not args.port:
        print("ERROR: --port is required with --enable-hardware")
        return 2
    if args.enable_hardware:
        print(
            "ERROR: signed mapping is currently print-test only; hardware output "
            "is intentionally blocked until signed motor commands are validated"
        )
        return 2

    stream = VisionProHandStream(
        avp_ip=args.avp_ip,
        hand=args.hand,
        origin=args.origin,
    )
    mapper = MH6HandMapper()
    palm_solver = MH6PalmSolver()
    palm_solution_selector = PalmSolutionSelector()
    period = 1.0 / args.rate
    next_print = 0.0
    command_filter = CommandLowPassFilter(args.filter_tau) if args.filter_tau > 0.0 else None
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

        while True:
            loop_start = time.monotonic()
            frame = stream.get_latest_frame()
            if frame is not None:
                raw_result = mapper.step(frame.points)
                output_result = (
                    command_filter.apply(raw_result, loop_start)
                    if command_filter is not None
                    else raw_result
                )
                palm_preview = select_palm_motor_preview(
                    output_result,
                    palm_solver,
                    palm_solution_selector,
                    timestamp=loop_start,
                )
                if loop_start >= next_print:
                    print_mapping_line(output_result if args.print_filtered else raw_result)
                    print_palm_motor_preview(palm_preview)
                    next_print = loop_start + 0.2
                if hardware_sender is not None:
                    if not hardware_sender.send(output_result):

                        print("WARNING: hardware command failed")
                else:
                    send_to_hardware_placeholder(output_result)

            sleep_time = period - (time.monotonic() - loop_start)
            if sleep_time > 0.0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("KeyboardInterrupt: stopping MH6 teleop mapping runner")
    except RuntimeError as exc:
        print(f"ERROR: {exc}")
        return 2
    finally:
        if hardware_sender is not None:
            hardware_sender.stop()
        stream.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
