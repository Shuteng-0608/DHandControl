"""Build fixed human mapping parameters from a separate hand calibration session."""

from mh6_hand_session import ReplayHandStream
from mh6_mapping import MappingCalibration, MH6HandMapper


def calibration_from_session(path: str) -> MappingCalibration:
    """Consume neutral/range samples offline, using the same live algorithms."""
    stream = ReplayHandStream(path, no_wait=True, hand="right")
    mapper = MH6HandMapper()
    try:
        stream.start()
        if stream.metadata.get("calibration_protocol") == "staged_v1":
            from mh6_guided_calibration import calibration_from_staged_stream
            return calibration_from_staged_stream(stream)
        for phase in ("neutral", "range"):
            stream.set_phase(phase)
            samples = []
            while not stream.phase_finished:
                frame = stream.get_latest_frame()
                if frame is not None:
                    samples.append(frame.points)
            if phase == "neutral":
                mapper.calibrate_neutral(samples)
            else:
                mapper.calibrate_motion_range(samples)
        mapper.calibration.validate()
        return mapper.calibration
    finally:
        stream.stop()
