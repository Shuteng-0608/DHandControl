"""Short operator prompts, played synchronously before calibration timers start."""

from pathlib import Path
import shutil
import subprocess
import wave

VOICE_DIRECTORY = Path(__file__).resolve().parents[1] / "assets" / "calibration_voice"
CUES = ("neutral", "grasp", "thumb_rotate", "opp_index", "opp_middle", "opp_ring", "opp_little", "complete")
OPTIONAL_CUES = ("thumb_abduct",)
TEXT = {
    "neutral": "请保持右手自然张开（自然放松，不要过度伸直）。",
    "grasp": "请缓慢握拳。",
    "thumb_rotate": "请将拇指向掌心内侧旋转。",
    "thumb_abduct": "请将拇指抬离掌面，到位后保持。",
    "opp_index": "请用拇指与食指对指。",
    "opp_middle": "请用拇指与中指对指。",
    "opp_ring": "请用拇指与无名指对指。",
    "opp_little": "请用拇指与小指对指。",
    "complete": "标定完成。",
}


def add_voice_arguments(parser):
    parser.add_argument("--no-calibration-voice", action="store_true",
                        help="Use text prompts only; replay always stays silent.")
    parser.add_argument("--calibration-voice-dir", default=str(VOICE_DIRECTORY),
                        help="Directory containing the eight WAV prompts; optional thumb_abduct.wav for M4.")


class CalibrationVoice:
    def __init__(self, directory=VOICE_DIRECTORY, *, enabled=True):
        self.directory = Path(directory).expanduser()
        self.enabled = enabled
        self._missing_optional_notified = set()
        self.player = next((path for name in ("paplay", "aplay", "afplay")
                            if (path := shutil.which(name))), None) if enabled else None
        if enabled:
            # Fail before connecting to AVP for missing or malformed assets.
            for cue in CUES + tuple(c for c in OPTIONAL_CUES if (self.directory / f"{c}.wav").exists()):
                try:
                    with wave.open(str(self.directory / f"{cue}.wav"), "rb") as source:
                        if source.getnframes() == 0 or source.getcomptype() != "NONE":
                            raise ValueError(f"invalid calibration voice: {cue}")
                except (wave.Error, EOFError) as exc:
                    raise ValueError(f"invalid calibration voice: {cue}") from exc
            if self.player is None:
                print("WARNING: no paplay/aplay/afplay found; using text calibration prompts.", flush=True)

    def play(self, cue):
        print(TEXT[cue], flush=True)
        if not self.enabled or self.player is None:
            return
        if cue in OPTIONAL_CUES and not (self.directory / f"{cue}.wav").exists():
            if cue not in self._missing_optional_notified:
                print("[M4] 暂无拇指偏转录音，请按上述文字操作；其他阶段继续语音提示。", flush=True)
                self._missing_optional_notified.add(cue)
            return
        try:
            # run() waits for playback and terminates its child on Ctrl-C.
            subprocess.run([self.player, str(self.directory / f"{cue}.wav")],
                           check=True, stdout=subprocess.DEVNULL,
                           stderr=subprocess.PIPE, timeout=30)
        except (OSError, subprocess.SubprocessError) as exc:
            print(f"WARNING: voice playback failed ({type(exc).__name__}); using text prompts.", flush=True)
            self.player = None
