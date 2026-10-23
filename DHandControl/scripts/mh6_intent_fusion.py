"""Experimental thumb participation, separate from human/motor calibration."""
from dataclasses import asdict, dataclass
import json
import math
from numbers import Real
from pathlib import Path

DEFAULT_INTENT_FUSION_PATH = Path(__file__).resolve().parents[1]/"config/mh6_thumb_participation.json"


def unit(value):
    if isinstance(value,bool) or not isinstance(value,Real) or not math.isfinite(value):
        raise ValueError("thumb participation requires finite normalized inputs")
    return min(max(float(value),0.),1.)


@dataclass(frozen=True)
class ThumbParticipationFusion:
    thumb_rotation_weight: float = .5
    thumb_abduction_weight: float = .5
    vertical_thumb_gain: float = .2
    lateral_thumb_gain: float = .2

    def __post_init__(self):
        for name,value in asdict(self).items():
            if isinstance(value,bool) or not isinstance(value,Real) or not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        weight_sum=self.thumb_rotation_weight+self.thumb_abduction_weight
        if not math.isfinite(weight_sum) or weight_sum <= 0:
            raise ValueError("thumb participation weights must have a positive sum")
        if self.vertical_thumb_gain > 1 or self.lateral_thumb_gain > 1:
            raise ValueError("thumb palm gains must lie in [0,1]")

    @classmethod
    def load(cls,path=DEFAULT_INTENT_FUSION_PATH):
        data=json.loads(Path(path).expanduser().read_text())
        if not isinstance(data,dict) or type(data.get("schema_version")) is not int or data["schema_version"] != 1:
            raise ValueError("expected thumb-participation schema_version 1")
        if set(data) != {"schema_version",*cls.__dataclass_fields__}:
            raise ValueError("unexpected thumb-participation configuration fields")
        return cls(**{key:value for key,value in data.items() if key != "schema_version"})

    def to_dict(self):
        return {"schema_version":1,**asdict(self)}

    def participation(self,rotation,abduction):
        if abduction is None:
            raise ValueError("thumb participation requires saved M4 calibration; use a complete staged_v4 Mapping or --intent-fusion legacy")
        weight_sum=self.thumb_rotation_weight+self.thumb_abduction_weight
        return (self.thumb_rotation_weight*unit(rotation)+self.thumb_abduction_weight*unit(abduction))/weight_sum

    def commands(self,intent,rotation,calibration):
        power=unit(intent["power_grasp"])
        opposition=float(intent["opposition_cross"])
        if not math.isfinite(opposition) or opposition < 0:
            raise ValueError("opposition contribution must be finite and nonnegative")
        thumb=unit(intent["thumb_participation"])
        h_power=calibration.vertical_power_gain*power
        h_thumb=self.vertical_thumb_gain*thumb
        v_power=calibration.lateral_power_gain*power
        v_opposition=calibration.lateral_opposition_gain*opposition
        v_thumb=self.lateral_thumb_gain*thumb
        h_pre=h_power+h_thumb; v_pre=v_power+v_opposition+v_thumb
        h,v,r=unit(h_pre),unit(v_pre),unit(rotation)
        return {"vertical":h,"lateral":v,"thumb_rotation_measured":r,
                "thumb_rotation_compensation":0.,"thumb_rotation_command":r,
                "common":h,"lateral_unit":v,
                "thumb_participation":thumb,
                "vertical_from_power":h_power,"vertical_from_thumb":h_thumb,
                "lateral_from_power":v_power,"lateral_from_opposition":v_opposition,"lateral_from_thumb":v_thumb,
                "vertical_before_clip":h_pre,"lateral_before_clip":v_pre}
