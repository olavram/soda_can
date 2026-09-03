from dataclasses import dataclass
from .parameter_enum import Param


@dataclass
class RigParameters:
    aL: float
    aR: float
    hL: float
    hR: float
    hJ: float
    Lp: float
    rL: float
    rR: float

    @classmethod
    def from_rig(cls, rig: dict, rL: float, rR: float) -> "RigParameters":
        return cls(
            aL=rig["reach_left"],      aR=rig["reach_right"],
            hL=rig["arm_height_left"], hR=rig["arm_height_right"],
            hJ=rig["joint_height"],    Lp=rig["pole_length"],
            rL=rL, rR=rR,
        )

    def get(self, param: Param) -> float:
        return getattr(self, param.value)

    def with_perturbation(self, param: Param, delta: float) -> "RigParameters":
        values = {p.value: self.get(p) for p in Param}
        values[param.value] += delta
        return RigParameters(**values)
