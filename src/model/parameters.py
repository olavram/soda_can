from dataclasses import dataclass
from .parameter_enum import Param


@dataclass
class RigParameters:
    reach_left: float
    reach_right: float
    arm_height_left: float
    arm_height_right: float
    joint_height: float
    pole_length: float
    laser_range_left: float
    laser_range_right: float

    @classmethod
    def from_rig(cls, rig: dict, laser_range_left: float, laser_range_right: float) -> "RigParameters":
        return cls(
            reach_left=rig["reach_left"],           reach_right=rig["reach_right"],
            arm_height_left=rig["arm_height_left"], arm_height_right=rig["arm_height_right"],
            joint_height=rig["joint_height"],       pole_length=rig["pole_length"],
            laser_range_left=laser_range_left,      laser_range_right=laser_range_right,
        )

    def get(self, param: Param) -> float:
        return getattr(self, param.value)

    def with_perturbation(self, param: Param, delta: float) -> "RigParameters":
        values = {p.value: self.get(p) for p in Param}
        values[param.value] += delta
        return RigParameters(**values)
