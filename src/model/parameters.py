from dataclasses import dataclass
from .parameter_enum import Param


@dataclass
class RigParameters:
    aL: float
    aR: float
    hL: float
    hR: float
    bL: float
    bR: float
    hJ: float
    Lp: float
    rL: float
    rR: float

    def get(self, param: Param) -> float:
        return getattr(self, param.value)

    def with_perturbation(self, param: Param, delta: float) -> "RigParameters":
        values = {p.value: self.get(p) for p in Param}
        values[param.value] += delta
        return RigParameters(**values)
