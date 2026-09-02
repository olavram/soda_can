import numpy as np

from .parameters import RigParameters
from .parameter_enum import Param


class RigSolver:
    def __init__(self, params: RigParameters):
        self.aL = params.aL
        self.aR = params.aR
        self.hL = params.hL
        self.hR = params.hR
        self.bL = params.bL
        self.bR = params.bR
        self.hJ = params.hJ
        self.Lp = params.Lp
        self.rL = params.rL
        self.rR = params.rR
        self._params = params

    def arm_tips(self):
        s = 1.0 / np.sqrt(2)
        L = np.array([-self.aL * s, -self.aL * s, self.hL])
        R = np.array([ self.aR * s, -self.aR * s, self.hR])
        return L, R

    def joint(self):
        L, R = self.arm_tips()

        if self.hJ - self.hL >= self.bL:
            raise ValueError(f"Left brace too short: hJ-hL={self.hJ - self.hL:.2f} >= bL={self.bL}")
        if self.hJ - self.hR >= self.bR:
            raise ValueError(f"Right brace too short: hJ-hR={self.hJ - self.hR:.2f} >= bR={self.bR}")

        rL2 = self.bL**2 - (self.hJ - self.hL)**2
        rR2 = self.bR**2 - (self.hJ - self.hR)**2

        Lx, Ly = L[0], L[1]
        Rx, Ry = R[0], R[1]

        ax = 2.0 * (Rx - Lx)
        ay = 2.0 * (Ry - Ly)
        c  = rL2 - rR2 - Lx**2 - Ly**2 + Rx**2 + Ry**2

        if abs(ax) >= abs(ay):
            u = c / ax - Lx
            v = ay / ax
            A = v**2 + 1
            B = -2*u*v - 2*Ly
            C = u**2 + Ly**2 - rL2
            disc = B**2 - 4*A*C
            if disc < 0:
                raise ValueError("No real joint solution: sphere intersection is empty")
            Jy1 = (-B + np.sqrt(disc)) / (2*A)
            Jy2 = (-B - np.sqrt(disc)) / (2*A)
            Jx1 = (c - ay*Jy1) / ax
            Jx2 = (c - ay*Jy2) / ax
        else:
            u = c / ay - Ly
            v = ax / ay
            A = v**2 + 1
            B = -2*u*v - 2*Lx
            C = u**2 + Lx**2 - rL2
            disc = B**2 - 4*A*C
            if disc < 0:
                raise ValueError("No real joint solution: sphere intersection is empty")
            Jx1 = (-B + np.sqrt(disc)) / (2*A)
            Jx2 = (-B - np.sqrt(disc)) / (2*A)
            Jy1 = (c - ax*Jx1) / ay
            Jy2 = (c - ax*Jx2) / ay

        j1 = np.array([Jx1, Jy1, self.hJ])
        j2 = np.array([Jx2, Jy2, self.hJ])
        J  = j1 if np.hypot(Jx1, Jy1) <= np.hypot(Jx2, Jy2) else j2

        offset = np.hypot(J[0], J[1])
        if offset > 3.0:
            print(f"WARNING: joint {offset:.2f} cm from post axis — inputs may be inconsistent")

        return J

    def pole_tip(self):
        J    = self.joint()
        L, R = self.arm_tips()

        Jx, Jy, Jz = J
        J2 = float(np.dot(J, J))
        L2 = float(np.dot(L, L))
        R2 = float(np.dot(R, R))

        a1 = 2*(Jx - L[0]);  b1 = 2*(Jy - L[1]);  c1 = 2*(Jz - L[2]);  d1 = self.rL**2 - self.Lp**2 + J2 - L2
        a2 = 2*(Jx - R[0]);  b2 = 2*(Jy - R[1]);  c2 = 2*(Jz - R[2]);  d2 = self.rR**2 - self.Lp**2 + J2 - R2

        det = a1*b2 - a2*b1
        if abs(det) < 1e-10:
            raise ValueError("Degenerate geometry: trilateration planes are parallel")

        e1 = (d1*b2 - d2*b1) / det;  f1 = (-c1*b2 + c2*b1) / det
        e2 = (a1*d2 - a2*d1) / det;  f2 = (-a1*c2 + a2*c1) / det

        g1 = e1 - Jx
        g2 = e2 - Jy

        A    = f1**2 + f2**2 + 1
        B    = 2*(g1*f1 + g2*f2 - Jz)
        C    = g1**2 + g2**2 + Jz**2 - self.Lp**2
        disc = B**2 - 4*A*C

        if disc < 0:
            raise ValueError("No real pole tip solution: range measurements describe no realisable geometry")

        z1 = (-B + np.sqrt(disc)) / (2*A)
        z2 = (-B - np.sqrt(disc)) / (2*A)

        candidates = [
            np.array([e1 + f1*z1, e2 + f2*z1, z1]),
            np.array([e1 + f1*z2, e2 + f2*z2, z2]),
        ]
        above = [P for P in candidates if P[2] > self.hJ]

        if not above:
            raise ValueError("No pole tip above joint: beam would never reach the floor")

        return max(above, key=lambda P: P[2])

    def ray_cast(self):
        J         = self.joint()
        P         = self.pole_tip()
        direction = J - P

        if direction[2] >= 0:
            raise ValueError("Ray direction is upward — cannot reach the floor")

        t = self.hJ / (-direction[2])
        return J + t * direction

    def distances(self):
        J = self.joint()
        C = self.ray_cast()
        return {
            'D_joint': float(np.hypot(C[0] - J[0], C[1] - J[1])),
            'D_post':  float(np.hypot(C[0], C[1])),
        }

    def cross_check(self):
        P       = self.pole_tip()
        D_joint = self.distances()['D_joint']
        theta   = np.arcsin((P[2] - self.hJ) / self.Lp)
        D_check = self.hJ / np.tan(theta)

        if abs(D_check - D_joint) > 0.1:
            print(f"WARNING: cross-check mismatch {abs(D_check - D_joint):.3f} cm")

        return D_check

    def error_propagation(self, tol: float = 0.5):
        partials = {}
        for param in Param:
            hi = RigSolver(self._params.with_perturbation(param,  tol))
            lo = RigSolver(self._params.with_perturbation(param, -tol))
            partials[param] = (hi.distances()['D_joint'] - lo.distances()['D_joint']) / (2 * tol)
        return partials

    def check_warnings(self):
        warnings = []

        if self.hJ - self.hL >= self.bL:
            warnings.append(f"Left brace cannot reach joint: hJ-hL={self.hJ - self.hL:.2f} >= bL={self.bL}")
        if self.hJ - self.hR >= self.bR:
            warnings.append(f"Right brace cannot reach joint: hJ-hR={self.hJ - self.hR:.2f} >= bR={self.bR}")

        try:
            J         = self.joint()
            offset    = np.hypot(J[0], J[1])
            P         = self.pole_tip()
            L, R      = self.arm_tips()
            threshold = 0.1 * self.Lp

            if offset > 3.0:
                warnings.append(f"Joint {offset:.2f} cm from post axis — inputs may be inconsistent")
            if np.hypot(P[0] - L[0], P[1] - L[1]) < threshold:
                warnings.append(f"Pole tip too close to left arm tip (threshold {threshold:.1f} cm) — rL sensitivity degraded")
            if np.hypot(P[0] - R[0], P[1] - R[1]) < threshold:
                warnings.append(f"Pole tip too close to right arm tip (threshold {threshold:.1f} cm) — rR sensitivity degraded")

        except ValueError as e:
            warnings.append(f"Geometry error: {e}")

        return warnings

    def solve(self):
        L, R     = self.arm_tips()
        J        = self.joint()
        P        = self.pole_tip()
        C        = self.ray_cast()
        D_check  = self.cross_check()
        partials = self.error_propagation()
        warnings = self.check_warnings()

        return {
            'L':        L,
            'R':        R,
            'J':        J,
            'P':        P,
            'C':        C,
            'x':        float(C[0]),
            'y':        float(C[1]),
            'D_check':  D_check,
            'partials': partials,
            'warnings': warnings,
        }
