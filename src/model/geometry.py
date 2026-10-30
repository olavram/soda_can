import numpy as np

from .parameters import RigParameters
from .parameter_enum import Param


class RigSolver:
    def __init__(self, params: RigParameters):
        self.reach_left        = params.reach_left
        self.reach_right       = params.reach_right
        self.arm_height_left   = params.arm_height_left
        self.arm_height_right  = params.arm_height_right
        self.joint_height      = params.joint_height
        self.pole_length       = params.pole_length
        self.laser_range_left  = params.laser_range_left
        self.laser_range_right = params.laser_range_right
        self._params = params

    def arm_tips(self):
        s = 1.0 / np.sqrt(2)
        L = np.array([-self.reach_left * s,  -self.reach_left * s,  self.arm_height_left])
        R = np.array([ self.reach_right * s, -self.reach_right * s, self.arm_height_right])
        return L, R

    def joint(self):
        return np.array([0.0, 0.0, self.joint_height])

    def pole_tip(self):
        joint               = self.joint()
        left_tip, right_tip = self.arm_tips()

        joint_x, joint_y, joint_z = joint

        joint_sq     = float(np.dot(joint, joint))
        left_tip_sq  = float(np.dot(left_tip, left_tip))
        right_tip_sq = float(np.dot(right_tip, right_tip))

        left_nx  = 2*(joint_x - left_tip[0])
        left_ny  = 2*(joint_y - left_tip[1])
        left_nz  = 2*(joint_z - left_tip[2])
        left_rhs = self.laser_range_left**2 - self.pole_length**2 + joint_sq - left_tip_sq

        right_nx  = 2*(joint_x - right_tip[0])
        right_ny  = 2*(joint_y - right_tip[1])
        right_nz  = 2*(joint_z - right_tip[2])
        right_rhs = self.laser_range_right**2 - self.pole_length**2 + joint_sq - right_tip_sq

        det = left_nx*right_ny - right_nx*left_ny
        if abs(det) < 1e-10:
            raise ValueError("Degenerate geometry: trilateration planes are parallel")

        line_x0    = (left_rhs*right_ny - right_rhs*left_ny) / det
        line_dx_dz = (-left_nz*right_ny + right_nz*left_ny) / det
        line_y0    = (left_nx*right_rhs - right_nx*left_rhs) / det
        line_dy_dz = (-left_nx*right_nz + right_nx*left_nz) / det

        offset_x = line_x0 - joint_x
        offset_y = line_y0 - joint_y

        quad_a       = line_dx_dz**2 + line_dy_dz**2 + 1
        quad_b       = 2*(offset_x*line_dx_dz + offset_y*line_dy_dz - joint_z)
        quad_c       = offset_x**2 + offset_y**2 + joint_z**2 - self.pole_length**2
        discriminant = quad_b**2 - 4*quad_a*quad_c

        if discriminant < 0:
            raise ValueError("No real pole tip solution: range measurements describe no realisable geometry")

        z_plus  = (-quad_b + np.sqrt(discriminant)) / (2*quad_a)
        z_minus = (-quad_b - np.sqrt(discriminant)) / (2*quad_a)

        candidates = [
            np.array([line_x0 + line_dx_dz*z_plus,  line_y0 + line_dy_dz*z_plus,  z_plus]),
            np.array([line_x0 + line_dx_dz*z_minus, line_y0 + line_dy_dz*z_minus, z_minus]),
        ]

        above = [P for P in candidates if P[2] > self.joint_height]

        if not above:
            raise ValueError("No pole tip above joint: beam would never reach the floor")

        return above[0]

    def ray_cast(self):
        J         = self.joint()
        P         = self.pole_tip()
        P         = P + np.array([0.0, 0.0, 1.3])
        direction = J - P

        if direction[2] >= 0:
            raise ValueError("Ray direction is upward — cannot reach the floor")

        t = self.joint_height / (-direction[2])

        return J + t * direction

    def distances(self):
        C = self.ray_cast()
        return {
            'D_joint': float(np.hypot(C[0], C[1])),
            'D_post':  float(np.hypot(C[0], C[1])),
        }

    def cross_check(self):
        P       = self.pole_tip()
        D_joint = self.distances()['D_joint']
        theta   = np.arcsin((P[2] - self.joint_height) / self.pole_length)
        D_check = self.joint_height / np.tan(theta)

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

        try:
            P         = self.pole_tip()
            L, R      = self.arm_tips()
            threshold = 0.1 * self.pole_length

            if np.hypot(P[0] - L[0], P[1] - L[1]) < threshold:
                warnings.append(f"Pole tip too close to left arm tip (threshold {threshold:.1f} cm) — laser_range_left sensitivity degraded")
            if np.hypot(P[0] - R[0], P[1] - R[1]) < threshold:
                warnings.append(f"Pole tip too close to right arm tip (threshold {threshold:.1f} cm) — laser_range_right sensitivity degraded")

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