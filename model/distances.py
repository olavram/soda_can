import numpy as np


def distance_from_joint(joint, strike):
    return float(np.hypot(strike[0] - joint[0], strike[1] - joint[1]))


def distance_from_post(strike):
    return float(np.hypot(strike[0], strike[1]))
