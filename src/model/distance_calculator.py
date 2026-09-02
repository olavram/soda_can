import numpy as np

CAN_RADIUS = 2.65  # cm


class DistanceCalculator:
    def __init__(self, *measurements):
        if not 1 <= len(measurements) <= 3:
            raise ValueError("Pass 1 to 3 (x, y) measurements")
        self.cans = list(measurements)


    def _post_to_can(self, i):
        x, y = self.cans[i]
        return float(np.hypot(x, y))


    def _can_to_can(self, i, j):
        x1, y1 = self.cans[i]
        x2, y2 = self.cans[j]
        return float(np.hypot(x2 - x1, y2 - y1) - 2 * CAN_RADIUS)


    def all_distances(self):
        result = {}
        n = len(self.cans)

        for i in range(n):
            result[f"post -> can {i}"] = self._post_to_can(i)

        for i in range(n):
            for j in range(i + 1, n):
                result[f"can {i} -> can {j}"] = self._can_to_can(i, j)

        return result