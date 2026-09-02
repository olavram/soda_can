import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

from ..model.distance_calculator import CAN_RADIUS


class FloorPlot:
    def __init__(self, positions):
        self.positions = positions

    def _draw_line(self, ax, p1, p2, label):
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]],
                linestyle="--", color="steelblue", linewidth=1)
        mx, my = (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2
        ax.annotate(label, (mx, my),
                    fontsize=7, color="steelblue",
                    ha="center", va="bottom",
                    bbox=dict(boxstyle="round,pad=0.1", fc="white", alpha=0.7, ec="none"))

    def show(self):
        fig, ax = plt.subplots(figsize=(7, 7))

        post = (0, 0)
        ax.plot(*post, marker="x", color="black", markersize=10, label="Post (P)")

        for i, (x, y) in enumerate(self.positions):
            circle = patches.Circle((x, y), CAN_RADIUS, color="tab:red", alpha=0.35)
            ax.add_patch(circle)
            ax.plot(x, y, "o", color="tab:red")
            ax.annotate(f"Can {i}\n({x:.1f}, {y:.1f})", (x, y),
                        textcoords="offset points", xytext=(8, 4), fontsize=8)

            dist_to_post = np.hypot(x, y)
            self._draw_line(ax, post, (x, y), f"{dist_to_post:.1f} cm")

        n = len(self.positions)
        for i in range(n):
            for j in range(i + 1, n):
                x1, y1 = self.positions[i]
                x2, y2 = self.positions[j]
                edge = np.hypot(x2 - x1, y2 - y1) - 2 * CAN_RADIUS
                self._draw_line(ax, (x1, y1), (x2, y2), f"{edge:.1f} cm")

        all_x = [x for x, y in self.positions] + [0]
        all_y = [y for x, y in self.positions] + [0]
        margin = max(CAN_RADIUS * 5, 10)
        ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

        ax.set_aspect("equal")
        ax.set_xlabel("x (cm)")
        ax.set_ylabel("y (cm)")
        ax.set_title("Floor plan")
        ax.legend()
        ax.grid(True, linestyle="--", alpha=0.3)

        plt.tight_layout()
        plt.show()
