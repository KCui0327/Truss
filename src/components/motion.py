# src/components/motion.py
from __future__ import annotations

from typing import List


def interpolate_joint_path(q_start: List[float], q_goal: List[float], steps: int) -> List[List[float]]:
    """
    Deterministic joint-space interpolation.

    Note: This is not a cubic spline (yet). It uses a smoothstep easing function
    to reduce jerk relative to pure linear interpolation.

    Replace later with cubic splines per-joint if needed.
    """
    if len(q_start) != len(q_goal):
        raise ValueError("q_start and q_goal must have same length")
    if steps < 2:
        return [q_goal[:]]

    def smoothstep(t: float) -> float:
        return t * t * (3.0 - 2.0 * t)

    path: List[List[float]] = []
    for i in range(steps):
        t = i / (steps - 1)
        s = smoothstep(t)
        q = [(1.0 - s) * a + s * b for a, b in zip(q_start, q_goal)]
        path.append(q)
    return path
