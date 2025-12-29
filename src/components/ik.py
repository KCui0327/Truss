# src/components/ik.py
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass(frozen=True)
class IKResult:
    success: bool
    q_deg: List[float]
    reason: str = ""


class IKSolver:
    """
    Placeholder IK solver.

    Next step for real arm integration:
      - define DH parameters for your 5-DOF arm
      - implement decoupling-based IK for position + wrist orientation
      - enforce joint limits
    """

    def __init__(self, joint_limits_deg: Optional[List[Tuple[float, float]]] = None) -> None:
        self.joint_limits_deg = joint_limits_deg

    def solve_to_pose(self, x: float, y: float, z: float, roll: float, pitch: float) -> IKResult:
        # TODO: Replace with actual IK.
        # For now, return a stable "reasonable" configuration.
        q = [90.0, 90.0, 90.0, 90.0, 90.0]
        if self.joint_limits_deg:
            for i, (lo, hi) in enumerate(self.joint_limits_deg):
                if not (lo <= q[i] <= hi):
                    return IKResult(False, q, reason=f"Joint {i} out of limits")
        return IKResult(True, q, reason="stub")
