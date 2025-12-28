import numpy as np
from dataclasses import dataclass

@dataclass
class RobotModel:
    dh_table: np.ndarray        # shape (n, 4): [a, alpha, d, theta_offset]
    joint_limits: np.ndarray    # shape (n, 2): [min, max]

    def __init__(
        self,
        *,
        a,
        alpha,
        d,
        theta,
        joint_limits,
    ):
        self.n = len(a)
        self.dh_table = np.column_stack((a, alpha, d, theta))
        self.joint_limits = np.asarray(joint_limits)

