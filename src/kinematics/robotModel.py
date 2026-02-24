import numpy as np
from dataclasses import dataclass
from logutils import init, get_logger

init()
logger = get_logger("RobotModel")

@dataclass
class RobotModel:
    dh_table: np.ndarray        # shape (n, 4): [a, alpha, d, theta_offset]
    joint_limits: np.ndarray    # shape (n, 2): [min, max]


    #  a=[25, 315, 35, 0, 0, -296.23]
    #  alpha=[math.pi/2, 0, math.pi/2, -math.pi/2, math.pi/2, 0]
    #  d=[400, 0, 0, 365, 0, 161.44]
    #  theta=[0, 0, 0, 0, 0, 0]
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
        self.dh_table = np.column_stack((a, alpha, d, theta)).astype(float, copy=True)
        self.joint_limits = np.asarray(joint_limits)

        logger.info("RobotModel initialized with %d joints", self.n)
        logger.debug("DH table shape: %s", self.dh_table.shape)
        try:
            if self.joint_limits.shape[0] != self.n:
                logger.warning(
                    "Joint limits rows (%s) do not match number of joints (%s)",
                    self.joint_limits.shape[0],
                    self.n,
                )
        except Exception:
            logger.exception("Exception while validating joint limits shape")

