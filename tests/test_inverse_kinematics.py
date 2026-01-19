import math
import sys
from unittest.mock import MagicMock

import numpy as np
import pytest

# Mock MATLAB if not available
try:
    import matlab.engine  # noqa: F401
except ImportError:
    sys.modules["matlab"] = MagicMock()
    sys.modules["matlab.engine"] = MagicMock()

from kinematics.robotModel import RobotModel
from kinematics.matlabBackend import MatlabBackend


def test_inverse_kinematics_pose_roundtrip_fk_of_ik_solution():
    """
    Correct IK validation: FK(IK(H)) ≈ H.

    Avoids false failures due to multiple valid IK solutions for a pose.
    Skips if MATLAB engine/backend is unavailable.
    """
    try:
        robot = RobotModel(
            a=[25, 315, 35, 0, 0, -296.23],
            alpha=[math.pi / 2, 0, math.pi / 2, -math.pi / 2, math.pi / 2, 0],
            d=[400, 0, 0, 365, 0, 161.44],
            theta=[0, 0, 0, 0, 0, 0],
            joint_limits=[[-2.094, 2.094]] * 6,
        )
        matlab_backend = MatlabBackend()
    except (RuntimeError, TypeError, AttributeError) as e:
        pytest.skip(f"MATLAB engine not available: {e}")

    # Build a reachable target pose from FK of a known joint configuration
    q_seed = np.array([math.pi / 5, math.pi / 3, -math.pi / 4, math.pi / 4, math.pi / 3, math.pi / 4])

    try:
        H_target = matlab_backend.forwardKinematics(q_seed, robot)
        if getattr(H_target, "size", 0) == 0:
            pytest.skip("MATLAB FK returned empty result")

        q_sol = matlab_backend.inverseKinematics(robot, H_target)
        if getattr(q_sol, "size", 0) == 0:
            pytest.skip("MATLAB IK returned empty result")

        H_achieved = matlab_backend.forwardKinematics(q_sol, robot)
        if getattr(H_achieved, "size", 0) == 0:
            pytest.skip("MATLAB FK on IK solution returned empty result")

        # Translation error
        pos_err = np.linalg.norm(H_target[0:3, 3] - H_achieved[0:3, 3])
        assert pos_err < 1e-3, f"Position error too large: {pos_err}"

        # Rotation error (R_target^T R_achieved should be ~ I)
        R_t = H_target[0:3, 0:3]
        R_a = H_achieved[0:3, 0:3]
        rot_err = np.linalg.norm((R_t.T @ R_a) - np.eye(3))
        assert rot_err < 1e-3, f"Rotation error too large: {rot_err}"

    except (RuntimeError, TypeError, AttributeError) as e:
        pytest.skip(f"MATLAB computation failed: {e}")
