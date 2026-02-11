from kinematics.robotModel import RobotModel 
from kinematics.matlabBackend import MatlabBackend, MATLAB_AVAILABLE
import math
import numpy as np
import pytest
import logging

logger = logging.getLogger(__name__)


@pytest.mark.skipif(not MATLAB_AVAILABLE, reason="MATLAB not installed")
def test_inverse_kinematics_returns_correct_joint_values():
    """
    Test that inverse kinematics returns the same joint angles that were
    used to compute the forward kinematics transformation matrix.
    
    Flow:
    1. Start with known joint angles (q_original)
    2. Compute forward kinematics to get end-effector transformation (H)
    3. Compute inverse kinematics from that transformation
    4. Verify that computed angles match original angles
    """
    # Initialize robot with DH parameters
    robot = RobotModel(
        a=[25, 315, 35, 0, 0, -296.23],
        alpha=[math.pi / 2, 0, math.pi / 2, -math.pi / 2, math.pi / 2, 0],
        d=[400, 0, 0, 365, 0, 161.44],
        theta=[0, 0, 0, 0, 0, 0],
        joint_limits=[[-2.094, 2.094]] * 6,
    )
    
    # Initialize MATLAB backend
    matlab = MatlabBackend()
    
    # Original joint angles (in radians)
    q_original = [math.pi/5, math.pi/3, -math.pi/4, math.pi/4, math.pi/3, math.pi/4]
    
    logger.info(f"Original joint angles: {q_original}")
    
    # Step 1: Compute forward kinematics
    H = matlab.forwardKinematics(q_original, robot)
    logger.info(f"Computed transformation matrix (H):\n{H}")
    
    # Step 2: Compute inverse kinematics
    q_computed = matlab.inverseKinematics(robot, H)
    logger.info(f"Computed joint angles: {q_computed}")
    
    # Step 3: Verify they match within tolerance
    np.testing.assert_allclose(
        q_original, 
        q_computed, 
        rtol=1e-5, 
        atol=1e-8,
        err_msg="Inverse kinematics did not return the same angles as input to forward kinematics"
    )
