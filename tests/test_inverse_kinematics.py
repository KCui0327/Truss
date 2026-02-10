from kinematics.robotModel import RobotModel 
from kinematics.matlabBackend import MatlabBackend, MATLAB_AVAILABLE
import math
import sys
from unittest.mock import MagicMock

import numpy as np
import logging
import pytest

@pytest.mark.skipif(not MATLAB_AVAILABLE, reason="MATLAB not installed")
def test_inverse_kinematics_returns_correct_joint_values():
    robot = RobotModel(
            a=[25, 315, 35, 0, 0, -296.23],
            alpha=[math.pi / 2, 0, math.pi / 2, -math.pi / 2, math.pi / 2, 0],
            d=[400, 0, 0, 365, 0, 161.44],
            theta=[0, 0, 0, 0, 0, 0],
            joint_limits=[[-2.094, 2.094]] * 6,
        )
    matlab = MatlabBackend()
    
    q_original = [math.pi/5, math.pi/3, -math.pi/4, math.pi/4, math.pi/3, math.pi/4]

    H = matlab.forwardKinematics(q_original, robot)
    q_computed = matlab.inverseKinematics(robot, H) 
    
    logging.basicConfig(level=logging.INFO)
    logging.info(f"q_original: {q_original}")
    logging.info(f"q_computed: {q_computed}")
    np.testing.assert_allclose(q_original, q_computed, rtol=1e-5, atol=1e-8)
