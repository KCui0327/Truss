from kinematics.robotModel import RobotModel 
from kinematics.matlabBackend import MatlabBackend
import math
import numpy as np
import logging

def test_forward_kinematics_returns_correct_result():
    robot = RobotModel(
                a=[0, 119.64, 82, 22, 46],
                alpha=[math.pi/2, 0, -math.pi/2, math.pi/2, 0],
                d=[98, 24, 0, 60, 0],
                theta=[0, 0, 0, 0, 0],
                joint_limits=[[-2.094, 2.094]] * 5,
            )
    matlab = MatlabBackend()

    q_original = [0, 1.78130588892, 4.50187941826, 0, 0]

    end_effector_position = np.array([125, -24, 275])
    H = matlab.forwardKinematics(q_original, robot)
    extracted_position = H[0:3, 3]
    logging.basicConfig(level=logging.INFO)
    logging.info(f"H Matrix:\n{H}")
    np.testing.assert_allclose(end_effector_position, extracted_position, rtol=1e-5, atol=1e-8)
    
    
test_forward_kinematics_returns_correct_result()