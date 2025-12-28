import matlab.engine
import numpy as np
import matlab.engine
import matlab
import os

class MatlabBackend:
    def __init__(self):
        project_dir = os.path.dirname(__file__)
        self.eng = matlab.engine.start_matlab()
        self.eng.addpath(project_dir, nargout=0) # tell the engine to read matlab files in this directory
        
    # Run forward_kinematics.m in MATLAB using RobotModel DH parameters
    def forwardKinematics(self, q, robot):
        a, alpha, d, theta_offset = robot.get_dh_params()

        q_mat = matlab.double(q.tolist())

        H_mat = self.eng.forward_kuka(
            q_mat,
            matlab.double(a.tolist()),
            matlab.double(alpha.tolist()),
            matlab.double(d.tolist()),
            matlab.double(theta_offset.tolist()),
            nargout=1
        )

        return np.array(H_mat, dtype=float)
    
    # Run inverse_kinematics.m in MATLAB using RobotModel DH parameters
    def inverseKinematics(self, robotModel, H):
        a, alpha, d, theta_offset = robotModel.dh_table
        H_mat = matlab.double(H.tolist())
        q_mat = self.eng.inverse_kuka(
            H_mat,
            matlab.double(a.tolist()),
            matlab.double(alpha.tolist()),
            matlab.double(d.tolist()),
            matlab.double(theta_offset.tolist()),
            nargout=1
        )

        return np.array(q_mat).flatten()
    
        
        

        
        