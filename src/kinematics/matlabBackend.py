import numpy as np
import matlab.engine
import os

class MatlabBackend:
    def __init__(self):
        project_dir = os.path.dirname(__file__)
        self.eng = matlab.engine.start_matlab()
        self.eng.addpath(project_dir, nargout=0) # tell the engine to read matlab files in this directory
    
    # convert into a MATLAB suitable column vector 
    def matlab_col(self, x):
        return matlab.double([[v] for v in x])
    
    # Run forward_kinematics.m in MATLAB using RobotModel DH parameters
    def forwardKinematics(self, q, robotModel):
        a, alpha, d, theta_offset = robotModel.dh_table.T

        q_mat = self.matlab_col(q)

        H_mat = self.eng.forwardKinematics(
            q_mat,
            self.matlab_col(a),
            self.matlab_col(alpha),
            self.matlab_col(d),
            self.matlab_col(theta_offset),
            nargout=1
        )

        return np.array(H_mat, dtype=float)
    
    # Run inverse_kinematics.m in MATLAB using RobotModel DH parameters
    def inverseKinematics(self, robotModel, H):
        a, alpha, d, theta_offset = robotModel.dh_table.T
        H_mat = matlab.double(H.tolist())
        q_mat = self.eng.inverseKinematics(
            H_mat,
            self.matlab_col(a),
            self.matlab_col(alpha),
            self.matlab_col(d),
            self.matlab_col(theta_offset),
            nargout=1
        )

        return np.array(q_mat).flatten()
        

        
        