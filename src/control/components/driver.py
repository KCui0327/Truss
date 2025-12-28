from kinematics.robotModel import RobotModel 
from kinematics.matlabBackend import MatlabBackend
from servo import busServoCmd
from servo import board
import time

TOLERANCE = 0.1
class Driver:
    def __init__(self):
        self.robot = RobotModel(
            a=[],
            alpha=[],
            d=[],
            joint_limits=[[-2.094, 2.094]] * 5,
        )
        self.matlab = MatlabBackend()
        
if __name__ == "__main__":
    driver = Driver()
    heavyMotorIds = [1, 2]
    H = [] # Representing the desired end effector position - obtained from depth perception 
    q_end = driver.matlab.inverseKinematics(driver.robot, H) # 5 element array of joint angles
    for i, angle in enumerate(q_end):
        retry_count = 0
        # actuate servo motor
        board.setBusServoPulse(i + 1)
        while (abs(board.getPWMServoAngle - angle) > TOLERANCE):
            if retry_count == 0:
                time.sleep(0.5)
                board.setBusServoPulse(i + 1)
                retry_count += 1
            elif retry_count == 1:
                time.sleep(1)
                board.setBusServoPulse(i + 1)
                retry_count += 1
            else:
                break # implement return home
        
                
        
                
                
                
            
            
            
            
        
            
            
                
                