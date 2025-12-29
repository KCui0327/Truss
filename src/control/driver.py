from kinematics.robotModel import RobotModel 
from kinematics.matlabBackend import MatlabBackend
from control.servo import busServoCmd
from control.servo import board
import time
import math

TOLERANCE = 0.1
class Driver:
    def __init__(self):
        self.robot = RobotModel(
            a=[25, 315, 35, 0, 0, -296.23],
            alpha=[math.pi/2, 0, math.pi/2, -math.pi/2, math.pi/2, 0],
            d=[400, 0, 0, 365, 0, 161.44],
            theta=[0, 0, 0, 0, 0, 0],
            joint_limits=[[-2.094, 2.094]] * 6,
        )
        self.matlab = MatlabBackend()
        
if __name__ == "__main__":
    driver = Driver()
    heavyMotorIds = [1, 2]
    
    # q = [math.pi/5, math.pi/3, -math.pi/4, math.pi/4, math.pi/3, math.pi/4]
    # H = driver.matlab.forwardKinematics(q, driver.robot)
    # print(H)
    
    H = [] # Representing the desired end effector position - obtained from depth perception 
    q_end = driver.matlab.inverseKinematics(driver.robot, H) 

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
        
                
        
                
                
                
            
            
            
            
        
            
            
                
                