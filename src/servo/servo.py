import serial
import time
import driver.servo_fns as servo_fns

home_position = [120,120,210,120,120] #Mapped to matlab angles
stable_home_position = [120,120,180,210,120] # Better home configuration practically

EXPONENTIAL_BACKOFF_BASE = [2, 4, 6]  # in seconds

class ServoController:
    def __init__(self):
        try:
            self.serial_handle = serial.Serial("/dev/ttyUSB0", 115200)
        except serial.SerialException as e:
            raise Exception(f"Error initializing serial port: {e}")
    
    def angle_to_pos(self, angle):
        return int(angle * (1000/240))

    def pos_to_angle(self, pos):
        return int(pos * (240/1000))

    def return_home(self, duration, parallel=False):
        """ 
        Return robot to home configuration

        Args:
            duration: time in milliseconds (0-30000)
                Note: Adding soft min limit of 500ms
            parallel: 
                True - run servos together
                False - run one by one
                
        Return: None
        """
        if duration < 500 or duration > 30000:
            raise ValueError(f"Invalid duration: {duration}\nValid between 500-30000")
        if not parallel:
            for i, angle in enumerate(stable_home_position):
                print(f"Moving servo {i+1}")
                self.move_servo_motor(i+1, angle, duration)
        else:
            for i, angle in enumerate(stable_home_position):
                servo_fns.setBusServoPulse(i+1, self.angle_to_pos(angle), duration, self.serial_handle)
            time.sleep(duration / 1000)  # Wait for the movement to complete
    
    def move_robot(self, angle_list, duration, parallel=False):
        """ 
        Move robot to user-defined angle configuration

        Args:
            angle_list: [5x1] List of servo motor angles
            duration: time in milliseconds (0-30000)
                Note: Adding soft min limit of 500ms
                
        Return: None
        """
        if duration < 500 or duration > 30000:
            raise ValueError(f"Invalid duration: {duration}\nValid between 500-30000")
        
        if not parallel:
            for i, angle in enumerate(angle_list):
                print(f"Moving servo {i+1}")
                self.move_servo_motor(i+1, angle, duration)
        else:
            for i, angle in enumerate(angle_list):
                servo_fns.setBusServoPulse(i+1, self.angle_to_pos(angle), duration, self.serial_handle)
            time.sleep(duration / 1000)  # Wait for the movement to complete

    def move_servo_motor(self, servo_id, angle, duration, tol=10):
        """ 
        Move servo motor to angle within duration (ms)

        Args:
            servo_id: servo id
            angle: servo angle (0-240)
            duration: time in milliseconds (0-30000)
                Note: Adding soft min limit of 500ms
            tol: tolerance of target position error (default +/- 10 positions)
                
        Return: None
        """
        try:
            if not 0 <= angle <= 240:
                raise ValueError(f"Invalid angle: {angle}")
            if duration < 500 or duration > 30000:
                raise ValueError(f"Invalid duration: {duration}\nValid between 500-30000")  

            target_pos = self.angle_to_pos(angle)
            servo_fns.setBusServoPulse(servo_id, target_pos, duration, self.serial_handle)
            time.sleep(duration / 1000)  # Wait for the movement to complete

            curr_backoff_index = 0
            while True:
                print("Before blocking call")
                actual_pos = servo_fns.getBusServoPulse(servo_id, self.serial_handle, timeout=1) # blocking call, need timeout
                if actual_pos is None:
                    raise TimeoutError(f"Timeout Error: Servo Read Timeout after 1 second")

                print("After blocking call")

                if abs(actual_pos - target_pos) <= tol:
                    break
                servo_fns.setBusServoPulse(servo_id, target_pos, EXPONENTIAL_BACKOFF_BASE[curr_backoff_index] * 1000, self.serial_handle)
                time.sleep(EXPONENTIAL_BACKOFF_BASE[curr_backoff_index])  # Sleep for the first backoff duration
                if curr_backoff_index < len(EXPONENTIAL_BACKOFF_BASE) - 1:
                    curr_backoff_index += 1
                else:
                    print(f"Warning: Servo {servo_id} did not reach the target angle {angle}. Actual angle: {self.pos_to_angle(actual_pos)}")
                    break
        except Exception as e:
            print(f"Error moving servo {servo_id}: {e}")

    def get_servo_angle(self, servo_id, timeout=1):
        try:
            actual_pos = servo_fns.getBusServoPulse(servo_id, self.serial_handle, timeout)
            if actual_pos is None:
                raise TimeoutError(f"Timeout Error: Servo Read Timeout after 1 second")
        except Exception as e:
            print(f"Error reading servo {servo_id}: {e}")
            return None
        return self.pos_to_angle(actual_pos)


def matlab_to_servo_angles(matlab_q):
    servo_q = [0] * 5

    servo_q[0] = matlab_q[0] + 120
    servo_q[1] = matlab_q[1] + 17.94
    servo_q[2] = -matlab_q[2] + 467.94
    servo_q[3] = matlab_q[3] + 120
    servo_q[4] = matlab_q[4] + 120

    return servo_q

if __name__ == "__main__":
    print("Sandbox code")
    servo = ServoController()

    #servo_position = matlab_to_servo_angles([0, 102.06, 257.94, 0, 0])
    #print(servo_position)
    #servo.move_robot(servo_position, 8000)

    #new_position = [x - 50 for x in stable_home_position]
    #new_position = [120,220,210,210,120]
    #print(new_position)
    #servo.move_robot(new_position, 8000, parallel=True)

    servo.return_home(8000,parallel=True)

    #servo_3_angle = servo.get_servo_angle(3)
    #print(servo_3_angle)
    