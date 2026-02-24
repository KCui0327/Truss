import serial
import time
from .driver import servo_fns
from logutils import init, get_logger

init()
logger = get_logger("ServoController")

BAUD_RATE = 115200
home_position = [120,120,210,120,120] #Mapped to matlab angles
stable_home_position = [120,120,180,210,120] # Better home configuration practically

polarity_vec = [1, 1, -1, 1, 1]
offset_vec = [120.0, 17.94, 467.94, 120.0, 120.0]

EXPONENTIAL_BACKOFF_BASE = [2, 4, 6]  # in seconds

class ServoController:
    def __init__(self):
        try:
            logger.info("Initializing ServoController with baud rate %d", BAUD_RATE)
            self.serial_handle = serial.Serial("/dev/ttyUSB0", baudrate=BAUD_RATE)
            logger.info("Opened serial connection on /dev/ttyUSB0 at %d", BAUD_RATE)
        except serial.SerialException as e:
            logger.exception("Error initializing serial port")
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
            logger.error("Invalid duration for return_home: %s", duration)
            raise ValueError(f"Invalid duration: {duration}\nValid between 500-30000")
        if not parallel:
            for i, angle in enumerate(stable_home_position):
                logger.info("Moving servo %d to home angle %s", i+1, angle)
                self.move_servo_motor(i+1, angle, duration)
        else:
            for i, angle in enumerate(stable_home_position):
                servo_fns.setBusServoPulse(i+1, self.angle_to_pos(angle), duration, self.serial_handle)
            time.sleep(duration / 1000)  # Wait for the movement to complete
            logger.debug("Parallel return_home movement completed in %d ms", duration)
    
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
            logger.error("Invalid duration for move_robot: %s", duration)
            raise ValueError(f"Invalid duration: {duration}\nValid between 500-30000")
        
        if not parallel:
            for i, angle in enumerate(angle_list):
                logger.info("Moving servo %d to angle %s", i+1, angle)
                self.move_servo_motor(i+1, angle, duration)
        else:
            for i, angle in enumerate(angle_list):
                servo_fns.setBusServoPulse(i+1, self.angle_to_pos(angle), duration, self.serial_handle)
            time.sleep(duration / 1000)  # Wait for the movement to complete
            logger.debug("Parallel move_robot completed in %d ms", duration)

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
                logger.error("Invalid angle provided to move_servo_motor: %s", angle)
                raise ValueError(f"Invalid angle: {angle}")
            if duration < 500 or duration > 30000:
                logger.error("Invalid duration provided to move_servo_motor: %s", duration)
                raise ValueError(f"Invalid duration: {duration}\nValid between 500-30000")  

            target_pos = self.angle_to_pos(angle)
            logger.debug("Setting servo %s target pos %s (angle %s) with duration %s", servo_id, target_pos, angle, duration)
            servo_fns.setBusServoPulse(servo_id, target_pos, duration, self.serial_handle)
            time.sleep(duration / 1000)  # Wait for the movement to complete

            curr_backoff_index = 0
            while True:
                actual_pos = servo_fns.getBusServoPulse(servo_id, self.serial_handle, timeout=1)
                logger.debug("Read servo %s actual pos: %s", servo_id, actual_pos)
                if actual_pos is None:
                    logger.error("Servo read timeout for servo %s", servo_id)
                    raise TimeoutError(f"Timeout Error: Servo Read Timeout after 1 second")

                if abs(actual_pos - target_pos) <= tol:
                    logger.info("Servo %s reached target position (pos=%s, tol=%s)", servo_id, actual_pos, tol)
                    break
                backoff = EXPONENTIAL_BACKOFF_BASE[curr_backoff_index]
                logger.warning("Servo %s did not reach target yet. Retrying with backoff %s seconds (index %s)", servo_id, backoff, curr_backoff_index)
                servo_fns.setBusServoPulse(servo_id, target_pos, backoff * 1000, self.serial_handle)
                time.sleep(backoff)  # Sleep for the backoff duration
                if curr_backoff_index < len(EXPONENTIAL_BACKOFF_BASE) - 1:
                    curr_backoff_index += 1
                else:
                    logger.error("Servo %s did not reach the target angle %s after backoffs. Actual angle: %s", servo_id, angle, self.pos_to_angle(actual_pos))
                    break
        except Exception as e:
            logger.exception("Error moving servo %s: %s", servo_id, e)

    def get_servo_angle(self, servo_id, timeout=1):
        """ 
        Get servo angle

        Args:
            servo_id: servo id
            timeout: Timeout value for servo read (default 1 second)
                
        Return: servo_id's angle (in degrees)
        """
        try:
            actual_pos = servo_fns.getBusServoPulse(servo_id, self.serial_handle, timeout)
            logger.debug("Read servo %s actual pos: %s", servo_id, actual_pos)
            if actual_pos is None:
                logger.error("Servo read timeout for servo %s", servo_id)
                raise TimeoutError(f"Timeout Error: Servo Read Timeout after 1 second")
        except Exception as e:
            logger.exception("Error reading servo %s: %s", servo_id, e)
            return None
        return self.pos_to_angle(actual_pos)


def matlab_to_servo_angles(matlab_q):
    servo_q = [0] * 5

    for i in range(5):
        # y = Ax + B
        servo_q[i] = polarity_vec[i] * matlab_q[i] + offset_vec[i]

    return servo_q


if __name__ == "__main__":
    logger.info("Running servo sandbox code")
    servo = ServoController()

    # Matlab Angles Example (Move to home position)
    #servo_position = matlab_to_servo_angles([0, 102.06, 257.94, 0, 0])
    #servo.move_robot(servo_position, 8000, parallel=True)

    # Return Home Example
    #servo.return_home(8000,parallel=True)

    # Get Angle Example
    #servo_3_angle = servo.get_servo_angle(3)
    #logger.info("Servo 3 angle: %s", servo_3_angle)
    