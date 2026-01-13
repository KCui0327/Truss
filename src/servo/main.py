import serial
import time
import driver.servo_fns as servo_fns

HOME_ANGLE = None #TODO: Define the home angle position
EXPONENTIAL_BACKOFF_BASE = [500, 1000]  # in milliseconds

class ServoController:
    def __init__(self):
        try:
            self.serial_handle = serial.Serial("/dev/ttyUSB0", 115200)
        except serial.SerialException as e:
            raise Exception(f"Error initializing serial port: {e}")
    
    def return_home(self, duration):
        servo_fns.setBusServoPulse(1, HOME_ANGLE, duration, self.serial_handle)
        servo_fns.setBusServoPulse(2, HOME_ANGLE, duration, self.serial_handle)
        servo_fns.setBusServoPulse(3, HOME_ANGLE, duration, self.serial_handle)
        servo_fns.setBusServoPulse(4, HOME_ANGLE, duration, self.serial_handle)
        servo_fns.setBusServoPulse(5, HOME_ANGLE, duration, self.serial_handle)

        time.sleep(duration + 500)  # Wait for the movement to complete
    
    def move_servo_motor(self, servo_id, angle, duration):
        try:
            servo_fns.setBusServoPulse(servo_id, angle, duration, self.serial_handle)
            time.sleep(duration + 500)  # Wait for the movement to complete

            curr_backoff_index = 0
            while True:
                pos = servo_fns.getBusServoPulse(servo_id, self.serial_handle)
                if abs(pos - angle) <= 10:
                    break
                time.sleep(EXPONENTIAL_BACKOFF_BASE[curr_backoff_index] / 1000)  # Sleep for the first backoff duration
                if curr_backoff_index < len(EXPONENTIAL_BACKOFF_BASE) - 1:
                    curr_backoff_index += 1
                else:
                    print(f"Warning: Servo {servo_id} did not reach the target angle {angle}. Current position: {pos}")
                    break
        except Exception as e:
            print(f"Error moving servo {servo_id}: {e}")

    