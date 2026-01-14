import serial
import time
import driver.servo_fns as servo_fns

HOME_ANGLE = None #TODO: Define the home angle position
EXPONENTIAL_BACKOFF_BASE = [0.5, 1]  # in seconds

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

    def return_home(self, duration):
        servo_fns.setBusServoPulse(1, self.angle_to_pos(HOME_ANGLE), duration, self.serial_handle)
        servo_fns.setBusServoPulse(2, self.angle_to_pos(HOME_ANGLE), duration, self.serial_handle)
        servo_fns.setBusServoPulse(3, self.angle_to_pos(HOME_ANGLE), duration, self.serial_handle)
        servo_fns.setBusServoPulse(4, self.angle_to_pos(HOME_ANGLE), duration, self.serial_handle)
        servo_fns.setBusServoPulse(5, self.angle_to_pos(HOME_ANGLE), duration, self.serial_handle)

        time.sleep(duration + 500)  # Wait for the movement to complete
    
    def move_servo_motor(self, servo_id, angle, duration, tol=10):
        try:
            target_pos = self.angle_to_pos(angle)
            servo_fns.setBusServoPulse(servo_id, target_pos, duration, self.serial_handle)
            time.sleep(duration / 1000)  # Wait for the movement to complete

            curr_backoff_index = 0
            while True:
                actual_pos = servo_fns.getBusServoPulse(servo_id, self.serial_handle)
                print(actual_pos)
                if abs(actual_pos - target_pos) <= tol:
                    break
                servo_fns.setBusServoPulse(servo_id, target_pos, 500, self.serial_handle)
                time.sleep(EXPONENTIAL_BACKOFF_BASE[curr_backoff_index])  # Sleep for the first backoff duration
                if curr_backoff_index < len(EXPONENTIAL_BACKOFF_BASE) - 1:
                    curr_backoff_index += 1
                else:
                    print(f"Warning: Servo {servo_id} did not reach the target angle {angle}. Actual angle: {self.pos_to_angle(actual_pos)}")
                    break
        except Exception as e:
            print(f"Error moving servo {servo_id}: {e}")


""" servo = ServoController()

#servo.return_home(5000)
x=200
print(f"target position: {servo.angle_to_pos(x)}")
servo.move_servo_motor(1,x,5000) """