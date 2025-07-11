import RPi.GPIO as GPIO
from logs.logging_setup import get_logger

logger = get_logger(__name__)

def dummy_callback(channel):
    print(f"Pin: {channel}: In dummy")
    return

class GPIOController:
    """
    GPIOController: A class to handle GPIO operations.
    """

    def __init__(self):
        self.setup()
    
    def setup(self) -> None:
        """
        Setup the GPIO pins for the Raspberry Pi.
        """

        logger.info("Setting up GPIO pins.")
        GPIO.setmode(GPIO.BCM)
        logger.info("GPIO pins setup complete.")

    def setup_pin(self, pin_num: int, mode: int, pull_mode=GPIO.PUD_OFF) -> int:
        """
        Setup the specified GPIO pin.
            pin_num: BCM Pin number
            mode:
                GPIO.IN
                GPIO.OUT
            pull_mode:
                GPIO.PUD_UP     = read 3.3V by default
                GPIO.PUD_DOWN   = read 0V by default
                GPIO.PUD_OFF    = None
        """
        if pull_mode not in (GPIO.PUD_UP, GPIO.PUD_DOWN, GPIO.PUD_OFF):
            logger.error(f"Invalid GPIO value: {value}. Must be GPIO.PUD_UP, GPIO.PUD_DOWN, GPIO.PUD_OFF.")
            raise ValueError(f"Invalid GPIO pull_mode: {pull_mode}. Must be GPIO.PUD_UP, GPIO.PUD_DOWN, or GPIO.PUD_OFF.")
        
        if(mode == GPIO.OUT and pull_mode in (GPIO.PUD_UP, GPIO.PUD_DOWN)):
            logger.error(f"Setup failed, pull_mode (GPIO.PUD_UP, GPIO.PUD_DOWN) is not valid for output")
            raise ValueError(f"Setup failed, pull_mode (GPIO.PUD_UP, GPIO.PUD_DOWN) is not valid for output.")

        logger.debug(f"Setting up pin {pin_num}")
        try:
            GPIO.setup(pin_num, mode, pull_up_down=pull_mode)
            logger.info(f"Successfully setup from pin {pin_num}")
        except Exception as e:
            if isinstance(e, KeyboardInterrupt ):
                    GPIO.cleanup()
            logger.error(f"Error setting up pin {pin_num}: {e}")
            raise e

    def read(self, pin_num: int) -> int:
        """
        Read the value from a specified GPIO pin.
            pin_num: BCM Pin number

        Return:
            (readback_value, ErrorCode)
        """

        logger.debug(f"Reading value from pin {pin_num}")

        try:
            value = GPIO.input(pin_num)
            logger.info(f"Successfully read {value} from pin {pin_num}")
        except Exception as e:
            if isinstance(e, KeyboardInterrupt):
                    GPIO.cleanup()

            logger.error(f"Error reading from pin {pin_num}: {e}")
            return -1
        
        return value

    def write(self, pin_num: int, value: int) -> None:
        """
        Write a value to a specified GPIO pin.
            pin_num: BCM Pin number
            value: 
                GPIO.LOW
                GPIO.HIGH
        """
        
        if value not in (GPIO.LOW, GPIO.HIGH):
            logger.error(f"Invalid GPIO value: {value}. Must be GPIO.LOW or GPIO.HIGH.")
            raise ValueError(f"Invalid GPIO value: {value}. Must be GPIO.LOW or GPIO.HIGH.")

        logger.debug(f"Writing value to pin {pin_num}")

        try:
            GPIO.output(pin_num, value)
            if(GPIO.input(pin_num) == value):
                logger.info(f"Successfully wrote {value} to pin {pin_num}")
            else:
                logger.error(f"Readback of value {value} to pin {pin_num} is incorrect.")
                raise RuntimeError(f"Invalid GPIO value: Readback of value {value} to pin {pin_num} is incorrect.")
                
        except Exception as e:
            if isinstance(e, KeyboardInterrupt):
                GPIO.cleanup()
            
            logger.error(f"Error writing {value} to pin {pin_num}: {e}")
            raise e
    
    def add_event_handler(
        self, 
        pin_num: int, 
        edge_type: int, 
        callback=dummy_callback, 
        bouncetime=50
    ) -> None:
        """
        Set up GPIO pin for edge callback.
            pin_num: BCM Pin number
            edge_type: 
                GPIO.LOW
                GPIO.HIGH
                GPIO.BOTH
            callback: Callback function
            bouncetime: Ignore further changes on pin for this amount of time

        Recommend setting up the pin with pull mode before calling this function.
        """
        if edge_type not in (GPIO.LOW, GPIO.HIGH, GPIO.BOTH):
            logger.error(f"Invalid GPIO value: {value}. Must be GPIO.LOW, GPIO.HIGH, GPIO.BOTH.")
            raise ValueError(f"Invalid GPIO value: {value}. Must be GPIO.LOW, GPIO.HIGH, GPIO.BOTH.")

        logger.debug(f"Setting up event handler on {pin_num}")
        try:
            GPIO.add_event_detect(pin_num, edge_type, callback=callback, bouncetime=bouncetime)
            logger.info(f"Successfully setup event handler on {pin_num} with callback function: {callback.__name__}")
        except Exception as e:
            if isinstance(e, KeyboardInterrupt):
                GPIO.cleanup()
            logger.error(f"Error Creating an event handler on {pin_num}: {e}")
            raise e

    def cleanup(self) -> None:
        logger.debug(f"Calling cleanup")

        try:
            GPIO.cleanup()
            logger.info(f"Successfully called cleanup")
        except Exception as e:            
            logger.error(f"Error calling cleanup: {e}")
            raise e