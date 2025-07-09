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

    def read(self, pin_num: int, pull_mode=GPIO.PUD_OFF, skip_setup=False) -> int:
        """
        Read the value from a specified GPIO pin.
            pin_num: BCM Pin number
            pull_mode:
                GPIO.PUD_UP     = read 3.3V by default
                GPIO.PUD_DOWN   = read 0V by default
                GPIO.PUD_OFF    = None
            skip_setup:
                Skip calling GPIO.setup, useful when it was already called previously
        """

        logger.debug(f"Reading value from pin {pin_num}")

        try:
            if not skip_setup:
                GPIO.setup(pin_num, GPIO.IN, pull_up_down=pull_mode)
            value = GPIO.input(pin_num)
            logger.info(f"Successfully read {value} from pin {pin_num}")
        except Exception as e:
            if isinstance(e, KeyBoardInterrupt):
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
            raise ValueError(f"Invalid GPIO value: {value}. Must be GPIO.LOW or GPIO.HIGH.")

        logger.debug(f"Writing value to pin {pin_num}")

        try:
            GPIO.setup(pin_num, GPIO.OUT)
            GPIO.output(pin_num, value)
            logger.info(f"Successfully wrote {value} to pin {pin_num}")
        except Exception as e:
            if isinstance(e, KeyBoardInterrupt):
                GPIO.cleanup()
            
            logger.error(f"Error writing {value} to pin {pin_num}: {e}")
            raise e
    
    def add_event_handler(
        self, 
        pin_num: int, 
        edge_type: int, 
        callback=dummy_callback, 
        pull_mode=GPIO.PUD_OFF,
        bouncetime=50
    ) -> None:
        """
        Set up GPIO pin for edge callback.
            pin_num: BCM Pin number
            edge_type: 
                GPIO.LOW
                GPIO.HIGH
            callback: Callback function
            pull_mode:
                GPIO.PUD_UP     = read 3.3V by default
                GPIO.PUD_DOWN   = read 0V by default
                GPIO.PUD_OFF    = None
            bouncetime: Ignore further changes on pin for this amount of time
        """

        logger.debug(f"Creating an interrupt on {pin_num}")

        try:
            GPIO.setup(pin_num, GPIO.IN, pull_up_down=pull_mode)
            GPIO.add_event_detect(pin_num, edge_type, callback=callback, bouncetime=bouncetime)
            logger.info(f"Successfully setup interrupt on {pin_num} with callback function: {callback.__name__}")
        except Exception as e:
            if isinstance(e, KeyBoardInterrupt):
                GPIO.cleanup()
            logger.error(f"Error Creating an interrupt on {pin_num}: {e}")
            raise e

    def cleanup(self) -> None:
        logger.debug(f"Calling cleanup")

        try:
            GPIO.cleanup()
            logger.info(f"Successfully called cleanup")
        except Exception as e:            
            logger.error(f"Error calling cleanup: {e}")
            raise e