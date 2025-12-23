import RPi.GPIO as GPIO
from logs.logging_setup import get_logger

logger = get_logger(__name__)

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
    
    def read(self, pin_num: int, ) -> int:
        """
        Read the value from a specified GPIO pin.
            pin_num: BCM Pin number
        """

        logger.debug(f"Reading value from pin {pin_num}")

        try:
            GPIO.setup(pin_num, GPIO.IN)
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
    
    def cleanup(self) -> None:
        logger.debug(f"Calling cleanup")

        try:
            GPIO.cleanup()
            logger.info(f"Successfully called cleanup")
        except Exception as e:            
            logger.error(f"Error calling cleanup: {e}")
            raise e