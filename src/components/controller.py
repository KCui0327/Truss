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
    
    def read(pin_num: int, ) -> int:
        """
        Read the value from a specified GPIO pin.
        """

        logger.debug(f"Reading value from pin {pin_num}")

        try:
            val = GPIO.input(pin_num)
        except Exception as e:
            if isinstance(e, KeyBoardInterrupt):
                    GPIO.cleanup()

            logger.error(f"Error reading from pin {pin_num}: {e}")
            return -1
        
        return val

    def write(pin_num: int, ) -> None:
        """
        Write a value to a specified GPIO pin.
        """

        logger.debug(f"Writing value to pin {pin_num}")

        try:
            GPIO.output(pin_num, GPIO.HIGH)
            logger.info(f"Successfully wrote to pin {pin_num}")
        except Exception as e:
            if isinstance(e, KeyBoardInterrupt):
                GPIO.cleanup()
            
            logger.error(f"Error writing to pin {pin_num}: {e}")
            raise e
