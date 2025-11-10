from logs.logging_setup import init, get_logger
from components import GPIOController
import RPi.GPIO as GPIO
from time import sleep

def main() -> None:
    """
    Main function run the application.
    """

    init()
    logger = get_logger(__name__)
    logger.info("Application started.")
    gpio_controller = GPIOController()
    
    try:
        # Complete GPIO operations in this block
        gpio_controller.write(27, GPIO.HIGH)
        sleep(1)
        gpio_controller.write(27, GPIO.LOW)
    finally:
        gpio_controller.cleanup()
    logger.info("Application finished.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logger = get_logger(__name__)
        logger.info("Application interrupted by user.")
        exit(0)
