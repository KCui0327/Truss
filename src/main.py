from logs.logging_setup import init, get_logger
from components import GPIOController
import RPi.GPIO as GPIO
from time import sleep

def test_callback(channel, controller):
    state = controller.read(27, skip_setup=1)
    if(state == GPIO.HIGH):
        print("Button released")
    else:
        print("Button pressed")
    #print(f"pin {channel}: State is {state}")

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
        #gpio_controller.write(27, GPIO.HIGH)
        #sleep(1)
        #gpio_controller.write(27, GPIO.LOW)
        
        gpio_controller.add_event_handler(
            27, 
            GPIO.BOTH, 
            #callback=lambda channel: test_callback(channel, gpio_controller),
            pull_mode=GPIO.PUD_UP
        )

        while True:
            sleep(1)
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
