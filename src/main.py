from logs.logging_setup import init, get_logger
from components import GPIOController

def main() -> None:
    """
    Main function run the application.
    """

    init()
    logger = get_logger(__name__)
    logger.info("Application started.")
    gpio_controller = GPIOController()
    logger.info("Application finished.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logger = get_logger(__name__)
        logger.info("Application interrupted by user.")
        exit(0)
