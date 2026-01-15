from .safety import EmergencyStop
from .fsm import FSM
from logutils import get_logger

def main() -> None:
    logger = get_logger(__name__)

    logger.info("Truss FSM starting")

    estop = EmergencyStop()
    fsm = FSM()

    # TODO: the following are placeholders for actual values
    servo_ids = [1, 2, 3, 4, 5]
    q_home = [90.0, 90.0, 90.0, 90.0, 90.0]


if __name__ == "__main__":
    main()
