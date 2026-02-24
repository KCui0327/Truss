from .fsm import FSM
import signal
from logutils import init, get_logger

init()
logger = get_logger("Control")

fsm = FSM()

def handle_emergency_stop(signum=None, frame=None):
    try:
        logger.warning("Emergency stop signal received: %s", signum)
        fsm.halt()
        logger.info("FSM halted due to emergency stop")
    except Exception:
        logger.exception("Exception while handling emergency stop")


try:
    signal.signal(signal.SIGTERM, handle_emergency_stop) # register handler for emergency stop signal
    logger.debug("Registered SIGTERM handler for emergency stop")
except Exception:
    logger.exception("Failed to register SIGTERM handler")


#TODO: for dalia to fill in with actual control logic
def control():
    logger.info("Starting control loop")
    try:
        fsm.start()
        logger.debug("FSM start() called")

        while True:
            logger.debug("Signaling home_ok transition")
            fsm.home_ok()  # HOME -> PERCEIVE

            target = None #TODO: replace with perception stuff
            if target is None:
                logger.info("No target acquired; signaling no_target and exiting control loop")
                fsm.no_target()
                return

            logger.debug("Target acquired; signaling target_ok")
            fsm.target_ok()  # PERCEIVE -> PLAN

            ikr = None #TODO: replace with IK stuff
            if ikr is None:
                logger.warning("IKR object is None; signaling plan_fail and exiting")
                fsm.plan_fail()
                return
            if not getattr(ikr, 'success', False):
                logger.warning("IKR reported failure: %s", getattr(ikr, 'success', None))
                fsm.plan_fail()
                return

            logger.debug("IKR successful; signaling plan_ok")
            fsm.plan_ok()  # PLAN -> MOVE
            fsm.move_ok()  # MOVE -> CUT
            fsm.cut_ok()  # CUT -> RETURN
            fsm.return_ok()

    except Exception:
        logger.exception("Unhandled exception in control loop")


if __name__ == "__main__":
    control()
