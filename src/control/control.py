from fsm import FSM
import signal

fsm = FSM()

def handle_emergency_stop():
    fsm.halt()
    # emergency stop handling FSM handling logic can be added here

signal.signal(signal.SIGTERM, handle_emergency_stop) # register handler for emergency stop signal

#TODO: for dalia to fill in with actual control logic
def control():
    fsm.start()

    while True:
        fsm.home_ok()  # HOME -> PERCEIVE

        target = None #TODO: replace with perception stuff
        if target is None:
            fsm.no_target()
            return

        fsm.target_ok()  # PERCEIVE -> PLAN

        ikr = None #TODO: replace with IK stuff
        if not ikr.success:
            fsm.plan_fail()
            return

        fsm.plan_ok()  # PLAN -> MOVE
        fsm.move_ok()  # MOVE -> CUT
        fsm.cut_ok()  # CUT -> RETURN
        fsm.return_ok()

if __name__ == "__main__":
    control()
