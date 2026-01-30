# tests/test_fsm.py
from control.fsm import FSM


def test_halt_from_idle():
    fsm = FSM()
    assert fsm.state == "IDLE"
    fsm.halt()
    assert fsm.state == "HALTED"


def test_start_to_home():
    fsm = FSM()
    fsm.start()
    assert fsm.state == "HOME"


def test_fault_sets_fault_info():
    fsm = FSM()
    fsm.faulted("x", "y")
    assert fsm.state == "FAULT"
    assert fsm.fault is not None
    assert fsm.fault.code == "x"
