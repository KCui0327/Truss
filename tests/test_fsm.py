# tests/test_fsm.py
from components.arm_fsm import ArmFSM


def test_halt_from_idle():
    fsm = ArmFSM()
    assert fsm.state == "IDLE"
    fsm.halt()
    assert fsm.state == "HALTED"


def test_start_to_home():
    fsm = ArmFSM()
    fsm.start()
    assert fsm.state == "HOME"


def test_fault_sets_fault_info():
    fsm = ArmFSM()
    fsm.faulted("x", "y")
    assert fsm.state == "FAULT"
    assert fsm.fault is not None
    assert fsm.fault.code == "x"
