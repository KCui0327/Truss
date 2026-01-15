import inspect

from components.arm_fsm import ArmFSM
from components.safety import EmergencyStop
from components.arm_control import ArmController, ArmConfig, ServoMoveParams


class SimServoBus:
    """Simulated servo bus for testing."""

    def __init__(self, servo_ids, q_home):
        self.servo_ids = servo_ids
        self.positions = {sid: q_home[i] for i, sid in enumerate(servo_ids)}
        self.enabled = True

    def read_position(self, servo_id):
        return self.positions.get(servo_id, 0.0)

    def write_position(self, servo_id, angle_deg, duration_ms):
        if not self.enabled:
            return
        self.positions[servo_id] = angle_deg

    def disable_all(self):
        self.enabled = False


def _make_controller(fsm, estop, bus, params, servo_ids, q_home):
    """
    Build ArmController using whatever constructor shape exists.
    This keeps the test resilient to small refactors.
    """
    sig = inspect.signature(ArmController.__init__)
    kwargs = {}

    # Common fields
    for name in sig.parameters.keys():
        if name == "self":
            continue
        if name == "fsm":
            kwargs[name] = fsm
        elif name == "estop":
            kwargs[name] = estop
        elif name == "bus":
            kwargs[name] = bus
        elif name == "move_params":
            kwargs[name] = params
        elif name in ("cfg", "config"):
            # Your controller may use an ArmConfig/ArmCfg object
            # If so, import and build it here.
            kwargs[name] = ArmConfig(
                servo_ids=servo_ids,
                q_home_deg=q_home,
                spline_steps=5,
                duration_ms=300,
            )
        elif name == "servo_ids":
            kwargs[name] = servo_ids
        elif name in ("q_home", "q_home_deg"):
            kwargs[name] = q_home
        elif name == "spline_steps":
            kwargs[name] = 5
        elif name == "duration_ms":
            kwargs[name] = 300

    return ArmController(**kwargs)


def test_controller_halts_when_estop_triggered():
    fsm = ArmFSM()
    estop = EmergencyStop()

    servo_ids = [1, 2, 3, 4, 5]
    q_home = [90, 90, 90, 90, 90]

    bus = SimServoBus(servo_ids, q_home)
    params = ServoMoveParams(timeout_s=0.1, poll_interval_s=0.01)

    arm = _make_controller(fsm, estop, bus, params, servo_ids, q_home)

    # Trigger estop before running
    estop.trigger("test estop")

    # Run whichever “one cycle” entrypoint exists
    if hasattr(arm, "run_cycle"):
        arm.run_cycle()
    elif hasattr(arm, "run_once"):
        arm.run_once()
    else:
        raise AssertionError("ArmController has no run_cycle() or run_once() method")

    assert fsm.state == "HALTED"
