"""
Minimal unit tests for FSM + IK integration.
Tests the current FSM with the current stub IK implementation.
"""

import pytest

from components.ik import IKSolver, IKResult
from components.arm_fsm import ArmFSM
from components.arm_control import ArmConfig, ArmController, ServoMoveParams
from components.safety import EmergencyStop


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


class TestFSMWithIK:
    """Test FSM state transitions with current IK solver."""

    def test_ik_returns_valid_result(self):
        """Current IK stub should return valid IKResult."""
        ik = IKSolver()
        result = ik.solve_to_pose(x=0.1, y=0.0, z=0.15, roll=0.0, pitch=0.0)
        assert isinstance(result, IKResult)
        assert result.success is True
        assert len(result.q_deg) == 5

    def test_fsm_transitions_idle_to_home(self):
        """FSM start() transitions IDLE -> HOME."""
        fsm = ArmFSM()
        assert fsm.state == "IDLE"
        fsm.start()
        assert fsm.state == "HOME"

    def test_fsm_transitions_home_to_perceive(self):
        """FSM home_ok() transitions HOME -> PERCEIVE."""
        fsm = ArmFSM()
        fsm.start()
        fsm.home_ok()
        assert fsm.state == "PERCEIVE"

    def test_fsm_transitions_perceive_to_plan(self):
        """FSM target_ok() transitions PERCEIVE -> PLAN."""
        fsm = ArmFSM()
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        assert fsm.state == "PLAN"

    def test_fsm_transitions_plan_to_move(self):
        """FSM plan_ok() transitions PLAN -> MOVE."""
        fsm = ArmFSM()
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.plan_ok()
        assert fsm.state == "MOVE"

    def test_fsm_transitions_move_to_cut(self):
        """FSM move_ok() transitions MOVE -> CUT."""
        fsm = ArmFSM()
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.plan_ok()
        fsm.move_ok()
        assert fsm.state == "CUT"

    def test_fsm_transitions_cut_to_return(self):
        """FSM cut_ok() transitions CUT -> RETURN."""
        fsm = ArmFSM()
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.plan_ok()
        fsm.move_ok()
        fsm.cut_ok()
        assert fsm.state == "RETURN"

    def test_fsm_transitions_return_to_perceive(self):
        """FSM return_ok() transitions RETURN -> PERCEIVE."""
        fsm = ArmFSM()
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.plan_ok()
        fsm.move_ok()
        fsm.cut_ok()
        fsm.return_ok()
        assert fsm.state == "PERCEIVE"

    def test_fsm_no_target_returns_home(self):
        """FSM no_target() transitions PERCEIVE -> HOME."""
        fsm = ArmFSM()
        fsm.start()
        fsm.home_ok()
        fsm.no_target()
        assert fsm.state == "HOME"

    def test_fsm_plan_fail_returns_home(self):
        """FSM plan_fail() transitions PLAN -> HOME."""
        fsm = ArmFSM()
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.plan_fail()
        assert fsm.state == "HOME"

    def test_fsm_move_fail_returns_home(self):
        """FSM move_fail() transitions MOVE -> HOME."""
        fsm = ArmFSM()
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.plan_ok()
        fsm.move_fail()
        assert fsm.state == "HOME"

    def test_fsm_halt_from_any_state(self):
        """FSM halt() should transition any state -> HALTED."""
        fsm = ArmFSM()
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.halt()
        assert fsm.state == "HALTED"

    def test_fsm_faulted_stores_info(self):
        """FSM faulted() should store fault info."""
        fsm = ArmFSM()
        fsm.faulted("test_code", "test_detail")
        assert fsm.state == "FAULT"
        assert fsm.fault.code == "test_code"
        assert fsm.fault.detail == "test_detail"


class TestControllerWithFSMAndIK:
    """Test ArmController coordinates FSM and IK."""

    def setup_method(self):
        """Setup test fixtures."""
        self.servo_ids = [1, 2, 3, 4, 5]
        self.q_home = [90.0, 90.0, 90.0, 90.0, 90.0]
        
        self.fsm = ArmFSM()
        self.estop = EmergencyStop()
        self.bus = SimServoBus(self.servo_ids, self.q_home)
        self.move_params = ServoMoveParams(
            tolerance_deg=2.0,
            poll_interval_s=0.10,
            timeout_s=1.0,
        )
        self.cfg = ArmConfig(
            servo_ids=self.servo_ids,
            q_home_deg=self.q_home,
            spline_steps=8,
            duration_ms=300,
        )

    def test_controller_has_ik(self):
        """Controller should have an IK solver."""
        ctrl = ArmController(
            fsm=self.fsm,
            estop=self.estop,
            bus=self.bus,
            move_params=self.move_params,
            cfg=self.cfg,
        )
        assert ctrl.ik is not None

    def test_controller_ik_solve_works(self):
        """Controller's IK should solve successfully."""
        ctrl = ArmController(
            fsm=self.fsm,
            estop=self.estop,
            bus=self.bus,
            move_params=self.move_params,
            cfg=self.cfg,
        )
        result = ctrl.ik.solve_to_pose(x=0.1, y=0.0, z=0.15, roll=0.0, pitch=0.0)
        assert result.success is True
        assert len(result.q_deg) == 5

    def test_controller_run_once_completes(self):
        """Controller run_once() should complete full cycle."""
        ctrl = ArmController(
            fsm=self.fsm,
            estop=self.estop,
            bus=self.bus,
            move_params=self.move_params,
            cfg=self.cfg,
        )
        ctrl.run_once()
        # Should end at PERCEIVE after RETURN -> PERCEIVE
        assert self.fsm.state == "PERCEIVE"

    def test_controller_run_once_transitions_through_plan(self):
        """run_once() should transition through PLAN state with IK."""
        ctrl = ArmController(
            fsm=self.fsm,
            estop=self.estop,
            bus=self.bus,
            move_params=self.move_params,
            cfg=self.cfg,
        )
        # Manually step to PLAN state
        self.fsm.start()
        self.fsm.home_ok()
        self.fsm.target_ok()
        assert self.fsm.state == "PLAN"
        
        # IK should succeed at this point
        target = {"x": 0.1, "y": 0.0, "z": 0.15, "roll": 0.0, "pitch": 0.0}
        ikr = ctrl.ik.solve_to_pose(**target)
        assert ikr.success is True
        
        # FSM should transition past PLAN
        self.fsm.plan_ok()
        assert self.fsm.state == "MOVE"

    def test_controller_ik_with_limits(self):
        """IK should respect joint limits if provided."""
        limits = [(0, 80), (0, 80), (0, 80), (0, 80), (0, 80)]
        ik = IKSolver(joint_limits_deg=limits)
        
        # Stub returns [90, 90, ...] which exceeds limits
        result = ik.solve_to_pose(x=0.1, y=0.0, z=0.15, roll=0.0, pitch=0.0)
        assert result.success is False
        assert "out of limits" in result.reason
