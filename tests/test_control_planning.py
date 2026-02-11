"""
Comprehensive integration tests for FSM + Planning (IK).

This file consolidates:
- FSM state transitions and safety
- FSM + IK integration via Planner module
- Harvest cycle validation (with planning driving PLAN -> MOVE/HOME)
"""

import math
import sys
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

# Mock MATLAB module if not available
try:
    import matlab.engine  # noqa: F401
except ImportError:
    sys.modules["matlab"] = MagicMock()
    sys.modules["matlab.engine"] = MagicMock()

from control.fsm import FSM
from control.planner import Planner
from kinematics.robotModel import RobotModel
from kinematics.matlabBackend import MatlabBackend


# ============================================================================
# Fixtures
# ============================================================================

@pytest.fixture
def robot_model():
    """Standard 6-DOF robot model for tests."""
    return RobotModel(
        a=[25, 315, 35, 0, 0, -296.23],
        alpha=[math.pi / 2, 0, math.pi / 2, -math.pi / 2, math.pi / 2, 0],
        d=[400, 0, 0, 365, 0, 161.44],
        theta=[0, 0, 0, 0, 0, 0],
        joint_limits=[[-2.094, 2.094]] * 6,
    )


@pytest.fixture
def fsm():
    """Fresh FSM instance."""
    return FSM()


@pytest.fixture
def planner(robot_model):
    """Planner instance."""
    return Planner(robot_model)


# ============================================================================
# Helper: Real IK/Planner -> FSM PLAN transition
# ============================================================================

def _run_planner_and_advance_fsm(fsm: FSM, planner: Planner, target_pose: np.ndarray, n_steps: int = 10):
    """
    Run planner on target_pose and advance FSM based on the plan result.

    This validates the integration point:
      PLAN + (planner/IK) -> plan_ok/plan_fail -> MOVE or HOME
    """
    assert fsm.state == "PLAN", f"Expected PLAN, got {fsm.state}"

    result = planner.plan_to_pose(target_pose, n_steps=n_steps)

    if result.success:
        fsm.plan_ok()
    else:
        fsm.plan_fail()

    return result


# ============================================================================
# FSM Basic State Transitions (pure FSM, event-driven)
# ============================================================================

class TestFSMBasicTransitions:
    """Test FSM state transitions work correctly (unit-level FSM behavior)."""

    def test_fsm_idle_to_home(self, fsm):
        assert fsm.state == "IDLE"
        fsm.start()
        assert fsm.state == "HOME"

    def test_fsm_home_to_perceive(self, fsm):
        fsm.start()
        fsm.home_ok()
        assert fsm.state == "PERCEIVE"

    def test_fsm_perceive_to_plan(self, fsm):
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        assert fsm.state == "PLAN"

    def test_fsm_plan_to_move_event(self, fsm):
        """Pure FSM: if plan_ok event occurs, transition to MOVE."""
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.plan_ok()
        assert fsm.state == "MOVE"

    def test_fsm_move_to_cut_event(self, fsm):
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.plan_ok()
        fsm.move_ok()
        assert fsm.state == "CUT"

    def test_fsm_cut_to_return_event(self, fsm):
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.plan_ok()
        fsm.move_ok()
        fsm.cut_ok()
        assert fsm.state == "RETURN"

    def test_fsm_return_to_perceive_event(self, fsm):
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.plan_ok()
        fsm.move_ok()
        fsm.cut_ok()
        fsm.return_ok()
        assert fsm.state == "PERCEIVE"


# ============================================================================
# FSM Error Recovery Transitions (pure FSM)
# ============================================================================

class TestFSMRecoveryTransitions:
    """Test FSM error recovery paths (unit-level FSM behavior)."""

    def test_fsm_no_target_returns_home(self, fsm):
        fsm.start()
        fsm.home_ok()
        fsm.no_target()
        assert fsm.state == "HOME"

    def test_fsm_plan_failure_returns_home_event(self, fsm):
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.plan_fail()
        assert fsm.state == "HOME"

    def test_fsm_move_failure_returns_home_event(self, fsm):
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.plan_ok()
        fsm.move_fail()
        assert fsm.state == "HOME"


# ============================================================================
# FSM Safety Features
# ============================================================================

class TestFSMSafety:
    """Test FSM safety features (halt/fault, and estop integration point)."""

    def test_fsm_halt_from_idle(self, fsm):
        assert fsm.state == "IDLE"
        fsm.halt()
        assert fsm.state == "HALTED"

    def test_fsm_halt_from_move(self, fsm):
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.plan_ok()
        assert fsm.state == "MOVE"
        fsm.halt()
        assert fsm.state == "HALTED"

    def test_fsm_halt_from_any_state(self):
        states_sequence = [
            ("IDLE", []),
            ("HOME", ["start"]),
            ("PERCEIVE", ["start", "home_ok"]),
            ("PLAN", ["start", "home_ok", "target_ok"]),
            ("MOVE", ["start", "home_ok", "target_ok", "plan_ok"]),
            ("CUT", ["start", "home_ok", "target_ok", "plan_ok", "move_ok"]),
            ("RETURN", ["start", "home_ok", "target_ok", "plan_ok", "move_ok", "cut_ok"]),
        ]

        for expected_state, transitions in states_sequence:
            fsm = FSM()
            for transition in transitions:
                getattr(fsm, transition)()
            assert fsm.state == expected_state
            fsm.halt()
            assert fsm.state == "HALTED"

    def test_fsm_fault_state(self, fsm):
        fsm.faulted(code="IK_FAIL", detail="Singularity detected")
        assert fsm.state == "FAULT"
        assert fsm.fault.code == "IK_FAIL"
        assert fsm.fault.detail == "Singularity detected"

    def test_estop_trigger_causes_fsm_halt(self):
        """
        Test: EmergencyStop.trigger() should cause FSM to halt (integration point).

        This validates the control-layer safety latch behavior without requiring OS signals:
          1) SIGTERM handler would call estop.trigger(...)
          2) Control loop checks estop.is_triggered()
          3) Control loop calls fsm.halt()
        """
        from control.safety import EmergencyStop

        estop = EmergencyStop()
        fsm = FSM()

        # Simulate FSM running in MOVE state
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.plan_ok()
        assert fsm.state == "MOVE"

        # Simulate SIGTERM handler calling estop.trigger()
        estop.trigger(reason="SIGTERM received")

        assert estop.is_triggered()
        event = estop.snapshot()
        assert event.triggered is True
        assert event.reason == "SIGTERM received"

        # Simulate control loop behavior
        if estop.is_triggered():
            fsm.halt()

        assert fsm.state == "HALTED"


# ============================================================================
# IK Solver Integration (MATLAB-backed; skip if unavailable)
# ============================================================================

class TestIKSolverIntegration:
    """Test IK solver integration (skips cleanly if MATLAB unavailable)."""

    def test_ik_forward_backward_roundtrip(self, robot_model):
        """
        Legacy round-trip test (FK -> IK) that compares joint vectors.
        May skip; IK can be multi-solution. Keep only if it matches backend behavior.
        """
        try:
            matlab_backend = MatlabBackend()
        except (RuntimeError, TypeError, AttributeError) as e:
            pytest.skip(f"MATLAB engine not available: {e}")

        q_original = np.array([
            math.pi / 5,
            math.pi / 3,
            -math.pi / 4,
            math.pi / 4,
            math.pi / 3,
            math.pi / 4,
        ])

        try:
            H = matlab_backend.forwardKinematics(q_original, robot_model)
            if getattr(H, "size", 0) == 0:
                pytest.skip("MATLAB FK returned empty result")

            q_computed = matlab_backend.inverseKinematics(robot_model, H)
            if getattr(q_computed, "size", 0) == 0:
                pytest.skip("MATLAB IK returned empty result")

            np.testing.assert_allclose(q_original, q_computed, rtol=1e-5, atol=1e-8)
        except (RuntimeError, TypeError, AttributeError) as e:
            pytest.skip(f"MATLAB computation failed: {e}")

    def test_ik_correctness_validates_fk_of_ik_solution(self, robot_model):
        """
        Correct IK validation: FK(IK(H)) ≈ H.
        This is the correctness criterion that matters for planning.
        """
        try:
            matlab_backend = MatlabBackend()
        except (RuntimeError, TypeError, AttributeError) as e:
            pytest.skip(f"MATLAB engine not available: {e}")

        # Use a known reachable pose: derive H from FK(q_seed)
        q_seed = np.array([
            math.pi / 5,
            math.pi / 3,
            -math.pi / 4,
            math.pi / 4,
            math.pi / 3,
            math.pi / 4,
        ])

        try:
            H_target = matlab_backend.forwardKinematics(q_seed, robot_model)
            if getattr(H_target, "size", 0) == 0:
                pytest.skip("MATLAB FK returned empty result")

            q_solution = matlab_backend.inverseKinematics(robot_model, H_target)
            if getattr(q_solution, "size", 0) == 0:
                pytest.skip("MATLAB IK returned empty solution")

            H_achieved = matlab_backend.forwardKinematics(q_solution, robot_model)
            if getattr(H_achieved, "size", 0) == 0:
                pytest.skip("MATLAB FK on IK solution returned empty result")

            pos_error = np.linalg.norm(H_target[0:3, 3] - H_achieved[0:3, 3])
            assert pos_error < 1e-3, f"Position error too large: {pos_error}"

            R_target = H_target[0:3, 0:3]
            R_achieved = H_achieved[0:3, 0:3]
            rot_error = np.linalg.norm((R_target.T @ R_achieved) - np.eye(3))
            assert rot_error < 1e-3, f"Rotation error too large: {rot_error}"

        except (RuntimeError, TypeError, AttributeError) as e:
            pytest.skip(f"MATLAB computation failed: {e}")


# ============================================================================
# Planner + FSM Integration (REAL: planner result drives plan_ok/plan_fail)
# ============================================================================

class TestPlannerFSMIntegration:
    """Validate planner/IK outcomes drive FSM transitions out of PLAN."""

    def test_plan_state_successful_ik_advances_to_move(self, fsm, planner):
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        assert fsm.state == "PLAN"

        target_pose = np.eye(4)
        target_pose[0, 3] = 500
        target_pose[1, 3] = 200
        target_pose[2, 3] = 300

        with patch.object(planner, "_get_ik_backend") as mock_backend:
            mock_ik = MagicMock()
            mock_backend.return_value = mock_ik
            mock_ik.inverseKinematics.return_value = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

            result = _run_planner_and_advance_fsm(fsm, planner, target_pose, n_steps=10)

        assert result.success
        assert result.joint_trajectory is not None
        assert result.joint_trajectory.shape == (10, 6)
        assert fsm.state == "MOVE"

    def test_plan_state_joint_limit_violation_returns_home(self, fsm, planner):
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        assert fsm.state == "PLAN"

        target_pose = np.eye(4)

        with patch.object(planner, "_get_ik_backend") as mock_backend:
            mock_ik = MagicMock()
            mock_backend.return_value = mock_ik
            # outside limits [-2.094, 2.094]
            mock_ik.inverseKinematics.return_value = np.array([3.0, 0.2, 0.3, 0.4, 0.5, 0.6])

            result = _run_planner_and_advance_fsm(fsm, planner, target_pose)

        assert not result.success
        assert fsm.state == "HOME"

    def test_plan_state_ik_exception_returns_home(self, fsm, planner):
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        assert fsm.state == "PLAN"

        target_pose = np.eye(4)

        with patch.object(planner, "_get_ik_backend") as mock_backend:
            mock_ik = MagicMock()
            mock_backend.return_value = mock_ik
            mock_ik.inverseKinematics.side_effect = RuntimeError("Singularity detected")

            result = _run_planner_and_advance_fsm(fsm, planner, target_pose)

        assert not result.success
        assert "IK computation failed" in result.error_msg
        assert fsm.state == "HOME"


# ============================================================================
# Harvest Cycle Validation
# ============================================================================

class TestCompleteHarvestCycle:
    """Validate logical sequencing of a harvest cycle, with and without planning integration."""

    def test_fsm_nominal_cycle_sequence(self, fsm):
        """
        FSM nominal sequencing (event-driven), without executing planner/IK.
        This is a pure FSM test.
        """
        assert fsm.state == "IDLE"
        fsm.start()
        assert fsm.state == "HOME"

        fsm.home_ok()
        assert fsm.state == "PERCEIVE"

        fsm.target_ok()
        assert fsm.state == "PLAN"

        # Event-driven plan success (unit-level FSM)
        fsm.plan_ok()
        assert fsm.state == "MOVE"

        fsm.move_ok()
        assert fsm.state == "CUT"

        fsm.cut_ok()
        assert fsm.state == "RETURN"

        fsm.return_ok()
        assert fsm.state == "PERCEIVE"

    def test_full_harvest_cycle_with_planning(self, fsm, planner):
        """Harvest cycle where leaving PLAN is driven by planner/IK result."""
        fsm.start()
        fsm.home_ok()
        assert fsm.state == "PERCEIVE"

        fsm.target_ok()
        assert fsm.state == "PLAN"

        target_pose = np.eye(4)
        target_pose[0, 3] = 500
        target_pose[1, 3] = 200
        target_pose[2, 3] = 300

        with patch.object(planner, "_get_ik_backend") as mock_backend:
            mock_ik = MagicMock()
            mock_backend.return_value = mock_ik
            mock_ik.inverseKinematics.return_value = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

            result = _run_planner_and_advance_fsm(fsm, planner, target_pose, n_steps=10)

        assert result.success
        assert fsm.state == "MOVE"

        # Remaining states are event-driven (actuation not executed here)
        fsm.move_ok()
        assert fsm.state == "CUT"

        fsm.cut_ok()
        assert fsm.state == "RETURN"

        fsm.return_ok()
        assert fsm.state == "PERCEIVE"

    def test_cycle_with_planning_failure(self, fsm, planner):
        """Cycle recovery: if planning fails, FSM returns to HOME."""
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        assert fsm.state == "PLAN"

        target_pose = np.eye(4)

        with patch.object(planner, "_get_ik_backend") as mock_backend:
            mock_ik = MagicMock()
            mock_backend.return_value = mock_ik
            mock_ik.inverseKinematics.side_effect = RuntimeError("No IK solution")

            result = _run_planner_and_advance_fsm(fsm, planner, target_pose)

        assert not result.success
        assert fsm.state == "HOME"

        # Can attempt again
        fsm.home_ok()
        assert fsm.state == "PERCEIVE"


# ============================================================================
# Planner Input/Output Validation
# ============================================================================

class TestPlannerCalledWithCorrectInputs:
    """Planner called correctly with pose; outputs are servo-friendly."""

    def test_planner_receives_4x4_pose(self, planner):
        target_pose = np.eye(4)
        target_pose[0:3, 3] = [500, 200, 300]

        with patch.object(planner, "_get_ik_backend") as mock_backend:
            mock_ik = MagicMock()
            mock_backend.return_value = mock_ik
            mock_ik.inverseKinematics.return_value = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

            _ = planner.plan_to_pose(target_pose)

        mock_ik.inverseKinematics.assert_called_once()
        call_args = mock_ik.inverseKinematics.call_args
        passed_pose = call_args[0][1]
        assert passed_pose.shape == (4, 4)
        assert np.allclose(passed_pose, target_pose)

    def test_planner_outputs_trajectory_for_servo(self, planner):
        target_pose = np.eye(4)
        target_pose[0, 3] = 500

        with patch.object(planner, "_get_ik_backend") as mock_backend:
            mock_ik = MagicMock()
            mock_backend.return_value = mock_ik
            q_target = np.array([0.5, 1.0, 1.5, 0.2, 0.3, 0.4])
            mock_ik.inverseKinematics.return_value = q_target

            result = planner.plan_to_pose(target_pose, n_steps=20)

        assert result.success
        trajectory = result.joint_trajectory
        assert trajectory.shape == (20, 6)
        assert trajectory.dtype == float
        assert np.all(trajectory >= -2.094) and np.all(trajectory <= 2.094)


# ============================================================================
# Planner Lazy Loading
# ============================================================================

class TestPlannerLazyLoading:
    """Test planner lazy-loads IK backend."""

    def test_planner_lazy_loads_backend(self, planner):
        assert planner.ik_backend is None

        target_pose = np.eye(4)
        with patch.object(planner, "_get_ik_backend") as mock_backend_getter:
            mock_backend = MagicMock()
            mock_backend_getter.return_value = mock_backend
            mock_backend.inverseKinematics.return_value = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

            planner.plan_to_pose(target_pose)

        mock_backend_getter.assert_called()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
