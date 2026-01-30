"""
Emergency Stop (E-stop) Tests for Control System

Tests for the existing emergency stop functionality:
  - FSM halt() method from all operational states
  - Signal handler integration (SIGTERM)
  - FSM halt callback (on_halt)
  - Halt transitions from any state to HALTED
  - Integration with control.py signal handler
"""

import signal
import sys
from unittest.mock import MagicMock, patch, call

import pytest

from control.fsm import FSM


# ============================================================================
# Fixtures
# ============================================================================

@pytest.fixture
def fsm():
    """Fresh FSM instance."""
    return FSM()


# ============================================================================
# FSM Halt Transition Tests
# ============================================================================

class TestFSMHaltTransition:
    """Test FSM halt() transition mechanism."""

    def test_fsm_halt_method_exists(self, fsm):
        """FSM has halt() method."""
        assert hasattr(fsm, "halt")
        assert callable(fsm.halt)

    def test_fsm_halt_transitions_to_halted(self, fsm):
        """halt() transitions FSM to HALTED state."""
        assert fsm.state == "IDLE"
        fsm.halt()
        assert fsm.state == "HALTED"

    def test_fsm_on_halt_callback_exists(self, fsm):
        """FSM has on_halt() callback method."""
        assert hasattr(fsm, "on_halt")
        assert callable(fsm.on_halt)

    def test_fsm_halted_state_exists(self, fsm):
        """HALTED state is defined in FSM."""
        assert "HALTED" in fsm.machine.states
        fsm.halt()
        assert fsm.state == "HALTED"


# ============================================================================
# FSM Halt Behavior from All States
# ============================================================================

class TestFSMHaltFromAllStates:
    """Test FSM halt() can be called from any state and transitions to HALTED."""

    def test_halt_from_idle(self, fsm):
        """Halt from IDLE state."""
        assert fsm.state == "IDLE"
        fsm.halt()
        assert fsm.state == "HALTED"

    def test_halt_from_home(self, fsm):
        """Halt from HOME state."""
        fsm.start()
        assert fsm.state == "HOME"
        fsm.halt()
        assert fsm.state == "HALTED"

    def test_halt_from_perceive(self, fsm):
        """Halt from PERCEIVE state."""
        fsm.start()
        fsm.home_ok()
        assert fsm.state == "PERCEIVE"
        fsm.halt()
        assert fsm.state == "HALTED"

    def test_halt_from_plan(self, fsm):
        """Halt from PLAN state."""
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        assert fsm.state == "PLAN"
        fsm.halt()
        assert fsm.state == "HALTED"

    def test_halt_from_move(self, fsm):
        """Halt from MOVE state."""
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.plan_ok()
        assert fsm.state == "MOVE"
        fsm.halt()
        assert fsm.state == "HALTED"

    def test_halt_from_cut(self, fsm):
        """Halt from CUT state."""
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.plan_ok()
        fsm.move_ok()
        assert fsm.state == "CUT"
        fsm.halt()
        assert fsm.state == "HALTED"

    def test_halt_from_return(self, fsm):
        """Halt from RETURN state."""
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.plan_ok()
        fsm.move_ok()
        fsm.cut_ok()
        assert fsm.state == "RETURN"
        fsm.halt()
        assert fsm.state == "HALTED"

    def test_halt_idempotent_already_halted(self, fsm):
        """halt() is idempotent: calling from HALTED stays HALTED."""
        fsm.halt()
        assert fsm.state == "HALTED"
        fsm.halt()
        assert fsm.state == "HALTED"


# ============================================================================
# FSM Halt from All States
# ============================================================================

class TestFSMHaltFromAllStates:
    """Test FSM can halt from any operational state."""

    def test_halt_from_idle(self, fsm):
        """halt() from IDLE transitions to HALTED."""
        assert fsm.state == "IDLE"
        fsm.halt()
        assert fsm.state == "HALTED"

    def test_halt_from_home(self, fsm):
        """halt() from HOME transitions to HALTED."""
        fsm.start()
        assert fsm.state == "HOME"
        fsm.halt()
        assert fsm.state == "HALTED"

    def test_halt_from_perceive(self, fsm):
        """halt() from PERCEIVE transitions to HALTED."""
        fsm.start()
        fsm.home_ok()
        assert fsm.state == "PERCEIVE"
        fsm.halt()
        assert fsm.state == "HALTED"

    def test_halt_from_plan(self, fsm):
        """halt() from PLAN transitions to HALTED."""
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        assert fsm.state == "PLAN"
        fsm.halt()
        assert fsm.state == "HALTED"

    def test_halt_from_move(self, fsm):
        """halt() from MOVE transitions to HALTED."""
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.plan_ok()
        assert fsm.state == "MOVE"
        fsm.halt()
        assert fsm.state == "HALTED"

    def test_halt_from_cut(self, fsm):
        """halt() from CUT transitions to HALTED."""
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.plan_ok()
        fsm.move_ok()
        assert fsm.state == "CUT"
        fsm.halt()
        assert fsm.state == "HALTED"

    def test_halt_from_return(self, fsm):
        """halt() from RETURN transitions to HALTED."""
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.plan_ok()
        fsm.move_ok()
        fsm.cut_ok()
        assert fsm.state == "RETURN"
        fsm.halt()
        assert fsm.state == "HALTED"

    def test_halt_idempotent_already_halted(self, fsm):
        """halt() when already HALTED stays HALTED."""
        fsm.halt()
        assert fsm.state == "HALTED"
        fsm.halt()
        assert fsm.state == "HALTED"


# ============================================================================
# Signal Handler Integration (control.py)
# ============================================================================

class TestSignalHandlerIntegration:
    """Test signal handler integration from control.py."""

    def test_handle_emergency_stop_function_exists(self):
        """handle_emergency_stop function exists in control module."""
        import control.control as control_module
        assert hasattr(control_module, "handle_emergency_stop")
        assert callable(control_module.handle_emergency_stop)

    def test_handle_emergency_stop_calls_halt(self):
        """handle_emergency_stop() calls fsm.halt()."""
        from control.control import handle_emergency_stop
        
        # Create a mock FSM to verify halt() is called
        mock_fsm = MagicMock()
        
        # Patch the fsm in control module
        with patch("control.control.fsm", mock_fsm):
            handle_emergency_stop()
            mock_fsm.halt.assert_called_once()

    def test_signal_handler_available_on_posix(self):
        """Signal handlers available on POSIX systems."""
        if sys.platform == "win32":
            pytest.skip("Signal handlers not available on Windows")

        # Verify SIGTERM signal is available
        assert hasattr(signal, "SIGTERM")
        assert isinstance(signal.SIGTERM, int)

    def test_fsm_instance_created_in_control_module(self):
        """FSM instance is created at module level in control.py."""
        import control.control as control_module
        assert hasattr(control_module, "fsm")
        assert control_module.fsm is not None


# ============================================================================
# FSM on_halt Callback Tests
# ============================================================================

class TestFSMOnHaltCallback:
    """Test FSM on_halt() callback is invoked during halt transition."""

    def test_on_halt_called_during_transition(self, fsm):
        """on_halt() is called when halt transition occurs."""
        fsm.on_halt = MagicMock()

        fsm.halt()

        fsm.on_halt.assert_called_once()
        assert fsm.state == "HALTED"

    def test_on_halt_called_from_any_state(self):
        """on_halt() is called regardless of source state."""
        states_to_test = [
            ("IDLE", []),
            ("HOME", ["start"]),
            ("PERCEIVE", ["start", "home_ok"]),
            ("PLAN", ["start", "home_ok", "target_ok"]),
            ("MOVE", ["start", "home_ok", "target_ok", "plan_ok"]),
        ]

        for state_name, transitions in states_to_test:
            fsm = FSM()
            fsm.on_halt = MagicMock()

            for transition in transitions:
                getattr(fsm, transition)()

            assert fsm.state == state_name
            fsm.halt()

            fsm.on_halt.assert_called_once()
            assert fsm.state == "HALTED"

    def test_on_halt_can_log_telemetry(self, fsm):
        """on_halt() can be used to log halt events."""
        telemetry_log = []

        def custom_on_halt():
            telemetry_log.append({"event": "halt", "state": fsm.state})

        fsm.on_halt = custom_on_halt

        fsm.start()
        fsm.home_ok()
        fsm.halt()

        assert len(telemetry_log) == 1
        assert telemetry_log[0]["event"] == "halt"
        assert fsm.state == "HALTED"


# ============================================================================
# Halt vs. Fault Distinction
# ============================================================================

class TestHaltVsFault:
    """Test distinction between halt (emergency stop) and fault (error)."""

    def test_halt_vs_fault_separate_states(self, fsm):
        """HALTED and FAULT are separate, distinct states."""
        assert fsm.state == "IDLE"

        # Halt -> HALTED
        fsm.halt()
        assert fsm.state == "HALTED"

        # Reset FSM
        fsm = FSM()

        # Fault -> FAULT
        fsm.faulted(code="IK_FAIL", detail="Singularity detected")
        assert fsm.state == "FAULT"

    def test_halt_produces_halted_not_fault(self, fsm):
        """halt() produces HALTED state, not FAULT."""
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()

        fsm.halt()

        assert fsm.state == "HALTED"
        assert fsm.fault is None

    def test_fault_stores_error_info(self, fsm):
        """FAULT state captures error details."""
        fsm.faulted(code="MOTOR_ERROR", detail="Temperature exceeded 80°C")
        assert fsm.state == "FAULT"
        assert fsm.fault is not None
        assert fsm.fault.code == "MOTOR_ERROR"
        assert fsm.fault.detail == "Temperature exceeded 80°C"

    def test_halt_does_not_set_fault_info(self, fsm):
        """halt() does not set fault info."""
        fsm.start()
        fsm.home_ok()
        fsm.halt()

        assert fsm.state == "HALTED"
        assert fsm.fault is None


# ============================================================================
# Control Loop Integration Pattern
# ============================================================================

class TestControlLoopPattern:
    """Test recommended control loop patterns with halt."""

    def test_halt_in_control_loop(self, fsm):
        """Simulate halt() being called in a control loop."""
        fsm.start()
        fsm.home_ok()
        fsm.target_ok()
        fsm.plan_ok()

        # Simulate loop detecting need to halt
        assert fsm.state == "MOVE"
        fsm.halt()

        assert fsm.state == "HALTED"

    def test_halt_during_harvest_cycle(self, fsm):
        """Test halt() can interrupt a harvest cycle at any point."""
        fsm.start()
        assert fsm.state == "HOME"

        fsm.home_ok()
        assert fsm.state == "PERCEIVE"

        fsm.target_ok()
        assert fsm.state == "PLAN"

        fsm.plan_ok()
        assert fsm.state == "MOVE"

        # Emergency: halt mid-move
        fsm.halt()
        assert fsm.state == "HALTED"


# ============================================================================
# Halt Transition Configuration
# ============================================================================

class TestHaltTransitionConfig:
    """Test FSM halt transition is configured correctly."""

    def test_halt_transition_from_any_state(self, fsm):
        """halt() transition is configured as 'from any state' ('*')."""
        # This is a meta-test verifying the transition rules
        # Halt should work from all states
        assert fsm.state == "IDLE"
        fsm.halt()
        assert fsm.state == "HALTED"

    def test_halt_transition_before_callback(self, fsm):
        """halt() transition has before='on_halt' callback."""
        # Verify the callback is called before transition completes
        callback_called = False

        def tracking_on_halt():
            nonlocal callback_called
            callback_called = True
            assert fsm.state == "IDLE"  # State before transition

        fsm.on_halt = tracking_on_halt
        fsm.halt()

        assert callback_called
        assert fsm.state == "HALTED"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
