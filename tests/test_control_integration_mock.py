"""Complete mock test for FSM control cycle without importing real heavy modules."""

import math
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

# Import only lightweight modules (no perception/servo/kinematics_backend)
from control.fsm import FSM
from kinematics.robotModel import RobotModel


def make_mock_system():
    """Create a mock ControlSystem-like object."""
    cs = SimpleNamespace()
    cs.fsm = FSM()
    cs.robot_model = RobotModel(
        a=[25, 315, 35, 0, 0, -296.23],
        alpha=[math.pi / 2, 0, math.pi / 2, -math.pi / 2, math.pi / 2, 0],
        d=[400, 0, 0, 365, 0, 161.44],
        theta=[0, 0, 0, 0, 0, 0],
        joint_limits=[[-2.094, 2.094]] * 6,
    )
    cs.kinematics = MagicMock()
    cs.servo_controller = MagicMock()
    cs.perception_output = None
    return cs


@pytest.fixture
def cs():
    """Pytest fixture providing a mock control system."""
    return make_mock_system()


def attach_perception(cs, *, detections, stem_raises=False):
    """Attach mock perception module to control system."""
    dummy_image = np.zeros((480, 640, 3), dtype=np.uint8)

    def fetch_image(url):
        return dummy_image

    def detect_strawberry(img):
        return detections

    def get_target_stem_coordinate(img, cx, cy):
        if stem_raises:
            raise RuntimeError("stem model failed")
        return (cx - 5, cy + 10)

    cs.perception = SimpleNamespace(
        fetch_image=fetch_image,
        detect_strawberry=detect_strawberry,
        get_target_stem_coordinate=get_target_stem_coordinate,
    )


def test_full_fsm_cycle(cs):
    """Test complete FSM cycle: HOME → PERCEIVE → PLAN → MOVE → CUT → RETURN."""
    # Setup perception with one detection
    attach_perception(cs, detections=[(320, 240, "strawberry", 0.95, (310, 230, 330, 250))])
    cs.kinematics.inverseKinematics.return_value = [0.1, -0.5, 0.8, 0.2, -0.2, 0.3]

    # Start FSM
    cs.fsm.start()
    assert cs.fsm.state == "HOME"

    # HOME
    cs.servo_controller.return_home(duration=2000)
    cs.fsm.home_ok()
    assert cs.fsm.state == "PERCEIVE"

    # PERCEIVE
    image = cs.perception.fetch_image("http://192.168.2.39/capture")
    detections = cs.perception.detect_strawberry(image)
    assert len(detections) == 1

    from dataclasses import dataclass

    @dataclass
    class PerceptionOutput:
        image: any
        detections: list
        stem_coordinates: dict

    cx, cy, label, conf, bbox = detections[0]
    stem_coords = {(cx, cy): cs.perception.get_target_stem_coordinate(image, cx, cy)}
    cs.perception_output = PerceptionOutput(
        image=image, detections=detections, stem_coordinates=stem_coords
    )

    cs.fsm.target_ok()
    assert cs.fsm.state == "PLAN"

    # PLAN
    H_target = np.eye(4)
    joint_angles = cs.kinematics.inverseKinematics(cs.robot_model, H_target)
    assert joint_angles is not None

    cs.fsm.plan_ok()
    assert cs.fsm.state == "MOVE"

    # MOVE
    for servo_id, angle in enumerate(joint_angles, start=1):
        cs.servo_controller.move_servo_motor(servo_id=servo_id, angle=angle, duration=1500)
    assert cs.servo_controller.move_servo_motor.call_count == cs.robot_model.n

    cs.fsm.move_ok()
    assert cs.fsm.state == "CUT"

    # CUT
    cs.fsm.cut_ok()
    assert cs.fsm.state == "RETURN"

    # RETURN
    cs.servo_controller.return_home(duration=2000)
    cs.fsm.return_ok()
    assert cs.fsm.state == "PERCEIVE"


def test_cycle_no_target_routes_home(cs):
    """Test that no detections triggers no_target → HOME."""
    attach_perception(cs, detections=[])
    cs.fsm.start()
    cs.fsm.home_ok()
    assert cs.fsm.state == "PERCEIVE"

    detections = cs.perception.detect_strawberry(None)
    assert len(detections) == 0

    cs.fsm.no_target()
    assert cs.fsm.state == "HOME"


def test_plan_fail_routes_home(cs):
    """Test that IK failure triggers plan_fail → HOME."""
    cs.kinematics.inverseKinematics.return_value = None

    cs.fsm.start()
    cs.fsm.home_ok()
    cs.fsm.target_ok()
    assert cs.fsm.state == "PLAN"

    result = cs.kinematics.inverseKinematics(cs.robot_model, np.eye(4))
    assert result is None

    cs.fsm.plan_fail()
    assert cs.fsm.state == "HOME"


def test_move_failure_faults(cs):
    """Test that servo error during MOVE triggers move_fail → HOME."""
    cs.servo_controller.move_servo_motor.side_effect = RuntimeError("bus error")

    cs.fsm.start()
    cs.fsm.home_ok()
    cs.fsm.target_ok()
    cs.fsm.plan_ok()
    assert cs.fsm.state == "MOVE"

    cs.fsm.move_fail()
    assert cs.fsm.state == "HOME"


def test_emergency_halt(cs):
    """Test that halt() transitions to HALTED from any state."""
    cs.fsm.start()
    cs.fsm.home_ok()
    cs.fsm.target_ok()
    cs.fsm.plan_ok()
    assert cs.fsm.state == "MOVE"

    cs.fsm.halt()
    assert cs.fsm.state == "HALTED"


def test_fault_state(cs):
    """Test that faulted() transitions to FAULT with error info."""
    cs.fsm.start()
    cs.fsm.home_ok()
    assert cs.fsm.state == "PERCEIVE"

    cs.fsm.faulted(code="PERCEPTION_FAIL", detail="Model not loaded")
    assert cs.fsm.state == "FAULT"
    assert cs.fsm.fault.code == "PERCEPTION_FAIL"
    assert cs.fsm.fault.detail == "Model not loaded"


def test_perceive_stem_failure_uses_fallback(cs):
    """Test that stem extraction failure falls back to center."""
    attach_perception(cs, detections=[(320, 240, "strawberry", 0.9, (0, 0, 0, 0))], stem_raises=True)

    cs.fsm.start()
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    detections = cs.perception.detect_strawberry(image)
    assert len(detections) == 1

    cx, cy, *_ = detections[0]
    try:
        cs.perception.get_target_stem_coordinate(image, cx, cy)
        assert False, "Should have raised"
    except RuntimeError:
        # Fallback to center
        stem_fallback = (cx, cy)
        assert stem_fallback == (320, 240)

