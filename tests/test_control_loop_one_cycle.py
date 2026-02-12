import math
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from control.control import ControlSystem
from kinematics.robotModel import RobotModel


def test_run_one_cycle_happy_path_converts_rad_to_deg():
    robot_model = RobotModel(
        a=[25, 315, 35, 0, 0, -296.23],
        alpha=[math.pi / 2, 0, math.pi / 2, -math.pi / 2, math.pi / 2, 0],
        d=[400, 0, 0, 365, 0, 161.44],
        theta=[0, 0, 0, 0, 0, 0],
        joint_limits=[[-2.094, 2.094]] * 6,
    )

    dummy_image = np.zeros((480, 640, 3), dtype=np.uint8)
    detections = [(320, 240, "strawberry", 0.95, (310, 230, 330, 250))]

    perception = SimpleNamespace(
        fetch_image=lambda url: dummy_image,
        detect_strawberry=lambda img: detections,
        get_target_stem_coordinate=lambda img, cx, cy: (cx - 5, cy + 10),
    )

    kinematics = MagicMock()
    q_rad = [0.1, -0.5, 0.8, 0.2, -0.2, 0.3]
    kinematics.inverseKinematics.return_value = q_rad

    servo = MagicMock()

    cs = ControlSystem(
        robot_model=robot_model,
        perception=perception,
        kinematics=kinematics,
        servo_controller=servo,
        register_signals=False,
    )

    cs.run(max_cycles=1)

    # After a full cycle, FSM returns to PERCEIVE
    assert cs.fsm.state == "PERCEIVE"

    # Ensure servo got degrees
    assert servo.move_servo_motor.call_count == robot_model.n
    calls = servo.move_servo_motor.call_args_list
    for i, call in enumerate(calls):
        angle_deg_sent = call.kwargs["angle"]
        assert angle_deg_sent == pytest.approx(math.degrees(q_rad[i]), rel=1e-6)
        assert call.kwargs["servo_id"] == i + 1
