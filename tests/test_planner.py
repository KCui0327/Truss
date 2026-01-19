"""
Tests for the Planner module (FSM PLAN state integration).

Tests that the planner correctly:
- Calls IK solver
- Validates joint limits
- Generates trajectories
- Handles errors gracefully
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

from control.planner import Planner, PlanResult
from kinematics.robotModel import RobotModel


@pytest.fixture
def robot_model():
    """Standard 6-DOF robot model."""
    return RobotModel(
        a=[25, 315, 35, 0, 0, -296.23],
        alpha=[math.pi / 2, 0, math.pi / 2, -math.pi / 2, math.pi / 2, 0],
        d=[400, 0, 0, 365, 0, 161.44],
        theta=[0, 0, 0, 0, 0, 0],
        joint_limits=[[-2.094, 2.094]] * 6,
    )


class TestPlannerInitialization:
    def test_planner_creates_with_robot_model(self, robot_model):
        planner = Planner(robot_model)
        assert planner.robot_model is robot_model
        assert planner.ik_backend is None  # Lazy-loaded


class TestPlannerTrajectoryGeneration:
    def test_planner_generates_trajectory(self, robot_model):
        planner = Planner(robot_model)

        with patch.object(planner, "_get_ik_backend") as mock_backend_getter:
            mock_backend = MagicMock()
            mock_backend_getter.return_value = mock_backend
            mock_backend.inverseKinematics.return_value = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

            target_pose = np.eye(4)
            target_pose[0, 3] = 500
            target_pose[1, 3] = 200
            target_pose[2, 3] = 300

            result = planner.plan_to_pose(target_pose, n_steps=10)

        assert result.success
        assert result.joint_trajectory is not None
        assert result.joint_trajectory.shape == (10, 6)

    def test_trajectory_starts_at_zero(self, robot_model):
        planner = Planner(robot_model)

        with patch.object(planner, "_get_ik_backend") as mock_backend_getter:
            mock_backend = MagicMock()
            mock_backend_getter.return_value = mock_backend
            q_target = np.array([0.5, 1.0, 1.5, 0.2, 0.3, 0.4])
            mock_backend.inverseKinematics.return_value = q_target

            target_pose = np.eye(4)
            result = planner.plan_to_pose(target_pose, n_steps=5)

        assert np.allclose(result.joint_trajectory[0], 0)

    def test_trajectory_ends_at_target(self, robot_model):
        planner = Planner(robot_model)

        with patch.object(planner, "_get_ik_backend") as mock_backend_getter:
            mock_backend = MagicMock()
            mock_backend_getter.return_value = mock_backend
            q_target = np.array([0.5, 1.0, 1.5, 0.2, 0.3, 0.4])
            mock_backend.inverseKinematics.return_value = q_target

            target_pose = np.eye(4)
            result = planner.plan_to_pose(target_pose, n_steps=5)

        assert np.allclose(result.joint_trajectory[-1], q_target)

    def test_trajectory_interpolates_linearly(self, robot_model):
        planner = Planner(robot_model)

        with patch.object(planner, "_get_ik_backend") as mock_backend_getter:
            mock_backend = MagicMock()
            mock_backend_getter.return_value = mock_backend
            q_target = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
            mock_backend.inverseKinematics.return_value = q_target

            target_pose = np.eye(4)
            result = planner.plan_to_pose(target_pose, n_steps=11)

        assert np.allclose(result.joint_trajectory[5], 0.5)


class TestPlannerInputValidation:
    def test_planner_rejects_invalid_pose_shape(self, robot_model):
        planner = Planner(robot_model)

        invalid_pose = np.eye(3)
        result = planner.plan_to_pose(invalid_pose)

        assert not result.success
        assert "Invalid pose shape" in result.error_msg

    def test_planner_rejects_wrong_pose_dimensions(self, robot_model):
        planner = Planner(robot_model)

        invalid_pose = np.zeros((4, 3))
        result = planner.plan_to_pose(invalid_pose)

        assert not result.success
        assert "Invalid pose shape" in result.error_msg


class TestPlannerJointLimitValidation:
    def test_planner_detects_joint_limit_violations(self, robot_model):
        planner = Planner(robot_model)

        with patch.object(planner, "_get_ik_backend") as mock_backend_getter:
            mock_backend = MagicMock()
            mock_backend_getter.return_value = mock_backend
            mock_backend.inverseKinematics.return_value = np.array([3.0, 0.2, 0.3, 0.4, 0.5, 0.6])

            target_pose = np.eye(4)
            result = planner.plan_to_pose(target_pose)

        assert not result.success
        assert "joint limits" in result.error_msg

    def test_planner_accepts_valid_joint_values(self, robot_model):
        planner = Planner(robot_model)

        with patch.object(planner, "_get_ik_backend") as mock_backend_getter:
            mock_backend = MagicMock()
            mock_backend_getter.return_value = mock_backend
            mock_backend.inverseKinematics.return_value = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])

            target_pose = np.eye(4)
            result = planner.plan_to_pose(target_pose)

        assert result.success
        assert result.q_target is not None


class TestPlannerErrorHandling:
    def test_planner_handles_ik_exception(self, robot_model):
        planner = Planner(robot_model)

        with patch.object(planner, "_get_ik_backend") as mock_backend_getter:
            mock_backend = MagicMock()
            mock_backend_getter.return_value = mock_backend
            mock_backend.inverseKinematics.side_effect = RuntimeError("IK solver crashed")

            target_pose = np.eye(4)
            result = planner.plan_to_pose(target_pose)

        assert not result.success
        assert "IK computation failed" in result.error_msg

    def test_planner_handles_backend_initialization_failure(self, robot_model):
        planner = Planner(robot_model)

        with patch.object(planner, "_get_ik_backend") as mock_backend_getter:
            mock_backend_getter.side_effect = RuntimeError("MATLAB engine failed")

            target_pose = np.eye(4)
            result = planner.plan_to_pose(target_pose)

        assert not result.success
        assert "backend initialization failed" in result.error_msg


class TestPlanResultDataClass:
    def test_plan_result_success(self):
        trajectory = np.zeros((10, 6))
        result = PlanResult(success=True, joint_trajectory=trajectory)

        assert result.success
        assert result.joint_trajectory is not None
        assert result.error_msg is None

    def test_plan_result_failure(self):
        result = PlanResult(success=False, error_msg="Test error")

        assert not result.success
        assert result.joint_trajectory is None
        assert result.error_msg == "Test error"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
