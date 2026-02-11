import logging
import math
import signal
import sys
from dataclasses import dataclass
from typing import Any, Optional

import numpy as np

from control.fsm import FSM
from kinematics.robotModel import RobotModel

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)


@dataclass
class PerceptionOutput:
    image: Any
    detections: list  # List of (cx, cy, label, conf, bbox)
    stem_coordinates: dict  # (cx, cy) -> (sx, sy)


class ControlSystem:
    """
    Main control system integrating FSM with perception, kinematics, and servo control.

    Design rule:
      - Internal units for joint angles are radians.
      - Servo boundary converts radians -> degrees.
      - State methods do NOT change FSM state; run() owns transitions.
    """

    def __init__(
        self,
        *,
        robot_model: Optional[RobotModel] = None,
        perception: Optional[Any] = None,
        kinematics: Optional[Any] = None,
        servo_controller: Optional[Any] = None,
        register_signals: bool = True,
    ):
        self.fsm = FSM()

        self.robot_model: Optional[RobotModel] = robot_model
        self.perception: Optional[Any] = perception
        self.kinematics: Optional[Any] = kinematics
        self.servo_controller: Optional[Any] = servo_controller

        self.perception_output: Optional[PerceptionOutput] = None

        logger.info("Initializing ControlSystem...")

        if self.robot_model is None:
            self._initialize_robot_model()

        # Only create real modules if not injected (keeps tests lightweight)
        if self.perception is None:
            try:
                from perception.perception import Perception  # local import to avoid cv2 during tests
                self.perception = Perception()
                logger.info("✓ Perception module initialized")
            except Exception as e:
                logger.error(f"✗ Failed to initialize Perception: {e}")
                self.perception = None

        if self.kinematics is None:
            try:
                from kinematics.matlabBackend import MatlabBackend  # local import
                self.kinematics = MatlabBackend()
                logger.info("✓ Kinematics (MATLAB) backend initialized")
            except Exception as e:
                logger.error(f"✗ Failed to initialize Kinematics: {e}")
                self.kinematics = None

        if self.servo_controller is None:
            try:
                from servo.servo import ServoController  # local import
                self.servo_controller = ServoController()
                logger.info("✓ Servo controller initialized")
            except Exception as e:
                logger.error(f"✗ Failed to initialize Servo Controller: {e}")
                self.servo_controller = None

        if register_signals:
            signal.signal(signal.SIGTERM, self._handle_emergency_stop)

    # -------------------------
    # Init / safety
    # -------------------------

    def _initialize_robot_model(self) -> None:
        try:
            # Your DH parameters -> RobotModel stores dh_table internally
            self.robot_model = RobotModel(
                a=[25, 315, 35, 0, 0, -296.23],
                alpha=[math.pi / 2, 0, math.pi / 2, -math.pi / 2, math.pi / 2, 0],
                d=[400, 0, 0, 365, 0, 161.44],
                theta=[0, 0, 0, 0, 0, 0],
                joint_limits=[[-2.094, 2.094]] * 6,
            )
            logger.info(f"✓ Robot model initialized with {self.robot_model.n} DOF")
        except Exception as e:
            logger.error(f"✗ Failed to initialize robot model: {e}")
            self.robot_model = None

    def _handle_emergency_stop(self, signum, frame):
        logger.warning("🛑 EMERGENCY STOP triggered!")
        try:
            self.fsm.halt()
        except Exception:
            pass

        if self.servo_controller:
            try:
                self.servo_controller.return_home(duration=1000)
            except Exception as e:
                logger.error(f"Failed to return home during emergency stop: {e}")
        sys.exit(1)

    # -------------------------
    # Helpers
    # -------------------------

    def _rad_to_deg(self, rad: float) -> float:
        return rad * 180.0 / math.pi

    def _validate_joint_angles(self, joint_angles: list) -> None:
        if not self.robot_model:
            raise ValueError("Robot model not initialized")
        if len(joint_angles) != self.robot_model.n:
            raise ValueError(f"Expected {self.robot_model.n} joint angles, got {len(joint_angles)}")

        limits = self.robot_model.joint_limits
        for i, q in enumerate(joint_angles):
            qmin, qmax = float(limits[i][0]), float(limits[i][1])
            if not (qmin <= q <= qmax):
                raise ValueError(f"Joint {i+1} angle {q:.4f} rad out of limits [{qmin:.4f}, {qmax:.4f}]")

    # -------------------------
    # State actions (do NOT touch FSM)
    # -------------------------

    def home(self) -> bool:
        logger.info("→ Entering HOME state")
        if not self.servo_controller:
            logger.error("Servo controller not available")
            return False
        try:
            self.servo_controller.return_home(duration=2000)
            return True
        except Exception as e:
            logger.error(f"✗ Homing failed: {e}")
            return False

    def perceive(self) -> bool:
        logger.info("→ Entering PERCEIVE state")
        if not self.perception:
            logger.error("Perception module not available")
            return False

        try:
            image = self.perception.fetch_image("http://192.168.2.39/capture")
            if image is None:
                logger.warning("Failed to fetch image")
                return False

            detections = self.perception.detect_strawberry(image)
            if not detections:
                logger.info("No strawberries detected")
                return False

            stem_coords = {}
            for (cx, cy, label, conf, bbox) in detections:
                try:
                    stem = self.perception.get_target_stem_coordinate(image, cx, cy)
                    stem_coords[(cx, cy)] = stem
                except Exception as e:
                    logger.warning(f"Stem extraction failed: {e} (falling back to center)")
                    stem_coords[(cx, cy)] = (cx, cy)

            self.perception_output = PerceptionOutput(
                image=image, detections=detections, stem_coordinates=stem_coords
            )
            return True

        except Exception as e:
            logger.error(f"✗ Perception failed: {e}")
            return False

    def plan(self) -> Optional[list]:
        logger.info("→ Entering PLAN state")
        if not self.kinematics or not self.robot_model:
            logger.error("Kinematics or robot model not available")
            return None
        if not self.perception_output or not self.perception_output.detections:
            logger.error("No perception output available")
            return None

        try:
            cx, cy, label, conf, bbox = self.perception_output.detections[0]
            stem_coord = self.perception_output.stem_coordinates.get((cx, cy), (cx, cy))

            # Placeholder pose until calibration is implemented
            H_target = np.eye(4)
            H_target[0, 3] = (stem_coord[0] - 320) * 0.5
            H_target[1, 3] = (stem_coord[1] - 240) * 0.5
            H_target[2, 3] = 200.0

            joint_angles = self.kinematics.inverseKinematics(self.robot_model, H_target)
            if joint_angles is None:
                return None

            return list(joint_angles)  # radians

        except Exception as e:
            logger.error(f"✗ Planning failed: {e}")
            return None

    def move(self, joint_angles: list) -> bool:
        logger.info("→ Entering MOVE state")
        if not self.servo_controller or not self.robot_model:
            logger.error("Servo controller or robot model not available")
            return False

        try:
            self._validate_joint_angles(joint_angles)

            duration_ms = 1500
            for servo_id, q_rad in enumerate(joint_angles, start=1):
                q_deg = self._rad_to_deg(q_rad)
                self.servo_controller.move_servo_motor(
                    servo_id=servo_id,
                    angle=q_deg,  # servo expects degrees
                    duration=duration_ms,
                )
            return True

        except Exception as e:
            logger.error(f"✗ Movement failed: {e}")
            return False

    def cut(self) -> bool:
        logger.info("→ Entering CUT state")
        # TODO: implement actual cutter behavior
        return True

    def return_home(self) -> bool:
        logger.info("→ Entering RETURN state")
        if not self.servo_controller:
            logger.error("Servo controller not available")
            return False
        try:
            self.servo_controller.return_home(duration=2000)
            return True
        except Exception as e:
            logger.error(f"✗ Return home failed: {e}")
            return False

    # -------------------------
    # Main loop (FSM transitions live here)
    # -------------------------

    def run(self, *, max_cycles: Optional[int] = None) -> None:
        logger.info("=" * 60)
        logger.info("Starting ControlSystem main loop")
        logger.info("=" * 60)

        try:
            self.fsm.start()  # IDLE -> HOME
            cycle_count = 0

            while True:
                cycle_count += 1
                logger.info(f"\n{'='*60}\nCYCLE {cycle_count}\n{'='*60}")

                # HOME
                if not self.home():
                    self.fsm.faulted(code="HOME_FAIL", detail="home() returned False")
                    break
                self.fsm.home_ok()

                # PERCEIVE
                if not self.perceive():
                    self.fsm.no_target()
                    if max_cycles is not None and cycle_count >= max_cycles:
                        break
                    continue
                self.fsm.target_ok()

                # PLAN
                joint_angles = self.plan()
                if joint_angles is None:
                    self.fsm.plan_fail()
                    if max_cycles is not None and cycle_count >= max_cycles:
                        break
                    continue
                self.fsm.plan_ok()

                # MOVE
                if not self.move(joint_angles):
                    self.fsm.move_fail()
                    if max_cycles is not None and cycle_count >= max_cycles:
                        break
                    continue
                self.fsm.move_ok()

                # CUT
                if not self.cut():
                    logger.warning("Cut failed but continuing")
                self.fsm.cut_ok()

                # RETURN
                if not self.return_home():
                    self.fsm.faulted(code="RETURN_FAIL", detail="return_home() returned False")
                    break
                self.fsm.return_ok()

                if max_cycles is not None and cycle_count >= max_cycles:
                    logger.info("Reached max_cycles; exiting run loop.")
                    break

        except KeyboardInterrupt:
            logger.info("Keyboard interrupt; halting")
            self.fsm.halt()
        except Exception as e:
            logger.error(f"Unexpected error in control loop: {e}", exc_info=True)
            self.fsm.faulted(code="UNKNOWN", detail=str(e))


if __name__ == "__main__":
    ControlSystem().run()
