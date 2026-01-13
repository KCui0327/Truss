# src/main.py
from __future__ import annotations

import argparse

from .logs.logging_setup import init, get_logger

from components.arm_fsm import ArmFSM
from components.safety import EmergencyStop
from components.servo_bus import SimServoBus, ServoMoveParams
from components.arm_control import ArmController

from kinematics.robotModel import RobotModel 
from kinematics.matlabBackend import MatlabBackend
import math


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--sim", action="store_true", help="Run in simulation (no hardware)")
    p.add_argument("--once", action="store_true", help="Run one cycle then exit")
    return p.parse_args()


def main() -> None:
    init()
    logger = get_logger(__name__)
    args = parse_args()

    logger.info("Truss FSM starting")

    estop = EmergencyStop()
    fsm = ArmFSM()
    
    robot = RobotModel(
            a=[25, 315, 35, 0, 0, -296.23],
            alpha=[math.pi/2, 0, math.pi/2, -math.pi/2, math.pi/2, 0],
            d=[400, 0, 0, 365, 0, 161.44],
            theta=[0, 0, 0, 0, 0, 0],
            joint_limits=[[-2.094, 2.094]] * 6,
        )
    matlab = MatlabBackend()

    servo_ids = [1, 2, 3, 4, 5]
    q_home = [90.0, 90.0, 90.0, 90.0, 90.0]

    # Sim bus for macOS; replace with real bus on Raspberry Pi.
    bus = SimServoBus(servo_ids, q_home)

    move_params = ServoMoveParams(
        tolerance_deg=2.0,
        poll_interval_s=0.10,
        timeout_s=1.0,
    )

    cfg = ArmConfig(
        servo_ids=servo_ids,
        q_home_deg=q_home,
        spline_steps=8,
        duration_ms=300,
    )

    ctrl = ArmController(fsm=fsm, estop=estop, bus=bus, move_params=move_params, cfg=cfg)

    if args.once:
        ctrl.run_once()
        return

    # If later want continuous operation:
    # while True:
    #     ctrl.run_once()
    #     time.sleep(0.2)


if __name__ == "__main__":
    main()
