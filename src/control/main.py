# src/main.py
from __future__ import annotations

import argparse

from logs.logging_setup import init, get_logger
from components import EmergencyStop, FSM, ArmController, SimServoBus, ServoMoveParams
from components.arm_control import ArmConfig


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
    fsm = FSM()

    servo_ids = [1, 2, 3, 4, 5]
    q_home = [90.0, 90.0, 90.0, 90.0, 90.0]

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
