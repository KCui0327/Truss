# src/components/arm_control.py
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

from .arm_fsm import ArmFSM
from .ik import IKSolver
from .motion import interpolate_joint_path
from .safety import EmergencyStop
from .servo_bus import ServoBus, ServoMoveParams, move_with_retry_backoff


@dataclass(frozen=True)
class ArmConfig:
    servo_ids: List[int]
    q_home_deg: List[float]
    spline_steps: int = 10
    duration_ms: int = 300


class ArmController:
    """
    Orchestrates the arm cycle using an FSM and a servo bus.

    Current structure:
      - HOME motion (real)
      - PERCEIVE/PLAN currently stubbed to demonstrate integration
      - MOVE uses joint path interpolation + servo polling
      - Safety: EStop can halt at any time
    """

    def __init__(
        self,
        fsm: ArmFSM,
        estop: EmergencyStop,
        bus: ServoBus,
        move_params: ServoMoveParams,
        cfg: ArmConfig,
        ik: Optional[IKSolver] = None,
    ) -> None:
        self.fsm = fsm
        self.estop = estop
        self.bus = bus
        self.move_params = move_params
        self.cfg = cfg
        self.ik = ik or IKSolver()

    def run_once(self) -> None:
        self.fsm.start()  # IDLE -> HOME

        ok = self._goto(self.cfg.q_home_deg)
        if not ok or self.fsm.state == "HALTED":
            return

        self.fsm.home_ok()  # HOME -> PERCEIVE

        target = self._perceive_stub()
        if target is None:
            self.fsm.no_target()
            return

        self.fsm.target_ok()  # PERCEIVE -> PLAN

        ikr = self.ik.solve_to_pose(**target)
        if not ikr.success:
            self.fsm.plan_fail()
            return

        self.fsm.plan_ok()  # PLAN -> MOVE

        ok = self._goto(ikr.q_deg)
        if not ok or self.fsm.state == "HALTED":
            return

        self.fsm.move_ok()  # MOVE -> CUT

        if not self._cut_stub():
            self.fsm.faulted("cut_fail", "Cut operation failed")
            return

        self.fsm.cut_ok()  # CUT -> RETURN

        ok = self._goto(self.cfg.q_home_deg)
        if not ok or self.fsm.state == "HALTED":
            return

        self.fsm.return_ok()

    def _perceive_stub(self):
        # Replace this with real perception output:
        # return dict(x=?, y=?, z=?, roll=?, pitch=?)
        return {"x": 0.10, "y": 0.00, "z": 0.15, "roll": 0.0, "pitch": 0.0}

    def _cut_stub(self) -> bool:
        # Replace with end-effector command(s)
        return True

    def _goto(self, q_goal: List[float]) -> bool:
        if self.estop.is_triggered():
            self.bus.disable_all()
            self.fsm.halt()
            return False

        q_cur = [self.bus.read_position(sid) for sid in self.cfg.servo_ids]
        path = interpolate_joint_path(q_cur, q_goal, steps=self.cfg.spline_steps)

        for waypoint in path:
            if self.estop.is_triggered():
                self.bus.disable_all()
                self.fsm.halt()
                return False

            # Sequential motor actuation (safe, slower)
            for sid, a in zip(self.cfg.servo_ids, waypoint):
                if self.estop.is_triggered():
                    self.bus.disable_all()
                    self.fsm.halt()
                    return False

                ok = move_with_retry_backoff(
                    self.bus,
                    servo_id=sid,
                    target_deg=a,
                    params=self.move_params,
                    duration_ms=self.cfg.duration_ms,
                )
                if not ok:
                    # Caller may return home. Record fault too.
                    self.fsm.faulted("servo_timeout", f"Servo {sid} failed to reach {a:.1f} deg")
                    return False

        return True
