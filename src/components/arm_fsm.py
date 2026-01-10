# src/components/arm_fsm.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

from transitions import Machine


@dataclass
class FaultInfo:
    code: str
    detail: str


class ArmFSM:
    """
    Finite State Machine for arm operations.

    Pattern:
      - Controller drives the FSM transitions.
      - FSM tracks state and provides safety transitions (halt/fault).
    """

    states = [
        "IDLE",
        "HOME",
        "PERCEIVE",
        "PLAN",
        "MOVE",
        "CUT",
        "RETURN",
        "HALTED",
        "FAULT",
    ]

    def __init__(self) -> None:
        self.fault: Optional[FaultInfo] = None

        self.machine = Machine(
            model=self,
            states=ArmFSM.states,
            initial="IDLE",
            auto_transitions=False,
            ignore_invalid_triggers=False,
        )

        # Normal flow
        self.machine.add_transition("start", "IDLE", "HOME")
        self.machine.add_transition("home_ok", "HOME", "PERCEIVE")
        self.machine.add_transition("target_ok", "PERCEIVE", "PLAN")
        self.machine.add_transition("plan_ok", "PLAN", "MOVE")
        self.machine.add_transition("move_ok", "MOVE", "CUT")
        self.machine.add_transition("cut_ok", "CUT", "RETURN")
        self.machine.add_transition("return_ok", "RETURN", "PERCEIVE")

        # Recovery / loops
        self.machine.add_transition("no_target", "PERCEIVE", "HOME")
        self.machine.add_transition("plan_fail", "PLAN", "HOME")
        self.machine.add_transition("move_fail", "MOVE", "HOME")

        # Safety
        self.machine.add_transition("halt", "*", "HALTED", before="on_halt")
        self.machine.add_transition("faulted", "*", "FAULT", before="on_faulted")

    def on_halt(self) -> None:
        # Placeholders for telemetry hooks (log reason elsewhere)
        return

    def on_faulted(self, code: str = "unknown", detail: str = "") -> None:
        self.fault = FaultInfo(code=code, detail=detail)
