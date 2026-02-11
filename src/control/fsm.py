from dataclasses import dataclass
from typing import Optional

from transitions import Machine


@dataclass
class FaultInfo:
    code: str
    detail: str


class FSM:
    """
    Finite State Machine for arm operations.

    Controller-driven pattern:
      - ControlSystem.run() triggers transitions.
      - State methods (home/perceive/plan/move/...) DO NOT mutate the FSM.
      - Safety triggers (halt/faulted) are always available.
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
            states=FSM.states,
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

        # Recovery
        self.machine.add_transition("no_target", "PERCEIVE", "HOME")
        self.machine.add_transition("plan_fail", "PLAN", "HOME")
        self.machine.add_transition("move_fail", "MOVE", "HOME")

        # Safety
        self.machine.add_transition("halt", "*", "HALTED", before="on_halt")
        self.machine.add_transition("faulted", "*", "FAULT", before="on_faulted")

    def on_halt(self) -> None:
        return

    def on_faulted(self, code: str = "unknown", detail: str = "") -> None:
        self.fault = FaultInfo(code=code, detail=detail)
