# src/components/servo_bus.py
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, List, Protocol, Tuple


@dataclass(frozen=True)
class ServoMoveParams:
    tolerance_deg: float = 2.0
    poll_interval_s: float = 0.10
    timeout_s: float = 2.0

    # Retry backoff sequence (seconds) after a timeout
    retry_backoffs_s: Tuple[float, float, float] = (0.5, 1.0, 0.0)  # 3rd means "give up"


class ServoBus(Protocol):
    """
    Abstract interface for the servo control layer.
    Implementations:
      - SimServoBus (macOS/CI)
      - LobotServoBus (Raspberry Pi / real hardware)
    """

    def move(self, servo_id: int, angle_deg: float, duration_ms: int = 300) -> None: ...
    def read_position(self, servo_id: int) -> float: ...
    def disable_all(self) -> None: ...
    def enable_all(self) -> None: ...


class SimServoBus:
    """
    Lightweight simulation: servo positions jump immediately.
    Useful for FSM + controller testing on non-Pi environments.
    """

    def __init__(self, servo_ids: List[int], initial_angles: List[float] | None = None) -> None:
        self._pos: Dict[int, float] = {}
        if initial_angles is None:
            initial_angles = [90.0] * len(servo_ids)
        if len(initial_angles) != len(servo_ids):
            raise ValueError("initial_angles must match servo_ids length")
        for sid, a in zip(servo_ids, initial_angles):
            self._pos[sid] = float(a)
        self._enabled = True

    def move(self, servo_id: int, angle_deg: float, duration_ms: int = 300) -> None:
        if not self._enabled:
            return
        self._pos[int(servo_id)] = float(angle_deg)

    def read_position(self, servo_id: int) -> float:
        return float(self._pos[int(servo_id)])

    def disable_all(self) -> None:
        self._enabled = False

    def enable_all(self) -> None:
        self._enabled = True


def move_with_polling(
    bus: ServoBus,
    servo_id: int,
    target_deg: float,
    params: ServoMoveParams,
    duration_ms: int = 300,
) -> bool:
    """
    Command a move and poll until within tolerance or timeout.
    Returns True if reached, False if timed out.
    """
    bus.move(servo_id, target_deg, duration_ms=duration_ms)
    t0 = time.time()
    while True:
        cur = bus.read_position(servo_id)
        if abs(cur - target_deg) <= params.tolerance_deg:
            return True
        if time.time() - t0 >= params.timeout_s:
            return False
        time.sleep(params.poll_interval_s)


def move_with_retry_backoff(
    bus: ServoBus,
    servo_id: int,
    target_deg: float,
    params: ServoMoveParams,
    duration_ms: int = 300,
) -> bool:
    """
    1st move attempt + up to 3 retries with backoff.
      - retry1: wait 500ms
      - retry2: wait 1s
      - retry3: give up (caller returns home)
    """
    if move_with_polling(bus, servo_id, target_deg, params, duration_ms=duration_ms):
        return True

    for backoff in params.retry_backoffs_s:
        if backoff <= 0.0:
            return False
        time.sleep(backoff)
        if move_with_polling(bus, servo_id, target_deg, params, duration_ms=duration_ms):
            return True

    return False
