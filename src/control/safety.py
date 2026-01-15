import threading
import time
from dataclasses import dataclass
from typing import Optional

@dataclass(frozen=True)
class EStopEvent:
    triggered: bool
    reason: str
    ts: float


class EmergencyStop:
    """
    Latching emergency-stop. Once triggered, remains triggered until reset().
    Thread-safe.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._event = EStopEvent(triggered=False, reason="", ts=0.0)

    def trigger(self, reason: str = "unspecified") -> None:
        with self._lock:
            if self._event.triggered:
                return
            self._event = EStopEvent(triggered=True, reason=reason, ts=time.time())

    def reset(self) -> None:
        with self._lock:
            self._event = EStopEvent(triggered=False, reason="", ts=0.0)

    def is_triggered(self) -> bool:
        with self._lock:
            return self._event.triggered

    def snapshot(self) -> EStopEvent:
        with self._lock:
            return self._event
