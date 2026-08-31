"""Unwrap the DM-J4310's +/-12.5 rad feedback window into a continuous shaft angle.

The worm gear cannot be back-driven, so a saved park position is adopted
EXACTLY (offset = park - first_wrapped): the motor's own frame resets modulo
2*pi on every power cycle, so modular reconciliation is never safe. Ported
from so101_teleop.py (v7). The window-rounding variant in dm4310.py is
retired on the controller path; autocal/drive still use it until Plan 2.
"""

from __future__ import annotations

from .port import SPAN


class MultiTurnTracker:
    def __init__(self, span: float = SPAN, start_unwrapped: float | None = None):
        self.span = span
        self._offset = 0.0
        self._last: float | None = None
        self._pending = start_unwrapped

    @property
    def seen(self) -> bool:
        return self._last is not None

    def update(self, wrapped: float) -> float:
        if self._last is None:
            self._last = wrapped
            if self._pending is not None:
                self._offset = self._pending - wrapped
                self._pending = None
        else:
            delta = wrapped - self._last
            if delta > self.span / 2:
                self._offset -= self.span
            elif delta < -self.span / 2:
                self._offset += self.span
            self._last = wrapped
        return self.position

    def anchor(self, known_position: float) -> None:
        """Declare the current shaft angle to be `known_position` (homing)."""
        if self._last is None:
            self._pending = known_position
            return
        self._offset = known_position - self._last

    @property
    def position(self) -> float:
        if self._last is None:
            raise RuntimeError("no feedback seen yet")
        return self._last + self._offset

    @property
    def wrapped(self) -> float:
        if self._last is None:
            raise RuntimeError("no feedback seen yet")
        return self._last
