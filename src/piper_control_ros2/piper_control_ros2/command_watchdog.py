"""State machine for stopping stale streamed arm commands."""

from __future__ import annotations

import dataclasses


@dataclasses.dataclass
class CommandWatchdog:
  """Latches after an armed command stream stops making progress."""

  timeout_seconds: float
  _last_command_time: float | None = None
  _tripped: bool = False

  def __post_init__(self) -> None:
    if self.timeout_seconds < 0:
      raise ValueError("command watchdog timeout must be non-negative")

  @property
  def enabled(self) -> bool:
    return self.timeout_seconds > 0

  @property
  def tripped(self) -> bool:
    return self._tripped

  def arm(self, now: float) -> None:
    """Start monitoring and clear a prior latched trip."""
    self._last_command_time = now
    self._tripped = False

  def disarm(self) -> None:
    """Stop monitoring while the arm is intentionally inactive."""
    self._last_command_time = None
    self._tripped = False

  def accept_command(self, now: float) -> bool:
    """Record a fresh command unless a timeout remains latched."""
    if self._tripped:
      return False
    if self._last_command_time is not None:
      if self.enabled and now - self._last_command_time > self.timeout_seconds:
        self._tripped = True
        return False
      self._last_command_time = now
    return True

  def check(self, now: float) -> bool:
    """Return true once when the active stream crosses its timeout."""
    if (
        not self.enabled
        or self._last_command_time is None
        or self._tripped
        or now - self._last_command_time <= self.timeout_seconds
    ):
      return False
    self._tripped = True
    return True
