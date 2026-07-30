"""State machine for stopping stale streamed arm commands."""

from __future__ import annotations

import dataclasses


def received_monotonic_time(
    received_timestamp_ns: object,
    *,
    monotonic_now: float,
    system_now_ns: int,
) -> float:
  """Map a local DDS receive timestamp onto the monotonic clock."""
  if not isinstance(received_timestamp_ns, int) or received_timestamp_ns <= 0:
    return monotonic_now

  age_seconds = (system_now_ns - received_timestamp_ns) / 1e9
  if age_seconds <= 0:
    return monotonic_now
  return monotonic_now - age_seconds


@dataclasses.dataclass
class CommandWatchdog:
  """Latches after an armed command stream stops making progress."""

  timeout_seconds: float
  _last_command_time: float | None = None
  _armed: bool = False
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
    self._armed = True
    self._last_command_time = now
    self._tripped = False

  def disarm(self) -> None:
    """Stop monitoring while the arm is intentionally inactive."""
    self._armed = False
    self._last_command_time = None
    self._tripped = False

  def restart(self, now: float) -> None:
    """Refresh an active stream without clearing a timeout interlock."""
    if self._armed and not self._tripped:
      self._last_command_time = now

  def accept_command(self, now: float) -> bool:
    """Record a fresh command unless a timeout remains latched."""
    if not self.enabled:
      return True
    if not self._armed or self._tripped:
      return False
    assert self._last_command_time is not None
    if now - self._last_command_time > self.timeout_seconds:
      self._armed = False
      self._tripped = True
      return False
    self._last_command_time = now
    return True

  def check(self, now: float) -> bool:
    """Return true once when the active stream crosses its timeout."""
    if (
        not self.enabled
        or not self._armed
        or self._last_command_time is None
        or self._tripped
        or now - self._last_command_time <= self.timeout_seconds
    ):
      return False
    self._armed = False
    self._tripped = True
    return True
