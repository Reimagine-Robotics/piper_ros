import pytest

from piper_control_ros2 import command_watchdog


def test_rejects_negative_timeout() -> None:
  with pytest.raises(ValueError, match="non-negative"):
    command_watchdog.CommandWatchdog(-0.1)


def test_disabled_watchdog_never_trips() -> None:
  watchdog = command_watchdog.CommandWatchdog(0.0)
  watchdog.arm(now=0.0)

  assert not watchdog.check(now=100.0)


def test_fresh_commands_extend_deadline() -> None:
  watchdog = command_watchdog.CommandWatchdog(0.25)
  watchdog.arm(now=1.0)

  assert watchdog.accept_command(now=1.2)
  assert not watchdog.check(now=1.4)
  assert watchdog.check(now=1.5)


def test_trip_latches_until_rearmed() -> None:
  watchdog = command_watchdog.CommandWatchdog(0.25)
  watchdog.arm(now=1.0)

  assert watchdog.check(now=1.3)
  assert not watchdog.accept_command(now=1.31)
  assert not watchdog.check(now=2.0)

  watchdog.arm(now=2.0)
  assert watchdog.accept_command(now=2.1)
  assert not watchdog.tripped


def test_disarm_stops_monitoring() -> None:
  watchdog = command_watchdog.CommandWatchdog(0.25)
  watchdog.arm(now=1.0)
  watchdog.disarm()

  assert not watchdog.check(now=2.0)
