"""Tests for the streamed arm command watchdog state machine."""

import pytest
from piper_control_ros2 import command_watchdog


def test_maps_receive_timestamp_to_monotonic_clock() -> None:
  assert command_watchdog.received_monotonic_time(
      10_800_000_000,
      monotonic_now=5.0,
      system_now_ns=11_000_000_000,
  ) == pytest.approx(4.8)


@pytest.mark.parametrize("received_timestamp", [None, 0, -1, "invalid"])
def test_invalid_receive_timestamp_uses_current_time(
    received_timestamp: object,
) -> None:
  assert (
      command_watchdog.received_monotonic_time(
          received_timestamp,
          monotonic_now=5.0,
          system_now_ns=11_000_000_000,
      )
      == 5.0
  )


def test_rejects_negative_timeout() -> None:
  with pytest.raises(ValueError, match="non-negative"):
    command_watchdog.CommandWatchdog(-0.1)


def test_disabled_watchdog_never_trips() -> None:
  watchdog = command_watchdog.CommandWatchdog(0.0)

  assert watchdog.accept_command(now=50.0)
  assert not watchdog.check(now=100.0)


def test_enabled_watchdog_rejects_commands_until_armed() -> None:
  watchdog = command_watchdog.CommandWatchdog(0.25)

  assert not watchdog.accept_command(now=1.0)
  watchdog.arm(now=1.0)
  assert watchdog.accept_command(now=1.1)
  watchdog.disarm()
  assert not watchdog.accept_command(now=1.2)


def test_fresh_commands_extend_deadline() -> None:
  watchdog = command_watchdog.CommandWatchdog(0.25)
  watchdog.arm(now=1.0)

  assert watchdog.accept_command(now=1.2)
  assert not watchdog.check(now=1.4)
  assert watchdog.check(now=1.5)


def test_late_command_trips_instead_of_extending_deadline() -> None:
  watchdog = command_watchdog.CommandWatchdog(0.25)
  watchdog.arm(now=1.0)

  assert not watchdog.accept_command(now=1.3)
  assert watchdog.tripped


def test_trip_latches_until_rearmed() -> None:
  watchdog = command_watchdog.CommandWatchdog(0.25)
  watchdog.arm(now=1.0)

  assert watchdog.check(now=1.3)
  assert not watchdog.accept_command(now=1.31)
  assert not watchdog.check(now=2.0)
  watchdog.restart(now=2.0)
  assert not watchdog.accept_command(now=2.1)

  watchdog.arm(now=2.0)
  assert watchdog.accept_command(now=2.1)
  assert not watchdog.tripped


def test_disarm_stops_monitoring() -> None:
  watchdog = command_watchdog.CommandWatchdog(0.25)
  watchdog.arm(now=1.0)
  watchdog.disarm()

  assert not watchdog.check(now=2.0)


def test_restart_refreshes_only_an_active_stream() -> None:
  watchdog = command_watchdog.CommandWatchdog(0.25)
  watchdog.arm(now=1.0)
  watchdog.restart(now=1.2)

  assert not watchdog.check(now=1.4)
  assert watchdog.check(now=1.5)
