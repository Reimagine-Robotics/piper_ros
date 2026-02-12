"""Gets metadata about the piper_ros.piper_control_ros2 ROS 2 node."""

import socket
import subprocess

from piper_control import __git_hash__ as __piper_control_git_hash__
from piper_control import __version__ as __piper_control_version__
from piper_control import piper_interface

from ._version import __version__ as __piper_ros_version__


def _resolve_git_hash() -> str:
  try:
    return (
        subprocess.check_output(["git", "rev-parse", "HEAD"]).decode().strip()
    )
  except Exception:  # pylint: disable=broad-except
    return "unknown"


_CACHED_GIT_HASH = _resolve_git_hash()


def get_metadata(
    piper: piper_interface.PiperInterface,
) -> dict:
  """Get metadata about the piper_ros.piper_control_ros2 node."""
  return {
      "piper_ros_version": get_piper_ros_version(),
      "piper_ros_git_hash": get_piper_ros_git_hash(),
      "piper_control_version": get_piper_control_version(),
      "piper_control_git_hash": get_piper_control_git_hash(),
      "hostname": socket.gethostname(),
      **_piper_version_info(piper),
  }


def get_piper_ros_version() -> str:
  """Get the version of the piper_ros package."""
  return __piper_ros_version__


def get_piper_ros_git_hash() -> str:
  """Get the git hash of the piper_ros package."""
  return _CACHED_GIT_HASH


def get_piper_control_version() -> str:
  """Get the version of the piper_control package."""
  return __piper_control_version__


def get_piper_control_git_hash() -> str:
  """Get the git hash of the piper_control package."""
  return __piper_control_git_hash__


def _piper_version_info(
    piper: piper_interface.PiperInterface,
) -> dict:
  """Get version information from the piper_interface."""
  return {
      "piper_interface_version": piper.get_piper_interface_name(),
      "piper_protocol_version": piper.get_piper_protocol_version(),
      "piper_sdk_version": piper.get_piper_sdk_version(),
      "piper_firmware_version": piper.get_piper_firmware_version(),
  }
