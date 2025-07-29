"""Gets metadata about the piper_ros.piper_control_node ROS 2 node."""

import socket
import subprocess

from piper_control import __git_hash__ as __piper_control_git_hash__
from piper_control import __version__ as __piper_control_version__

from ._version import __version__ as __piper_ros_version__


def get_metadata() -> dict:
  """Get metadata about the piper_ros.piper_control_node node."""
  return {
      "piper_ros_version": get_piper_ros_version(),
      "piper_ros_git_hash": get_piper_ros_git_hash(),
      "piper_control_version": get_piper_control_version(),
      "piper_control_git_hash": get_piper_control_git_hash(),
      "hostname": socket.gethostname(),
  }


def get_piper_ros_version() -> str:
  """Get the version of the piper_ros package."""
  return __piper_ros_version__


def get_piper_ros_git_hash() -> str:
  """Get the git hash of the piper_ros package."""
  try:
    git_hash = (
        subprocess.check_output(["git", "rev-parse", "HEAD"]).decode().strip()
    )
    return git_hash
  except Exception:  # pylint: disable=broad-except
    return "unknown"


def get_piper_control_version() -> str:
  """Get the version of the piper_control package."""
  return __piper_control_version__


def get_piper_control_git_hash() -> str:
  """Get the git hash of the piper_control package."""
  return __piper_control_git_hash__
