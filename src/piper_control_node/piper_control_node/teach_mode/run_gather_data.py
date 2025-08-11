"""
Grav Comp model data gathering script.

This script moves the arm to various joint configurations, and at each pose
it applies a torque (using a PD-controller) to find a stable configuration
where the torque exactly counteracts gravity. This is recorded as a data point.
Each data point consists of a joint angle configuration and a torque command
per joint that compensates for gravity.

The generated set of grav comp torques is later used in teach_mode.py to fit a
gravity model that can be used in any joint configuration.

Usage examples:

Gather data for single upright arm:
  $ python run_gather_data.py --mode=collect_data --arm_orientation=upright

Dump list of poses to visit:
  $ python run_gather_data.py --mode=collect_poses \
    --can_port=can1 \
    --arm_orientation=left \
    --poses_path=/tmp/left_poses.json

Gather data for left/right arm:
  $ python run_gather_data.py --mode=collect_data \
    --can_port=can1 \
    --arm_orientation=left \
    --poses_path=/tmp/left_poses.json
"""

import dataclasses
import json
import time
from typing import Sequence

import numpy as np
from absl import app, flags
from piper_control import piper_connect, piper_control, piper_init, piper_interface

_OUT_PATH = flags.DEFINE_string(
    "out_path",
    "/tmp/grav_comp_samples.json",
    "The output path for the sampled gravity compensation torques.",
)

_CAN_PORT = flags.DEFINE_string(
    "can_port",
    "",
    "The CAN port to use. Defaults to the first detected port.",
)

_MODE = flags.DEFINE_enum(
    "mode",
    "collect_data",
    ["collect_poses", "collect_data"],
    "Mode to run: 'collect_poses' for interactive collection of poses to visit,"
    "'collect_data' for actual gravity compensation data collection.",
)

_POSES_PATH = flags.DEFINE_string(
    "poses_path",
    "/tmp/sample_poses.json",
    "Path to load/save the sample poses JSON file.",
)

_ARM_ORIENTATION = flags.DEFINE_enum(
    "arm_orientation",
    "upright",
    ["upright", "left", "right"],
    "Arm mounting orientation: upright, left, or right.",
)


@dataclasses.dataclass
class GravityTorqueSample:
  """A gravity torque sample for a single joint."""

  joint_configuration: np.ndarray
  joint_idx: int
  torque: float


class GravityTorqueSampler:
  """Torque sampler that samples grav comp torques for a joint configuration."""

  def __init__(
      self,
      robot: piper_interface.PiperInterface,
      controller: piper_control.MitJointPositionController,
      target_joint_angles: np.ndarray,
      p_gains: np.ndarray = np.ones(6),
      d_gains: np.ndarray = np.ones(6),
  ):
    self._robot = robot
    self._controller = controller

    min_joints = np.array(piper_interface.JOINT_LIMITS_RAD["min"]) + 0.2
    max_joints = np.array(piper_interface.JOINT_LIMITS_RAD["max"]) - 0.2
    self._target_joint_angles = np.clip(
        target_joint_angles, min_joints, max_joints
    )

    self._p_gains = p_gains
    self._d_gains = d_gains

  def sample(self) -> Sequence[GravityTorqueSample]:
    print(f"Moving to target: {self._target_joint_angles}")
    self._move_to_target()
    time.sleep(1.0)

    print("Sampling stable gravity torques ...")
    return self._hold_target_with_torque()

  def _move_to_target(self) -> None:
    move_steps = 600
    p_gains = np.geomspace(0.5, 5.0, move_steps)

    for i in range(move_steps):
      joint_positions = np.array(self._robot.get_joint_positions())
      if np.all((joint_positions - self._target_joint_angles) < 0.05):
        print("Finished early")
        break

      kp_gains = [p_gains[i]] * 6
      self._controller.command_joints(
          list(self._target_joint_angles), kp_gains=kp_gains
      )
      time.sleep(0.005)

  def _hold_target_with_torque(self) -> Sequence[GravityTorqueSample]:
    stable_torques = []
    stable_joint_configs = []

    smooth_torques = None
    for _ in range(2000):
      joint_positions = self._robot.get_joint_positions()
      joint_velocities = self._robot.get_joint_velocities()

      error = joint_positions - self._target_joint_angles

      p_terms = -1.0 * error * self._p_gains
      d_terms = -1.0 * np.array(joint_velocities) * self._d_gains
      torques = p_terms + d_terms

      if smooth_torques is None:
        smooth_torques = torques
      else:
        smooth_torques = 0.3 * smooth_torques + 0.7 * torques

      torques = smooth_torques

      if np.all(np.abs(joint_velocities) < 0.01):
        if np.any(joint_positions <= piper_interface.JOINT_LIMITS_RAD["min"]):
          print("MIN JOINT LIMITS HIT!")
        elif np.any(joint_positions >= piper_interface.JOINT_LIMITS_RAD["max"]):
          print("MAX JOINT LIMITS HIT!")

        stable_torques.append(np.array(torques))
        stable_joint_configs.append(np.array(joint_positions))
      else:
        stable_torques = []
        stable_joint_configs = []

      self._controller.command_torques(list(torques))
      time.sleep(0.005)

      if len(stable_joint_configs) > 200:
        print("Got sufficient data")
        break

    # Drop the first few samples
    stable_torques = stable_torques[100:]
    stable_joint_configs = stable_joint_configs[100:]

    if not stable_joint_configs:
      print("Error, could not get grav comp data in this configuration")
      return []
    else:
      # stable_torques = np.median(stable_torques, axis=0)
      # stable_joints = np.median(stable_joint_configs, axis=0)
      stable_torques = np.mean(stable_torques, axis=0)
      stable_joints = np.mean(stable_joint_configs, axis=0)

      return [
          GravityTorqueSample(
              joint_configuration=stable_joints,
              joint_idx=i,
              torque=float(stable_torques[i]),
          )
          for i in range(6)
      ]


def get_rest_position_for_orientation(orientation: str) -> np.ndarray:
  """Get the appropriate rest position based on arm orientation."""
  arm_orientation = piper_control.ArmOrientations.from_string(orientation)
  return np.array(arm_orientation.rest_position)


def get_gravity_compensation_gains(
    orientation: str,
) -> tuple[np.ndarray, np.ndarray]:
  """Get PD gains for gravity compensation based on arm orientation."""
  if orientation == "upright":
    # Original gains for upright/vertical orientation
    p_gains = np.array([3.0, 15.0, 12.0, 3.0, 3.0, 2.0])
    d_gains = np.array([3.0, 3.0, 3.0, 2.0, 2.0, 2.0])
  elif orientation in ["left", "right"]:
    # Gains for left/right mounted arms (may need tuning)
    p_gains = np.array([10.0, 15.0, 12.0, 3.0, 3.0, 2.0])
    d_gains = np.array([3.0, 3.0, 3.0, 2.0, 2.0, 2.0])
  else:
    raise ValueError(f"Unknown arm orientation: {orientation}")

  return p_gains, d_gains


def collect_poses_interactively(
    robot: piper_interface.PiperInterface,
) -> list[np.ndarray]:
  """Interactive pose collection mode."""
  print("=" * 60)
  print("INTERACTIVE POSE COLLECTION MODE")
  print("=" * 60)
  print("Instructions:")
  print("1. Manually move the robot arm to desired poses")
  print("2. Press ENTER to capture the current pose")
  print("3. Type 's' + ENTER to skip this pose (continue without capturing)")
  print("4. Type 'q' + ENTER when done collecting poses")
  print("5. The collected poses will be saved to:", _POSES_PATH.value)
  print("=" * 60)

  collected_poses = []

  try:
    while True:
      joint_angles = np.array(robot.get_joint_positions())
      print(f"Current joint angles: {joint_angles}")

      # Simple input - will pause and wait for user
      user_input = (
          input("Press ENTER to capture, 's' to skip, or 'q' to quit: ")
          .strip()
          .lower()
      )

      if user_input == "q":
        break
      elif user_input == "s":
        print("⏭️  Skipped this pose")
        print("-" * 40)
      else:  # Any other input (including empty) captures
        collected_poses.append(joint_angles.copy())
        print(f"✓ Captured pose {len(collected_poses)}!")
        print(f"Pose: {joint_angles}")
        print("-" * 40)

  except KeyboardInterrupt:
    print("\nInterrupted by user")

  print("\nPose collection complete!")
  print(f"Total collected poses: {len(collected_poses)}")

  return collected_poses


def save_poses_to_json(poses: list[np.ndarray], filename: str) -> None:
  """Save poses to JSON file."""
  poses_list = [pose.tolist() for pose in poses]
  with open(filename, "w", encoding="utf-8") as f:
    json.dump(poses_list, f, indent=2)
  print(f"Poses saved to: {filename}")


def load_poses_from_json(filename: str) -> list[np.ndarray]:
  """Load poses from JSON file."""
  try:
    with open(filename, "r", encoding="utf-8") as f:
      poses_list = json.load(f)
    poses = [np.array(pose) for pose in poses_list]
    print(f"Loaded {len(poses)} poses from: {filename}")
    return poses
  except FileNotFoundError:
    print(f"Poses file not found: {filename}")
    print("Using default hardcoded poses instead.")
    return get_default_poses()


def get_default_poses() -> list[np.ndarray]:
  """Get the default hardcoded poses."""
  return [
      np.array([0.0, 0.4, -1.0, -0.1, 0.294, -0.021]),
      np.array([0.0, 0.7, -1.7, -0.1, 0.294, -0.021]),
      np.array([0.0, 0.6, -1.3, -0.1, 0.294, -0.021]),
      np.array([-0.05, 0.3, -0.2, 0.124, -1.299, -0.019]),
      np.array([-0.05, 0.2, -0.15, 1.0, -1.291, 0.013]),
      np.array([-0.05, 0.8, -0.2, 1.0, 0.8, -0.3]),
      np.array([0.109, 0.4, -0.3, -1.216, -1.296, 0.264]),
      np.array([0.109, 1.57, -1.606, 0.0217, -0.922, -0.074]),
      np.array([0.109, 1.2, -1.606, 0.0217, -0.922, -0.074]),
      np.array([0.022, 0.274, -0.792, 1.541, -1.098, -1.121]),
      np.array([0.019, 1.0, -0.792, 1.767, -0.463, 1.331]),
      np.array([0.019, 1.332, -0.818, 0.311, -0.256, -1.854]),
      np.array([0.040, 1.922, -1.066, -1.801, -0.637, -1.854]),
      np.array([0.359, 2.422, -2.319, -1.468, -1.297, 1.458]),
      np.array([0.356, 2.422, -2.319, -1.179, -0.119, -1.746]),
      np.array([0.159, 2.428, -2.074, 1.433, 0.581, -0.023]),
      np.array([-0.72, 2.501, -2.075, -0.607, 0.957, -1.139]),
      np.array([-1.39, 1.981, -1.464, 0.599, 1.303, -1.139]),
      np.array([0.328, 1.861, -1.464, -0.299, 0.936, 0.552]),
      np.array([0.338, 2.819, -2.73, 0.257, 0.578, 0.996]),
      np.array([1.063, 2.708, -2.689, 1.446, -0.8954, 0.971]),
      np.array([-0.89, 2.213, -1.144, -1.015, 1.24, 0.0 - 974]),
      np.array([-0.51, 2.213, -1.074, -0.01, -1.094, 0.279]),
      np.array([0.8, 0.5, -2.3, 0.0, -0.2, 0.0]),
      np.array([0.866, 0.585, -2.711, -0.702, -0.666, -0.632]),
      np.array([1.075, 0.547, -2.710, -1.782, 0.503, 0.63]),
      np.array([0.339, 0.411, -0.964, 1.293, 1.176, 0.316]),
      np.array([0.0, 1.4, -0.4, 0.0, 0.0, 0.0]),
      np.array([0.0, 1.3, -0.7, 0.0, 0.0, 0.0]),
      np.array([0.0, 1.1, -0.9, 0.0, 0.0, 0.0]),
      np.array([0.0, 0.7, -0.9, 0.0, 0.0, 0.0]),
      np.array([0.0, 0.8, -1.2, 0.0, 0.0, 0.0]),
      np.array([0.0, 1.3, -1.5, 0.0, 0.0, 0.0]),
  ]


def generate_samples(
    robot: piper_interface.PiperInterface,
    controller: piper_control.MitJointPositionController,
    sample_poses: list[np.ndarray] | None = None,
    orientation: str = "upright",
) -> Sequence[GravityTorqueSample]:
  """Generate gravity compensation samples using provided poses."""

  if sample_poses is None:
    sample_poses = get_default_poses()

  # Get orientation-specific gains
  p_gains, d_gains = get_gravity_compensation_gains(orientation)

  print(f"Using {len(sample_poses)} sample poses for data collection")
  print(f"Using gains for {orientation} orientation: P={p_gains}, D={d_gains}")

  result = []
  for i, sample_pose in enumerate(sample_poses):
    print(f"Processing pose {i+1}/{len(sample_poses)}")
    grav_torque_sampler = GravityTorqueSampler(
        robot,
        controller,
        sample_pose,
        p_gains=p_gains,
        d_gains=d_gains,
    )

    result += grav_torque_sampler.sample()
  return result


def export(samples: Sequence[GravityTorqueSample], filename: str) -> None:
  serialised_samples = []
  for s in samples:
    serialised_samples.append(
        {
            "joints_cfg": list(s.joint_configuration),
            "joint_idx": s.joint_idx,
            "grav_comp_torque": s.torque,
        }
    )

  with open(filename, "w", encoding="utf-8") as f:
    json.dump(serialised_samples, f, indent=4, sort_keys=True)


def main(_):
  ports = piper_connect.find_ports()
  print(f"Piper ports: {ports}")

  piper_connect.activate(ports)
  ports = piper_connect.active_ports()
  if not ports:
    raise ValueError(
        "No ports found. Make sure the Piper is connected and turned on. "
        "If you are having issues connecting to the piper, check our "
        "troubleshooting guide @ "
        "https://github.com/Reimagine-Robotics/piper_control/blob/main/README.md"
    )
  print(f"Active ports: {ports}")

  can_port = _CAN_PORT.value or ports[0]
  print(f"Using can_port: {can_port}")

  robot = piper_interface.PiperInterface(can_port=can_port)

  # Set installation position based on arm orientation flag. We don't *think*
  # this affects reported joint efforts during data gathering, but it does allow
  # using built-in teach mode during pose collection.
  if _ARM_ORIENTATION.value == "upright":
    robot.set_installation_pos(piper_interface.ArmInstallationPos.UPRIGHT)
  elif _ARM_ORIENTATION.value == "left":
    robot.set_installation_pos(piper_interface.ArmInstallationPos.LEFT)
  elif _ARM_ORIENTATION.value == "right":
    robot.set_installation_pos(piper_interface.ArmInstallationPos.RIGHT)

  # Get the appropriate rest position for this orientation
  rest_position = get_rest_position_for_orientation(_ARM_ORIENTATION.value)

  # Set up robot for manual manipulation (low gains)
  print("resetting arm")
  piper_init.reset_arm(
      robot,
      arm_controller=piper_interface.ArmController.MIT,
      move_mode=piper_interface.MoveMode.MIT,
  )
  robot.show_status()

  if _MODE.value == "collect_poses":
    print("Running in pose collection mode")
    print("Setting up robot for manual manipulation...")

    # Move the arm joints using MIT mode controller with low gains for manual
    # manipulation
    with piper_control.MitJointPositionController(
        robot,
        kp_gains=0.0,  # Zero gains for manual manipulation
        kd_gains=0.0,
        rest_position=None,
    ) as controller:

      # Send rest pose once to set gains.
      controller.move_to_position(list(rest_position), timeout=0.01)

      # Collect poses interactively
      collected_poses = collect_poses_interactively(robot)

      if collected_poses:
        save_poses_to_json(collected_poses, _POSES_PATH.value)
        print("\nTo use these poses for data collection, run:")
        print(
            f"python {__file__} --mode=collect_data --poses_path={_POSES_PATH.value}"
        )
      else:
        print("No poses collected.")

  elif _MODE.value == "collect_data":
    print("Running in data collection mode")

    # Load poses from file if it exists
    sample_poses = load_poses_from_json(_POSES_PATH.value)

    # Move the arm joints using MIT mode controller for data collection
    with piper_control.MitJointPositionController(
        robot,
        kp_gains=10.0,
        kd_gains=0.8,
        rest_position=list(rest_position),
    ) as controller:

      all_samples = generate_samples(
          robot, controller, sample_poses, _ARM_ORIENTATION.value
      )
      export(all_samples, _OUT_PATH.value)
      print(all_samples)
      print(f"Exported to: {_OUT_PATH.value}")
  else:
    print("Invalid mode selected.")

  piper_init.disable_arm(robot)


if __name__ == "__main__":
  app.run(main)
