"""Grav Comp model data gathering script.

This script will move the arm to various joint configurations, and at each pose
it will try to apply a torque (using a PD-controller) to find a stable
configuration where the torque exactly counteracts gravity. This will be
recorded as a data point. Each data point is a joint angle configuration and a
torque command per joint that compensates for gravity.
The generated set of grav comp torques is later used in teach_mode.py to fit a
gravity model that can be used in any joint configuration.
"""

from typing import Sequence
import dataclasses
import json
import time

from absl import app, flags
import numpy as np

from piper_control import piper_connect
from piper_control import piper_init
from piper_control import piper_interface
from piper_control import piper_control


_OUT_PATH = flags.DEFINE_string(
    "out_path",
    "/tmp/grav_comp_samples.json",
    "The output path for the sampled gravity compensation torques.",
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


def generate_samples(
    robot: piper_interface.PiperInterface,
    controller: piper_control.MitJointPositionController,
) -> Sequence[GravityTorqueSample]:

  sample_poses = [
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

  result = []
  for sample_pose in sample_poses:
    grav_torque_sampler = GravityTorqueSampler(
        robot,
        controller,
        sample_pose,
        p_gains=np.array([3.0, 15.0, 12.0, 3.0, 3.0, 2.0]),
        d_gains=np.array([3.0, 3.0, 3.0, 2.0, 2.0, 2.0]),
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

  robot = piper_interface.PiperInterface(can_port=ports[0])
  robot.set_installation_pos(piper_interface.ArmInstallationPos.UPRIGHT)

  print("resetting arm")
  piper_init.reset_arm(
      robot,
      arm_controller=piper_interface.ArmController.MIT,
      move_mode=piper_interface.MoveMode.MIT,
  )

  robot.show_status()

  # Move the arm joints using Mit mode controller.
  with piper_control.MitJointPositionController(
      robot,
      kp_gains=10.0,
      kd_gains=0.8,
      rest_position=piper_control.REST_POSITION,
  ) as controller:

    all_samples = generate_samples(robot, controller)
    export(all_samples, _OUT_PATH.value)
    print(all_samples)
    print(f"Exported to: {_OUT_PATH.value}")

  piper_init.disable_arm(robot)


if __name__ == "__main__":
  app.run(main)
