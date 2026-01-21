"""Teach mode via a custom gravity compensation mode.

The custom gravity compensation works by gathering a set of stable gravity
compensating torques using run_gather_data.py for various joint configurations.
These are then used to learn a torque residual function against a MUJOCO model,
and this composite gravity model (MUJOCO + learned residual) is used to generate
on-the-fly gravity compensation torques for the Piper arm. These torques are
just enough to prevent the arm dropping from gravity, but any external force
applied (eg: a person moving the arm) will be enough to move the arm around
freely.
"""

import numpy as np
from piper_control import piper_control, piper_interface


class TeachController:
  """A controller that allows teach mode by commanding grav comp torques."""

  def __init__(
      self,
      robot: piper_interface.PiperInterface,
      controller: piper_control.MitJointPositionController,
      gravity_model,
  ):
    self._robot = robot
    self._controller = controller
    self._gravity_model = gravity_model

  def step(self) -> None:
    """Perform a single control step of the teach controller."""
    qpos = self._robot.get_joint_positions()
    if self._gravity_model:
      qvel = np.array(self._robot.get_joint_velocities())

      hover_torque = self._gravity_model.predict(qpos)
      # Stability torque acts to counteract joint movement. A dampener
      # basically.
      stability_torque = -qvel * 1.0  # Damping factor
      applied_torque = hover_torque + stability_torque
      self._controller.command_torques(applied_torque)
    else:
      self._controller.command_torques(np.zeros_like(qpos).tolist())
