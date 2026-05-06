"""Piper control node for the Piper robot."""

from __future__ import annotations

import dataclasses
import functools
import json
import math
import os
import pathlib
import signal
import tarfile
import tempfile
import time

import numpy as np
import rclpy
from piper_control import piper_connect, piper_control, piper_init, piper_interface
from rclpy import logging
from rclpy.node import Node
from sensor_msgs import msg as sensor_msgs
from std_msgs import msg as std_msgs
from std_srvs import srv as std_srvs

from piper_control_ros2 import get_metadata
from piper_control_ros2.teach_mode import teach_mode

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


# Control parameters
CONTROL_HZ = 200
DEFAULT_KP_GAINS = (5.5, 5.5, 10.0, 15.0, 25.0, 15.0)
DEFAULT_KD_GAINS = (0.8, 0.8, 0.8, 0.8, 0.8, 0.8)


@dataclasses.dataclass
class JointCommand:
  """Data class for joint commands.

  Attributes:
    positions: The joint positions to command.
    velocities: The joint velocities to command.
    efforts: The joint efforts to command.
    kp_gains: P gains for the joint controller.
    kd_gains: D gains for the joint controller.
  """

  positions: tuple[float, ...] = ()
  velocities: tuple[float, ...] = ()
  efforts: tuple[float, ...] = ()
  kp_gains: tuple[float, ...] = ()
  kd_gains: tuple[float, ...] = ()

  @classmethod
  def from_msg(
      cls,
      msg: std_msgs.Float64MultiArray,
      logger: logging.RcutilsLogger | None = None,
  ) -> JointCommand:
    """Create a JointCommand from a Float64MultiArray message.

    Args:
      msg: The incoming joint command message containing positions, velocities,
        or efforts. The message may also contain kp and kd gains. The msg.layout
        contains information about how to split the command into positions,
        velocities, efforts and gains.
      logger: Optional logger to log warnings if the message is malformed.

    Returns:
      A JointCommand instance.
    """
    joint_command = cls()
    layout = msg.layout
    if not layout or not layout.dim:
      raise ValueError(f"Invalid message layout for message: {msg}")

    offset = layout.data_offset or 0
    for dim in layout.dim:
      if dim.label == "positions":
        joint_command.positions = tuple(msg.data[offset : offset + dim.size])
      elif dim.label == "velocities":
        joint_command.velocities = tuple(msg.data[offset : offset + dim.size])
      elif dim.label == "efforts":
        joint_command.efforts = tuple(msg.data[offset : offset + dim.size])
      elif dim.label == "kp_gains":
        joint_command.kp_gains = tuple(msg.data[offset : offset + dim.size])
      elif dim.label == "kd_gains":
        joint_command.kd_gains = tuple(msg.data[offset : offset + dim.size])
      elif logger:
        logger.warning(
            f"Unknown dimension label '{dim.label}' in message: {msg}",
        )

      offset += dim.size

    return joint_command


class PiperControlNode(Node):
  """ROS2 node for controlling the Piper robot."""

  def __init__(self):
    """PiperControlNode constructor."""
    super().__init__("piper_control_node")

    self.declare_parameter("can_port", "can0")
    self.can_port = (
        self.get_parameter("can_port").get_parameter_value().string_value
    )

    self.declare_parameter("arm_orientation", "upright")
    self.arm_orientation = (
        self.get_parameter("arm_orientation").get_parameter_value().string_value
    )

    # Gravity model options.
    # NOTE: For this to be used, your piper_control dependency must have the
    # gravity options enabled during installation.
    # This gravity model is used for the teach mode controller AND is used to
    # provide feed-forward torque terms during normal operation.
    self.declare_parameter("gravity_model_mujoco_path", "")
    self.gravity_model_mujoco_path = (
        self.get_parameter("gravity_model_mujoco_path")
        .get_parameter_value()
        .string_value
    )

    # A combination of a path to an archive containing a Mujoco model, and a
    # name of the model xml file inside the archive. This parameter should be
    # of the form "path#filename". So for example it can be
    # "/data/piper/model.tar.xz#mjmodel.xml".
    self.declare_parameter("gravity_model_archive", "")
    self.gravity_model_archive = (
        self.get_parameter("gravity_model_archive")
        .get_parameter_value()
        .string_value
    )

    self.declare_parameter("piper_arm_type", "PIPER")
    self._piper_arm_type = (
        self.get_parameter("piper_arm_type").get_parameter_value().string_value
    )

    self.declare_parameter("piper_gripper_type", "V2")  # 10 cm gripper.
    self._piper_gripper_type = (
        self.get_parameter("piper_gripper_type")
        .get_parameter_value()
        .string_value
    )

    # Create a CAN connection to robot.
    ports = piper_connect.find_ports()
    print(f"Available ports: {ports}")
    piper_connect.activate(ports)
    print(f"Active ports: {piper_connect.active_ports()}")

    if not ports:
      readme = (
          "https://github.com/Reimagine-Robotics/piper_control/blob/main/"
          "README.md"
      )
      raise ValueError(
          "No ports found. Make sure the Piper is connected and turned on. "
          "If you are having issues connecting to the piper, check our "
          "troubleshooting guide @ "
          f"{readme}",
      )

    self._robot = piper_interface.PiperInterface(
        can_port=self.can_port,
        piper_arm_type=_get_piper_arm_type(self._piper_arm_type),
        piper_gripper_type=_get_piper_gripper_type(self._piper_gripper_type),
    )

    # Get the appropriate rest position based on arm orientation
    arm_orientation = piper_control.ArmOrientations.from_string(
        self.arm_orientation,
    )
    self._robot.set_installation_pos(
        piper_interface.ArmInstallationPos.from_string(self.arm_orientation)
    )
    rest_position = arm_orientation.rest_position

    self._arm_controller = piper_control.MitJointPositionController(
        self._robot,
        kp_gains=DEFAULT_KP_GAINS,
        kd_gains=DEFAULT_KD_GAINS,
        rest_position=rest_position,
    )

    self._gripper_controller = piper_control.GripperController(self._robot)

    self.namespace = (
        self.declare_parameter("namespace", "piper")
        .get_parameter_value()
        .string_value
    )

    # Joint control
    self.joint_state_pub = self.create_publisher(
        sensor_msgs.JointState,
        f"{self.namespace}/joint_states",
        qos_profile=10,
    )
    self.joint_command_sub = self.create_subscription(
        std_msgs.Float64MultiArray,
        f"{self.namespace}/joint_commands",
        self.joint_cmd_callback,
        qos_profile=10,
    )

    # Gripper control
    self.gripper_state_pub = self.create_publisher(
        sensor_msgs.JointState,
        f"{self.namespace}/gripper_state",
        qos_profile=10,
    )
    self.gripper_command_sub = self.create_subscription(
        sensor_msgs.JointState,
        f"{self.namespace}/gripper_command",
        self.gripper_cmd_callback,
        qos_profile=10,
    )

    # Service servers
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/reset",
        self.handle_reset,  # type: ignore
    )
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/hard_reset",
        self.handle_hard_reset,  # type: ignore
    )
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/clear_errors",
        self.handle_clear_errors,  # type: ignore
    )
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/enable",
        self.handle_enable,  # type: ignore
    )
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/disable",
        self.handle_disable,  # type: ignore
    )
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/enable_arm",
        self.handle_enable_arm,  # type: ignore
    )
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/disable_arm",
        self.handle_disable_arm,  # type: ignore
    )
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/enable_gripper",
        self.handle_enable_gripper,  # type: ignore
    )
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/disable_gripper",
        self.handle_disable_gripper,  # type: ignore
    )
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/get_status",
        self.handle_get_status,  # type: ignore
    )
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/is_enabled",
        self.handle_is_enabled,  # type: ignore
    )
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/calibrate_j0",
        self.handle_calibrate_j0,  # type: ignore
    )

    # Publish metadata about the node.
    self.node_metadata_pub = self.create_publisher(
        std_msgs.String,
        f"{self.namespace}/node_metadata",
        qos_profile=10,
    )

    self.get_logger().info(
        f"Piper control node started with CAN ID: {self.can_port}",
    )

    # Timer to periodically publish joint states
    self.create_timer(1.0 / CONTROL_HZ, self.publish_joint_states)
    self.create_timer(1.0 / CONTROL_HZ, self.publish_gripper_state)

    # Timer to publish node metadata
    self.create_timer(1.0, self.publish_node_metadata)

    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/teach_mode_enable",
        self.handle_teach_mode_enable,  # type: ignore
    )
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/teach_mode_disable",
        self.handle_teach_mode_disable,  # type: ignore
    )

    self._gravity_model = self._create_gravity_model()

    self._teach_controller = teach_mode.TeachController(
        self._robot,
        self._arm_controller,
        self._gravity_model,
    )
    self._teach_mode_active = False
    self._teach_mode_timer = self.create_timer(
        0.005,
        self.teach_mode,
        autostart=False,
    )

  def _parse_gravity_model_archive(self) -> tuple[str, str] | None:
    """Parse the gravity model archive parameter."""
    if not self.gravity_model_archive:
      return None
    parts = self.gravity_model_archive.split("#")
    if len(parts) != 2:
      self.get_logger().warn(
          f"Invalid gravity model archive: {self.gravity_model_archive}"
      )
      return None

    if not parts[1].endswith("xml"):
      self.get_logger().warn(
          "Gravity archive model name does not end with xml: "
          f"{self.gravity_model_archive}"
      )
      return None

    return parts[0], parts[1]

  def _create_gravity_model(self):
    """Create and return gravity compensation model, or None if unavailable."""
    if not self.gravity_model_mujoco_path and not self.gravity_model_archive:
      self.get_logger().info("No gravity model path/archive provided.")
      return None

    mujoco_model_path = None

    if self.gravity_model_mujoco_path:
      if not os.path.isfile(self.gravity_model_mujoco_path):
        self.get_logger().warn(
            f"Mujoco path not found: {self.gravity_model_mujoco_path}"
        )
        return None

      mujoco_model_path = self.gravity_model_mujoco_path

    else:
      assert self.gravity_model_archive
      archive_path, model_filename = self._parse_gravity_model_archive()
      if not os.path.isfile(archive_path):
        self.get_logger().warn(f"Archive path not found: {archive_path}")
        return None

      model_archive_out_dir_path = tempfile.mkdtemp()

      with tarfile.open(archive_path) as model_archive:
        model_archive.extractall(model_archive_out_dir_path)

      mujoco_model_path = str(
          pathlib.Path(model_archive_out_dir_path) / model_filename
      )

    try:
      # pylint: disable-next=import-outside-toplevel
      from piper_control import gravity_compensation
    except ImportError as e:
      raise ImportError(
          "Please install piper_control with gravity support.",
      ) from e

    firmware_version = self._robot.get_piper_firmware_version()
    scaling = gravity_compensation.direct_scaling_factors(firmware_version)
    self.get_logger().info("Gravity compensation:")

    if self.gravity_model_mujoco_path:
      self.get_logger().info(f"  mujoco_path: {self.gravity_model_mujoco_path}")
    else:
      self.get_logger().info(f"  mujoco_archive: {self.gravity_model_archive}")

    self.get_logger().info(f"  arm_orientation: {self.arm_orientation}")
    self.get_logger().info(f"  firmware_version: {firmware_version}")
    self.get_logger().info(f"  direct scaling: {scaling}")
    self.get_logger().info(f"  arm type: {self._piper_arm_type}")
    self.get_logger().info(f"  gripper type: {self._piper_gripper_type}")
    return gravity_compensation.GravityCompensationModel(
        model_path=mujoco_model_path,
        firmware_version=firmware_version,
    )

  def clean_stop(self) -> None:
    self._arm_controller.stop()
    self._gripper_controller.stop()

    piper_init.disable_arm(self._robot)
    piper_init.disable_gripper(self._robot)

  def joint_cmd_callback(self, msg: std_msgs.Float64MultiArray) -> None:
    """Handle incoming joint commands.

    Args:
      msg: The incoming joint command message containing positions, velocities,
        or efforts. The message may also contain kp and kd gains. The msg.layout
        contains information about how to split the command into positions,
        velocities, efforts and gains.
    """
    try:
      joint_command = JointCommand.from_msg(msg, self.get_logger())
    except ValueError as e:
      self.get_logger().warn(f"Invalid joint command message: {e}")
      return

    positions = list(joint_command.positions)
    velocities = list(joint_command.velocities)
    efforts = list(joint_command.efforts)

    if positions:
      self.get_logger().debug(f"Received joint positions: {positions}")
      if joint_command.kp_gains:
        kp_gains = list(joint_command.kp_gains)
        if len(kp_gains) != len(positions):
          self.get_logger().warn(
              "Received joint positions with mismatched kp gains",
          )
          kp_gains = None
      else:
        kp_gains = None
      if joint_command.kd_gains:
        kd_gains = list(joint_command.kd_gains)
        if len(kd_gains) != len(positions):
          self.get_logger().warn(
              "Received joint positions with mismatched kd gains",
          )
          kd_gains = None
      else:
        kd_gains = None

      torque = None

      # If we have a gravity model, use it to compute feed-forward torques.
      if self._gravity_model:
        torque = self._gravity_model.predict(positions).tolist()

      # If there is a user-provided torque, then use it as an additional
      # residual over gravity (if any).
      if efforts:
        if torque is None:
          torque = efforts
        else:
          assert len(efforts) == len(torque)
          torque = [t + e for t, e in zip(torque, efforts)]

      self._arm_controller.command_joints(
          positions,
          kp_gains=kp_gains,
          kd_gains=kd_gains,
          torques_ff=torque,
          velocities=velocities,
      )

    elif velocities:
      self.get_logger().warn("Velocity actuation not currently supported")

    elif efforts:
      if positions or velocities:
        self.get_logger().warn(
            "Received joint efforts, but also positions or velocities",
        )

      self.get_logger().debug(f"Received joint efforts: {efforts}")
      self._arm_controller.command_torques(efforts)

    else:
      self.get_logger().warn(f"Received invalid joint command: {msg}")

  def publish_joint_states(self):
    joint_positions = self._robot.get_joint_positions()
    joint_velocities = self._robot.get_joint_velocities()
    joint_efforts = self._robot.get_joint_efforts()
    msg = sensor_msgs.JointState()
    msg.name = JOINT_NAMES
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.position = list(joint_positions)
    msg.velocity = list(joint_velocities)
    msg.effort = list(joint_efforts)
    self.joint_state_pub.publish(msg)

  def gripper_cmd_callback(self, msg: sensor_msgs.JointState) -> None:
    position = msg.position[0] if msg.position else 0.0
    effort = msg.effort[0] if msg.effort else 0.0

    self._gripper_controller.command_position(position, effort)

    self.get_logger().debug(
        f"Gripper command received: Position={position}, Effort={effort}",
    )

  def publish_gripper_state(self):
    position, effort = self._robot.get_gripper_state()

    msg = sensor_msgs.JointState()
    msg.position = [position]
    msg.effort = [effort]
    msg.header.stamp = self.get_clock().now().to_msg()

    self.gripper_state_pub.publish(msg)
    self.get_logger().debug(
        f"Published gripper state: Position={position}, Effort={effort}",
    )

  def handle_reset(
      self,
      request: std_srvs.Trigger.Request,
      response: std_srvs.Trigger.Response,
  ) -> std_srvs.Trigger.Response:
    del request

    try:
      self._arm_controller.stop()
      self._gripper_controller.stop()

      piper_init.reset_arm(
          self._robot,
          arm_controller=piper_interface.ArmController.MIT,
          move_mode=piper_interface.MoveMode.MIT,
      )
      piper_init.reset_gripper(self._robot)

      self._arm_controller.start()
      self._gripper_controller.start()

      response.success = True
      response.message = "Robot reset."

    except RuntimeError as e:
      print(f"Error resetting robot: {e}")
      response.message = "Robot not reset."
      response.success = False

    return response

  def handle_hard_reset(
      self,
      request: std_srvs.Trigger.Request,
      response: std_srvs.Trigger.Response,
  ) -> std_srvs.Trigger.Response:
    del request

    self._robot.hard_reset()

    response.success = True
    response.message = "Robot resumed from emergency stop."
    return response

  def handle_clear_errors(
      self,
      request: std_srvs.Trigger.Request,
      response: std_srvs.Trigger.Response,
  ) -> std_srvs.Trigger.Response:
    del request

    try:
      piper_init.clear_joint_errors(self._robot)
      response.success = True
      response.message = "Joint errors cleared."
    except (RuntimeError, TimeoutError) as e:
      self.get_logger().warn(f"Clear errors failed: {e}")
      response.success = False
      response.message = f"Clear errors failed: {e}"

    return response

  def handle_enable(
      self,
      request: std_srvs.Trigger.Request,
      response: std_srvs.Trigger.Response,
  ) -> std_srvs.Trigger.Response:
    del request

    try:
      piper_init.enable_arm(
          self._robot,
          arm_controller=piper_interface.ArmController.MIT,
          move_mode=piper_interface.MoveMode.MIT,
      )
      piper_init.enable_gripper(self._robot)
    except TimeoutError as e:
      self.get_logger().warn(f"Enable robot failed: {e}")
      response.success = False
      response.message = "Robot not enabled, gripper failed to enable."
      return response

    self._arm_controller.start()
    self._gripper_controller.start()

    response.success = True
    response.message = "Robot enabled."
    return response

  def handle_enable_arm(
      self,
      request: std_srvs.Trigger.Request,
      response: std_srvs.Trigger.Response,
  ) -> std_srvs.Trigger.Response:
    del request

    try:
      piper_init.enable_arm(
          self._robot,
          arm_controller=piper_interface.ArmController.MIT,
          move_mode=piper_interface.MoveMode.MIT,
      )
    except TimeoutError as e:
      self.get_logger().warn(f"Enable arm failed: {e}")
      response.success = False
      response.message = "Arm not enabled."
      return response

    self._arm_controller.start()

    response.success = True
    response.message = "Arm enabled."
    return response

  def handle_enable_gripper(
      self,
      request: std_srvs.Trigger.Request,
      response: std_srvs.Trigger.Response,
  ) -> std_srvs.Trigger.Response:
    del request

    try:
      piper_init.enable_gripper(self._robot)
    except TimeoutError as e:
      self.get_logger().warn(f"Enable gripper failed: {e}")
      response.success = False
      response.message = "Gripper not enabled."
      return response

    self._gripper_controller.start()

    response.success = True
    response.message = "Gripper enabled."
    return response

  def handle_disable(
      self,
      request: std_srvs.Trigger.Request,
      response: std_srvs.Trigger.Response,
  ) -> std_srvs.Trigger.Response:
    del request

    self._arm_controller.stop()
    self._gripper_controller.stop()

    try:
      piper_init.disable_arm(self._robot)
      piper_init.disable_gripper(self._robot)
    except TimeoutError as e:
      self.get_logger().warn(f"Disable robot failed: {e}")
      response.success = False
      response.message = "Robot not disabled, arm or gripper failed to disable."
      return response

    response.success = True
    response.message = "Robot disabled."
    return response

  def handle_disable_arm(
      self,
      request: std_srvs.Trigger.Request,
      response: std_srvs.Trigger.Response,
  ) -> std_srvs.Trigger.Response:
    del request

    self._arm_controller.stop()

    try:
      piper_init.disable_arm(self._robot)
    except TimeoutError as e:
      self.get_logger().warn(f"Disable arm failed: {e}")
      response.success = False
      response.message = "Arm not disabled."
      return response

    response.success = True
    response.message = "Arm disabled."
    return response

  def handle_disable_gripper(
      self,
      request: std_srvs.Trigger.Request,
      response: std_srvs.Trigger.Response,
  ) -> std_srvs.Trigger.Response:
    del request

    self._gripper_controller.stop()

    try:
      piper_init.disable_gripper(self._robot)
    except TimeoutError as e:
      self.get_logger().warn(f"Disable gripper failed: {e}")
      response.success = False
      response.message = "Gripper not disabled."
      return response

    response.success = True
    response.message = "Gripper disabled."
    return response

  def handle_get_status(
      self,
      request: std_srvs.Trigger.Request,
      response: std_srvs.Trigger.Response,
  ) -> std_srvs.Trigger.Response:
    del request
    response.success = True
    response.message = self._robot.format_status()
    return response

  def handle_is_enabled(
      self,
      request: std_srvs.Trigger.Request,
      response: std_srvs.Trigger.Response,
  ) -> std_srvs.Trigger.Response:
    del request
    is_enabled = self._robot.is_enabled()
    response.success = True
    response.message = f"Robot enabled: {is_enabled}"
    return response

  def _push_and_sample_hardstop(self) -> float:
    """Push toward the hard-stop, returns the joint value once stopped."""
    self._arm_controller.command_torques([-0.3] + [None] * 5)
    time.sleep(0.2)  # Wait a bit for the arm to start moving.

    samples = []
    while len(samples) < 30:
      jvel = self._robot.get_joint_velocities()
      raw_j0 = self._robot.get_joint_positions(raw=True)[0]

      if math.isclose(jvel[0], 0.0, abs_tol=0.001):
        samples.append(raw_j0)
      time.sleep(2.0 / CONTROL_HZ)

    return np.median(samples)

  def handle_calibrate_j0(
      self,
      request: std_srvs.Trigger.Request,
      response: std_srvs.Trigger.Response,
  ) -> std_srvs.Trigger.Response:
    """Calibrates the J0 sensors using the min hard-stop.

    Moves the arm near the hard-stop, pushes against it with torque and records
    the sensed joint value at the hard-stop. Then moves and returns the arm to
    the previous state it was in.
    """
    del request

    return_to_teach_mode = False
    if self._teach_mode_active:
      return_to_teach_mode = True
      self._teach_mode_active = False
      self._teach_mode_timer.cancel()

    return_joint_positions = self._robot.get_joint_positions()

    target = list(return_joint_positions)

    arm_type = _get_piper_arm_type(self._piper_arm_type)
    target[0] = piper_interface.get_joint_limits(arm_type)["min"][0] + 0.01
    self._arm_controller.move_to_position(target=target, threshold=0.1)

    sensed_j0_angle = self._push_and_sample_hardstop()

    calibrated_j0_offset = piper_interface.J0_MIN_HARDSTOP - sensed_j0_angle

    self._arm_controller.move_to_position(
        target=return_joint_positions, threshold=0.1
    )
    self._robot.set_j0_calibration_offset(calibrated_j0_offset)

    if return_to_teach_mode:
      self._teach_mode_active = True
      self._teach_mode_timer.reset()

    response.success = True
    response.message = f"Robot j0 offset calibrated: {calibrated_j0_offset}"
    return response

  def handle_teach_mode_enable(
      self,
      request: std_srvs.Trigger.Request,
      response: std_srvs.Trigger.Response,
  ) -> std_srvs.Trigger.Response:
    del request

    self._teach_mode_active = True
    self._teach_mode_timer.reset()

    response.success = True
    response.message = "Teach mode enabled."
    return response

  def handle_teach_mode_disable(
      self,
      request: std_srvs.Trigger.Request,
      response: std_srvs.Trigger.Response,
  ) -> std_srvs.Trigger.Response:
    del request

    self._teach_mode_active = False
    self._teach_mode_timer.cancel()

    # Ensure that the last command the robot sees isnt a constant torque command
    # from the last joint configuration that teach mode saw.
    cur_joint_positions = self._robot.get_joint_positions()
    self._arm_controller.command_joints(cur_joint_positions)

    response.success = True
    response.message = "Teach mode disabled."
    return response

  def teach_mode(self) -> None:
    assert self._teach_mode_active
    self._teach_controller.step()

  def publish_node_metadata(self) -> None:
    """Publish metadata about the node."""
    metadata = get_metadata.get_metadata(self._robot)
    metadata["gravity_model_mujoco_path"] = (
        self.gravity_model_mujoco_path or None
    )
    metadata["gravity_model_archive"] = self.gravity_model_archive or None
    metadata["arm_orientation"] = self.arm_orientation
    msg = std_msgs.String(data=json.dumps(metadata))
    self.node_metadata_pub.publish(msg)


def _get_piper_arm_type(
    piper_arm_type_str: str,
) -> piper_interface.PiperArmType:
  try:
    return piper_interface.PiperArmType[piper_arm_type_str]
  except KeyError as e:
    raise ValueError(
        f"Invalid piper_arm_type: {piper_arm_type_str}. Valid options are: "
        f"{[t.name for t in piper_interface.PiperArmType]}",
    ) from e


def _get_piper_gripper_type(
    piper_gripper_type_str: str,
) -> piper_interface.PiperGripperType:
  try:
    return piper_interface.PiperGripperType[piper_gripper_type_str]
  except KeyError as e:
    raise ValueError(
        f"Invalid piper_gripper_type: {piper_gripper_type_str}. "
        "Valid options are: "
        f"{[t.name for t in piper_interface.PiperGripperType]}",
    ) from e


def term_handler(signum, frame, node: PiperControlNode) -> None:
  del frame
  node.get_logger().info(f"Stopping node on signal: {signum}")
  node.clean_stop()
  node.destroy_node()
  rclpy.shutdown()


def main(args=None):
  rclpy.init(args=args)

  # Create the PiperControlNode. If a gravity model path is provided via a
  # command line argument, this will also enable teach mode for the ROS node.
  node = PiperControlNode()

  # Register signal handlers to prevent banging the arm on a Ctrl-C.
  signal.signal(signal.SIGINT, functools.partial(term_handler, node=node))
  signal.signal(signal.SIGTERM, functools.partial(term_handler, node=node))

  rclpy.spin(node)


if __name__ == "__main__":
  main()
