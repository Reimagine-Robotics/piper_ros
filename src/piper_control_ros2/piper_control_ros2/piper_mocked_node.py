"""
A mocked piper control node for testing purposes.
"""

from __future__ import annotations

import json
import socket
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs import msg as sensor_msgs
from std_msgs import msg as std_msgs
from std_srvs import srv as std_srvs

from piper_control_ros2 import get_metadata
from piper_control_ros2.piper_control_node import JOINT_NAMES, JointCommand


def always_succeed_response(msg: str) -> None:
  def callback(
      request: std_srvs.Trigger.Request, response: std_srvs.Trigger.Response
  ):
    del request  # unused
    response.message = msg
    response.success = True
    return response

  return callback


class PiperMockedNode(Node):
  """A mocked piper node."""

  def __init__(self):
    """PiperControlNode constructor."""

    super().__init__("piper_mock_node")

    initial_positions = (
        self.declare_parameter("initial_positions", [0.0] * len(JOINT_NAMES))
        .get_parameter_value()
        .double_array_value
    )

    if len(initial_positions) != len(JOINT_NAMES):
      raise RuntimeError("Expected 6 values for piper's initial positions")

    self._current_joint_positions = list(initial_positions)
    self._current_joint_velocities = [0.0] * len(JOINT_NAMES)
    self._current_joint_efforts = [0.0] * len(JOINT_NAMES)
    self._gripper_position = 0.0
    self._gripper_effort = 0.0
    self._joint_positions_lock = threading.Lock()

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
        always_succeed_response("Robot reset."),
    )
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/enable",
        always_succeed_response("Robot enabled."),
    )
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/disable",
        always_succeed_response("Robot disabled."),
    )
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/enable_arm",
        always_succeed_response("Arm enabled."),
    )
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/disable_arm",
        always_succeed_response("Arm disabled."),
    )
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/enable_gripper",
        always_succeed_response("Gripper enabled."),
    )
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/disable_gripper",
        always_succeed_response("Gripper disabled."),
    )
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/get_status",
        always_succeed_response("Mock arm status."),
    )
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/is_enabled",
        always_succeed_response("Robot enabled: True"),
    )

    # Publish metadata about the node.
    self.node_metadata_pub = self.create_publisher(
        std_msgs.String,
        f"{self.namespace}/node_metadata",
        qos_profile=10,
    )

    self.get_logger().info("Piper control node started.")

    # Timer to periodically publish joint states
    self.create_timer(0.005, self.publish_joint_states)

    # Timer to publish node metadata
    self.create_timer(1.0, self.publish_node_metadata)

    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/teach_mode_enable",
        always_succeed_response("Teach mode enabled."),
    )
    self.create_service(
        std_srvs.Trigger,
        f"{self.namespace}/teach_mode_disable",
        always_succeed_response("Teach mode disabled."),
    )

  def joint_cmd_callback(self, msg: std_msgs.Float64MultiArray) -> None:
    """Handle incoming joint commands."""
    try:
      joint_command = JointCommand.from_msg(msg, self.get_logger())
    except ValueError as e:
      self.get_logger().warn(f"Invalid joint command message: {e}")
      return

    positions = list(joint_command.positions)
    velocities = list(joint_command.velocities)
    efforts = list(joint_command.efforts)
    with self._joint_positions_lock:
      if positions:
        self._current_joint_positions = positions
      if velocities:
        self._current_joint_velocities = velocities
      if efforts:
        self._current_joint_efforts = efforts

  def publish_joint_states(self):
    msg = sensor_msgs.JointState()
    gripper_msg = sensor_msgs.JointState()
    msg.name = JOINT_NAMES
    msg.header.stamp = self.get_clock().now().to_msg()
    gripper_msg.header.stamp = self.get_clock().now().to_msg()
    with self._joint_positions_lock:
      msg.position = self._current_joint_positions
      msg.velocity = self._current_joint_velocities
      msg.effort = self._current_joint_efforts
      gripper_msg.position = [self._gripper_position]
      gripper_msg.effort = [self._gripper_effort]
    self.gripper_state_pub.publish(msg)
    self.joint_state_pub.publish(msg)

  def gripper_cmd_callback(self, msg: sensor_msgs.JointState) -> None:
    position = msg.position[0] if msg.position else 0.0
    effort = msg.effort[0] if msg.effort else 0.0

    with self._joint_positions_lock:
      self._gripper_position = position
      self._gripper_effort = effort

  def publish_node_metadata(self) -> None:
    """Publish metadata about the node."""
    metadata = {
        "piper_ros_version": get_metadata.get_piper_ros_version(),
        "piper_ros_git_hash": get_metadata.get_piper_ros_git_hash(),
        "piper_control_version": get_metadata.get_piper_control_version(),
        "piper_control_git_hash": get_metadata.get_piper_control_git_hash(),
        "hostname": socket.gethostname(),
        "piper_interface_version": "mocked_node",
        "piper_protocol_version": "mocked_node",
        "piper_sdk_version": "mocked_node",
        "piper_firmware_version": "mocked_node",
    }
    msg = std_msgs.String(data=json.dumps(metadata))
    self.node_metadata_pub.publish(msg)


def main():
  rclpy.init()

  node = PiperMockedNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.get_logger().info("Shutting down piper mock node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()
