"""
Piper control node for the Piper robot.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from piper_control import piper_connect, piper_control
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from transformations import transformations as tr


class PiperControlNode(Node):
  """
  ROS2 node for controlling the Piper robot.
  """

  def __init__(self):
    super().__init__("piper_control_node")

    self.declare_parameter("can_port", "can0")
    self.can_port = (
        self.get_parameter("can_port").get_parameter_value().string_value
    )

    # Create a CAN connection to robot.
    ports = piper_connect.find_ports()
    print(f"Available ports: {ports}")
    piper_connect.activate(ports)
    print(f"Active ports: {piper_connect.active_ports()}")

    self.robot = piper_control.PiperControl(can_port=self.can_port)

    self.namespace = (
        self.declare_parameter("namespace", "piper")
        .get_parameter_value()
        .string_value
    )

    # Joint control
    self.joint_state_pub = self.create_publisher(
        JointState, f"{self.namespace}/joint_states", 10
    )
    self.joint_command_sub = self.create_subscription(
        JointState,
        f"{self.namespace}/joint_positions_cmd",
        self.joint_cmd_callback,
        10,
    )

    # Cartesian control
    self.cartesian_position_pub = self.create_publisher(
        Pose, f"{self.namespace}/end_effector_pose", 10
    )
    self.cartesian_command_sub = self.create_subscription(
        Pose,
        f"{self.namespace}/cartesian_position_cmd",
        self.cartesian_cmd_callback,
        10,
    )

    # Gripper control
    self.gripper_state_pub = self.create_publisher(
        JointState, f"{self.namespace}/gripper_state", 10
    )
    self.gripper_command_sub = self.create_subscription(
        JointState,
        f"{self.namespace}/gripper_cmd",
        self.gripper_cmd_callback,
        10,
    )

    # Service servers
    self.create_service(
        Trigger, f"{self.namespace}/reset", self.handle_reset  # type: ignore
    )
    self.create_service(
        Trigger, f"{self.namespace}/enable", self.handle_enable  # type: ignore
    )
    self.create_service(
        Trigger, f"{self.namespace}/disable", self.handle_disable  # type: ignore
    )
    self.create_service(
        Trigger, f"{self.namespace}/get_status", self.handle_get_status  # type: ignore
    )
    self.create_service(
        Trigger, f"{self.namespace}/is_enabled", self.handle_is_enabled  # type: ignore
    )
    self.create_service(
        Trigger, f"{self.namespace}/go_to_rest", self.handle_go_to_rest  # type: ignore
    )
    self.create_service(
        Trigger, f"{self.namespace}/go_to_down", self.handle_go_to_down  # type: ignore
    )

    self.get_logger().info(
        f"Piper control node started with CAN ID: {self.can_port}"
    )

    # Timer to periodically publish joint states
    self.create_timer(0.01, self.publish_joint_states)
    self.create_timer(0.01, self.publish_end_effector_pose)
    self.create_timer(0.01, self.publish_gripper_state)

  def joint_cmd_callback(self, msg):
    positions = msg.position
    self.get_logger().debug(f"Received joint positions: {positions}")
    self.robot.set_joint_positions(positions)

  def publish_joint_states(self):
    joint_positions = self.robot.get_joint_positions()
    joint_velocities = self.robot.get_joint_velocities()
    joint_efforts = self.robot.get_joint_efforts()
    msg = JointState()
    msg.position = list(joint_positions)
    msg.velocity = list(joint_velocities)
    msg.effort = list(joint_efforts)
    self.joint_state_pub.publish(msg)

  def cartesian_cmd_callback(self, msg: Pose) -> None:
    quat = [
        msg.orientation.w,
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
    ]
    rpy = tr.quat_to_euler(quat, ordering="XYZ")
    rpy = np.array(rpy)
    pose = [
        msg.position.x,
        msg.position.y,
        msg.position.z,
        rpy[0],
        rpy[1],
        rpy[2],
    ]
    self.get_logger().debug(f"Received Cartesian position: {pose}")
    self.robot.set_cartesian_position(pose)

  def publish_end_effector_pose(self):
    ee_pose = self.robot.get_end_effector_pose()
    quat = tr.euler_to_quat(
        [ee_pose[3], ee_pose[4], ee_pose[5]], ordering="XYZ"
    )
    quat = np.array(quat)
    msg = Pose()
    msg.position.x = ee_pose[0]
    msg.position.y = ee_pose[1]
    msg.position.z = ee_pose[2]
    msg.orientation.w = quat[0]
    msg.orientation.x = quat[1]
    msg.orientation.y = quat[2]
    msg.orientation.z = quat[3]
    self.cartesian_position_pub.publish(msg)

  def gripper_cmd_callback(self, msg: JointState) -> None:
    position = msg.position[0] if msg.position else 0.0
    effort = msg.effort[0] if msg.effort else 0.0
    self.robot.set_gripper_ctrl(position, effort)
    self.get_logger().debug(
        f"Gripper control command received: Position={position}, Effort={effort}"
    )

  def publish_gripper_state(self):
    position, effort = self.robot.get_gripper_state()

    msg = JointState()
    msg.position = [position]
    msg.effort = [effort]

    self.gripper_state_pub.publish(msg)
    self.get_logger().debug(
        f"Published gripper state: Position={position}, Effort={effort}"
    )

  def handle_reset(
      self, request: Trigger.Request, response: Trigger.Response
  ) -> Trigger.Response:
    del request
    try:
      self.robot.reset()
      response.success = True
      response.message = "Robot reset."
    except RuntimeError as e:
      print(f"Error resetting robot: {e}")
      response.message = "Robot not reset."
      response.success = False
    return response

  def handle_enable(
      self, request: Trigger.Request, response: Trigger.Response
  ) -> Trigger.Response:
    del request
    self.robot.enable()
    response.success = True
    response.message = "Robot enabled."
    return response

  def handle_disable(
      self, request: Trigger.Request, response: Trigger.Response
  ) -> Trigger.Response:
    del request
    self.robot.disable()
    response.success = True
    response.message = "Robot disabled."
    return response

  def handle_get_status(
      self, request: Trigger.Request, response: Trigger.Response
  ) -> Trigger.Response:
    del request
    status = self.robot.get_status()
    response.success = True
    response.message = f"Robot status: {status}"
    return response

  def handle_is_enabled(
      self, request: Trigger.Request, response: Trigger.Response
  ) -> Trigger.Response:
    del request
    is_enabled = self.robot.is_enabled()
    response.success = True
    response.message = f"Robot enabled: {is_enabled}"
    return response

  def handle_go_to_rest(
      self, request: Trigger.Request, response: Trigger.Response
  ) -> Trigger.Response:
    del request
    self.robot.set_joint_positions(piper_control.REST_POSITION)
    response.success = True
    response.message = "Moved to REST position."
    return response

  def handle_go_to_down(
      self, request: Trigger.Request, response: Trigger.Response
  ) -> Trigger.Response:
    del request
    self.robot.set_joint_positions(piper_control.DOWN_POSITION)
    response.success = True
    response.message = "Moved to DOWN position."
    return response


def main(args=None):
  rclpy.init(args=args)
  node = PiperControlNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
