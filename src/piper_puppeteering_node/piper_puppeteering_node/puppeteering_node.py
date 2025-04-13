"""Simple puppeteering node that relays joint positions and gripper state."""

import rclpy
from piper_control import piper_control
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PuppeteeringNode(Node):
  """ROS2 node for puppeteering the Piper robot."""

  def __init__(self):
    super().__init__("piper_puppeteering_node")

    # Declare parameters with default values
    self.declare_parameter("puppet_ns", "robot1")
    self.declare_parameter("puppeteer_ns", "robot0")

    # Get the actual parameter values
    self.puppet_ns = (
        self.get_parameter("puppet_ns").get_parameter_value().string_value
    )
    self.puppeteer_ns = (
        self.get_parameter("puppeteer_ns").get_parameter_value().string_value
    )

    self.get_logger().info(f"Puppet namespace: {self.puppet_ns}")
    self.get_logger().info(f"Puppeteer namespace: {self.puppeteer_ns}")

    # Subscribe to puppeteer robot joint states
    self.puppeteer_joint_sub = self.create_subscription(
        JointState,
        f"{self.puppeteer_ns}/joint_states",
        self.joint_callback,
        10,
    )

    # Subscribe to puppeteer robot gripper state
    self.puppeteer_gripper_sub = self.create_subscription(
        JointState,
        f"{self.puppeteer_ns}/gripper_state",
        self.gripper_callback,
        10,
    )

    # Publish joint commands to the puppet robot
    self.puppet_joint_pub = self.create_publisher(
        JointState, f"{self.puppet_ns}/joint_positions_cmd", 10
    )

    # Publish gripper commands to the puppet robot
    self.puppet_gripper_pub = self.create_publisher(
        JointState, f"{self.puppet_ns}/gripper_cmd", 10
    )

  def joint_callback(self, msg: JointState) -> None:
    """Callback function for puppeteer joint states."""
    print(f"Relaying joint states: {msg}")
    self.puppet_joint_pub.publish(msg)

  def gripper_callback(self, msg: JointState) -> None:
    """Callback function for puppeteer gripper state."""
    print(f"Relaying gripper state: {msg}")
    msg.effort = [piper_control.GRIPPER_EFFORT_MAX]
    self.puppet_gripper_pub.publish(msg)


def main(args=None):
  rclpy.init(args=args)
  node = PuppeteeringNode()
  rclpy.spin(node)
  rclpy.shutdown()


if __name__ == "__main__":
  main()
