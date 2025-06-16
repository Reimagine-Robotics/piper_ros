"""
Piper control node for the Piper robot.
"""

import argparse
import functools
import os
import signal

from ament_index_python.packages import get_package_share_directory
import rclpy

from piper_control import piper_connect
from piper_control import piper_init
from piper_control import piper_interface
from piper_control import piper_control

from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class PiperControlNode(Node):
  """
  ROS2 node for controlling the Piper robot.
  """

  def __init__(self, gravity_model_path: str | None = None):
    """PiperControlNode constructor.

    Args:
      gravity_model_path: The path to the gravity model json file. By providing
        this, you will also enable teach mode. NOTE: You will need to have
        MUJOCO and Scipy installed as these are required for the custom gravity
        compensation using in the custom teach-mode.
    """

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

    if not ports:
      raise ValueError(
          "No ports found. Make sure the Piper is connected and turned on. "
          "If you are having issues connecting to the piper, check our "
          "troubleshooting guide @ "
          "https://github.com/Reimagine-Robotics/piper_control/blob/main/README.md"
      )

    self._robot = piper_interface.PiperInterface(can_port=self.can_port)

    self._arm_controller = piper_control.MitJointPositionController(
        self._robot,
        kp_gains=10.0,  # TODO: make this a parameter of the node.
        kd_gains=0.8,  # TODO: make this a parameter of the node.
        rest_position=piper_control.REST_POSITION,
    )

    self._gripper_controller = piper_control.GripperController(self._robot)

    self.namespace = (
        self.declare_parameter("namespace", "piper")
        .get_parameter_value()
        .string_value
    )

    # Joint control
    self.joint_state_pub = self.create_publisher(
        JointState,
        f"{self.namespace}/joint_states",
        qos_profile=10,
    )
    self.joint_command_sub = self.create_subscription(
        JointState,
        f"{self.namespace}/joint_commands",
        self.joint_cmd_callback,
        qos_profile=10,
    )

    # Gripper control
    self.gripper_state_pub = self.create_publisher(
        JointState,
        f"{self.namespace}/gripper_state",
        qos_profile=10,
    )
    self.gripper_command_sub = self.create_subscription(
        JointState,
        f"{self.namespace}/gripper_command",
        self.gripper_cmd_callback,
        qos_profile=10,
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

    self.get_logger().info(
        f"Piper control node started with CAN ID: {self.can_port}"
    )

    # Timer to periodically publish joint states
    self.create_timer(0.005, self.publish_joint_states)
    self.create_timer(0.005, self.publish_gripper_state)

    # Put teach mode behind a flag as it requires additional libraries be
    # installed (MUJOCO and Scipy).
    if gravity_model_path:
      print(f"Teach Mode available, using gravity model: {gravity_model_path}")

      from piper_control_node.teach_mode import teach_mode

      self.create_service(
          Trigger,
          f"{self.namespace}/teach_mode_enable",
          self.handle_teach_mode_enable,  # type: ignore
      )
      self.create_service(
          Trigger,
          f"{self.namespace}/teach_mode_disable",
          self.handle_teach_mode_disable,  # type: ignore
      )

      # Get the path to the share directory of your package
      package_share_directory = get_package_share_directory(
          "piper_control_node"
      )

      # Construct the full path to your XML file
      piper_model_file_path = os.path.join(
          package_share_directory, "data", "piper_grav_comp.xml"
      )

      self._teach_controller = teach_mode.TeachController(
          self._robot,
          self._arm_controller,
          piper_model_file_path,
          gravity_model_path,
      )
      self._teach_mode_active = False
      self._teach_mode_timer = self.create_timer(
          0.005, self.teach_mode, autostart=False
      )
    else:
      print("Teach Mode unavailable, no gravity model path provided")

  def clean_stop(self) -> None:
    self._arm_controller.stop()
    self._gripper_controller.stop()

    piper_init.disable_arm(self._robot)
    piper_init.disable_gripper(self._robot)

  def joint_cmd_callback(self, msg):
    positions = list(msg.position)
    velocities = list(msg.velocity)
    efforts = list(msg.effort)

    if positions:
      if velocities or efforts:
        self.get_logger().warn(
            "Received joint positions, but also velocities or efforts"
        )

      self.get_logger().debug(f"Received joint positions: {positions}")
      self._arm_controller.command_joints(positions)

    elif velocities:
      self.get_logger().warn("Velocity actuation not currently supported")

    elif efforts:
      if positions or velocities:
        self.get_logger().warn(
            "Received joint efforts, but also positions or velocities"
        )

      self.get_logger().debug(f"Received joint efforts: {efforts}")
      self._arm_controller.command_torques(efforts)

    else:
      self.get_logger().warn(f"Received invalid joint command: {msg}")

  def publish_joint_states(self):
    joint_positions = self._robot.get_joint_positions()
    joint_velocities = self._robot.get_joint_velocities()
    joint_efforts = self._robot.get_joint_efforts()
    msg = JointState()
    msg.position = list(joint_positions)
    msg.velocity = list(joint_velocities)
    msg.effort = list(joint_efforts)
    self.joint_state_pub.publish(msg)

  def gripper_cmd_callback(self, msg: JointState) -> None:
    position = msg.position[0] if msg.position else 0.0
    effort = msg.effort[0] if msg.effort else 0.0

    self._gripper_controller.command_position(position, effort)

    self.get_logger().debug(
        f"Gripper command received: Position={position}, Effort={effort}"
    )

  def publish_gripper_state(self):
    position, effort = self._robot.get_gripper_state()

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

  def handle_enable(
      self, request: Trigger.Request, response: Trigger.Response
  ) -> Trigger.Response:
    del request

    piper_init.enable_arm(
        self._robot,
        arm_controller=piper_interface.ArmController.MIT,
        move_mode=piper_interface.MoveMode.MIT,
    )
    piper_init.enable_gripper(self._robot)

    self._arm_controller.start()
    self._gripper_controller.start()

    response.success = True
    response.message = "Robot enabled."
    return response

  def handle_disable(
      self, request: Trigger.Request, response: Trigger.Response
  ) -> Trigger.Response:
    del request

    self._arm_controller.stop()
    self._gripper_controller.stop()

    piper_init.disable_arm(self._robot)
    piper_init.disable_gripper(self._robot)

    response.success = True
    response.message = "Robot disabled."
    return response

  def handle_get_status(
      self, request: Trigger.Request, response: Trigger.Response
  ) -> Trigger.Response:
    del request
    status = self._robot.get_arm_status()
    response.success = True
    response.message = f"Robot status: {status}"
    return response

  def handle_is_enabled(
      self, request: Trigger.Request, response: Trigger.Response
  ) -> Trigger.Response:
    del request
    is_enabled = self._robot.is_enabled()
    response.success = True
    response.message = f"Robot enabled: {is_enabled}"
    return response

  def handle_teach_mode_enable(
      self, request: Trigger.Request, response: Trigger.Response
  ) -> Trigger.Response:
    del request

    self._teach_mode_active = True
    self._teach_mode_timer.reset()

    response.success = True
    response.message = "Teach mode enabled."
    return response

  def handle_teach_mode_disable(
      self, request: Trigger.Request, response: Trigger.Response
  ) -> Trigger.Response:
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


def term_handler(signum, frame, node: PiperControlNode) -> None:
  del frame
  node.get_logger().info(f"Stoping node on signal: {signum}")
  node.clean_stop()
  node.destroy_node()
  rclpy.shutdown()


def main(args=None):
  # Parse the gravity model argument for teach mode.
  parser = argparse.ArgumentParser(
      prog="PiperControlNode", description="ROS2 node for Piper control."
  )
  parser.add_argument(
      "-gp",
      "--gravity_path",
      help="Path to the gravity data gatherd by teach_mode/run_gather_data.py",
  )
  cargs, _ = parser.parse_known_args()

  rclpy.init(args=args)

  # Create the PiperControlNode. If a gravity model path is provided via a
  # command line argument, this will also enable teach mode for the ROS node.
  node = PiperControlNode(gravity_model_path=cargs.gravity_path)

  # Register signal handlers to prevent banging the arm on a Ctrl-C.
  signal.signal(signal.SIGINT, functools.partial(term_handler, node=node))
  signal.signal(signal.SIGTERM, functools.partial(term_handler, node=node))

  rclpy.spin(node)


if __name__ == "__main__":
  main()
