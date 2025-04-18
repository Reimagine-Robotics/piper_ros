"""Launches two robots on can0 and can1 and puppeteering 0 -> 1.

Usage (from the root of the ROS workspace):
>>> ros2 launch piper_puppeteering_node puppeteering_launch.py
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
  return LaunchDescription(
      [
          Node(
              package="piper_control_node",
              executable="piper_control_node",
              name="piper_control_node_0",
              parameters=[{"namespace": "robot0"}, {"can_port": "can0"}],
          ),
          Node(
              package="piper_control_node",
              executable="piper_control_node",
              name="piper_control_node_1",
              parameters=[{"namespace": "robot1"}, {"can_port": "can1"}],
          ),
          Node(
              package="piper_puppeteering_node",
              executable="piper_puppeteering_node",
              name="piper_puppeteering_node",
              parameters=[{"puppet_ns": "robot0"}, {"puppeteer_ns": "robot1"}],
          ),
          # Delay to allow nodes to start
          TimerAction(
              period=2.0,  # Wait for 2 seconds before calling the service
              actions=[
                  # Enable robot0
                  ExecuteProcess(
                      cmd=[
                          "ros2",
                          "service",
                          "call",
                          "/robot0/enable",
                          "std_srvs/srv/Trigger",
                          "{}",
                      ],
                      output="screen",
                  ),
                  # Enable robot1
                  ExecuteProcess(
                      cmd=[
                          "ros2",
                          "service",
                          "call",
                          "/robot1/enable",
                          "std_srvs/srv/Trigger",
                          "{}",
                      ],
                      output="screen",
                  ),
              ],
          ),
      ]
  )
