# Piper Control Node

## Overview

The Piper Control Node is a ROS2 node for controlling the Piper robot arm. It
provides interfaces for both joint control and Cartesian control and exposes
several service interfaces for enabling/disabling the robot, resetting it, and
moving it to predefined positions.

## Features

-   Publishes joint states and end-effector pose.
-   Accepts joint position and Cartesian position commands.
-   Provides service interfaces for enabling/disabling, resetting, and moving
  the robot to specific positions.

## Running the Node

Before running the node, make sure your ROS2 workspace is properly set up and
sourced. See piper_ros/README.md for details.

### Connecting Robots

You can use `piper_connect.activate()` with no args to activate all robots
plugged into the computer:

```python
piper_connect.activate()
>>> ['can1', 'can0']
```

To activate specific robots use `piper_connect.find_ports()`:

```python
piper_connect.find_ports()
>>> [('can1', '1-3:1.0'), ('can0', '3-2:1.0')]
```

### Launching the Node

To start the Piper Control Node:

```bash
ros2 run piper_control_node piper_control_node
```

To start an arm under the namespace "piper":

```bash
ros2 run piper_control_node piper_control_node --ros-args -p namespace:=piper
```

### Viewing Topics

To see the list of active topics:

```bash
ros2 topic list
```

### Echoing Topic Messages

For example, to view the joint states:

```bash
ros2 topic echo /piper/joint_states
```

## Topics

### Joint Position Commands

Command the robot joints to a desired position:

```bash
ros2 topic pub /piper/joint_positions_cmd sensor_msgs/JointState "{position:
[0.0, 0.5, -0.5, 0.0, 1.0, 0.0]}"
```

### Cartesian Position Commands

Command the end effector to a Cartesian position:

```bash
ros2 topic pub /piper/cartesian_position_cmd geometry_msgs/Pose "{position: {x:
0.5, y: 0.0, z: 0.2}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}"
```

### Gripper Control Commands

Control the gripper position and effort using the new `JointState` message:

```bash
ros2 topic pub /piper/gripper_control sensor_msgs/JointState "{position:
[0.05], effort: [20.0]}"
```

### Viewing the Joint and Gripper States

To echo the current joint states:

```bash
ros2 topic echo /piper/joint_states
```

To echo the gripper state:

```bash
ros2 topic echo /piper/gripper_state
```

## Services

The Piper Control Node also exposes several services for controlling and
querying the robot state:

### Available Services

-   `/piper/reset` (`std_srvs/Trigger`): Resets the robot.
-   `/piper/enable` (`std_srvs/Trigger`): Enables the robot.
-   `/piper/disable` (`std_srvs/Trigger`): Disables the robot.
-   `/piper/get_status` (`std_srvs/Trigger`): Retrieves the robot's status.
-   `/piper/is_enabled` (`std_srvs/Trigger`): Checks if the robot is enabled.
-   `/piper/go_to_rest` (`std_srvs/Trigger`): Moves the robot to the REST
  position.
-   `/piper/go_to_down` (`std_srvs/Trigger`): Moves the robot to the DOWN
  position.

### Sending Service Requests

To invoke services, use the following commands:

#### Reset the Robot

```bash
ros2 service call /piper/reset std_srvs/srv/Trigger
```

#### Enable the Robot

```bash
ros2 service call /piper/enable std_srvs/srv/Trigger
```

#### Disable the Robot

```bash
ros2 service call /piper/disable std_srvs/srv/Trigger
```

#### Get Robot Status

```bash
ros2 service call /piper/get_status std_srvs/srv/Trigger
```

#### Check if the Robot is Enabled

```bash
ros2 service call /piper/is_enabled std_srvs/srv/Trigger
```

#### Move to REST Position

```bash
ros2 service call /piper/go_to_rest std_srvs/srv/Trigger
```

#### Move to DOWN Position

```bash
ros2 service call /piper/go_to_down std_srvs/srv/Trigger
```

## Troubleshooting

If the node does not start or cannot find packages, ensure that your ROS2
environment is sourced and your Conda environment is activated before running
the node.
