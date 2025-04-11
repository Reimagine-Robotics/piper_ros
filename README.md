# piper_ros

ROS2 driver for AgileX Piper arms using the [R2
piper_control](https://github.com/Reimagine-Robotics/piper_control) interface

## Installation

## Manual Install

  1.  Install `can-utils`:

      ```bash
      sudo apt install -y can-utils
      ```

  2.  Install `piper_control`:

      ```bash
        pip install "piper_control @ git+https://github.com/Reimagine-Robotics/piper_control.git@main"
      ```

  3.  Install `piper_ros` to your system or active virtual/conda environment:

      ```bash
      pip install .
      ```

## Conda Install (dev and testing)

  Create a conda environment with all required deps:

  ```bash
  ./conda_install.sh
  ```

  This script performs edittable installs (i.e. `pip -e`) of piper_control and
  piper_ros so they can be developed together.

### 🔧 Gotchas

#### Creating Conda-Compatible ROS2 Nodes

When using a Conda environment, ROS2 nodes may fail to discover packages
installed in your active environment (https://github.com/ros2/ros2/issues/1469)

The solution that worked for us was to install colcon into the active conda env:

  ```bash
  pip install colcon-common-extensions
  ```
