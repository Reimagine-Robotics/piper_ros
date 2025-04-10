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

### 🔧 Creating Conda-Compatible ROS2 Nodes

ROS2 nodes use the system-wide python in /usr/bin/python by default. To trick
ROS into using the overlaid python version in a Conda env we need the following
pattern:

1.  Create a my_package/scripts/run_my_package file containing the following
   (modify accordingly):

    ```python
    #!/usr/bin/env python

    from piper_control_node.piper_control_node import main

    if __name__ == "__main__":
    main()
    ```

      The key part of this script is `#!/usr/bin/env python` which forces ros to
      run from the python defined in the bash env, rather than the system
      install.

2.  Add `run_my_package` to the package's setup.py:

    ```python
    data_files=[
    ...
    (
        "lib/" + package_name,
        ["scripts/run_piper_puppeteering_node"],
    ),
    ...
    ]
    ```

3.  Build as usual:

    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```

4.  Run the wrapper script rather than the node directly:

      ```bash
      ros2 run piper_control_node run_piper_control_node
      ```
