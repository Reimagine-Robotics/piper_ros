# piper_ros

[![CI/CD](https://github.com/Reimagine-Robotics/piper_ros/actions/workflows/ci.yaml/badge.svg)](https://github.com/Reimagine-Robotics/piper_ros/actions/workflows/ci.yaml)

ROS2 driver for AgileX Piper arms using the [R2
piper_control](https://github.com/Reimagine-Robotics/piper_control) interface

## Installation

### pixi Install (dev and testing)

Make sure you have [pixi](https://pixi.sh/latest/#installation) installed.

```bash
pixi install
```

To build the ROS 2 packages

```bash
pixi run build
```

#### Linting

To lint the codebase, run:

```bash
pixi run lint
```

### Manual Install

  1.  Install `can-utils` and `ethtool`:

      ```bash
      sudo apt install -y can-utils ethtool
      ```

  2.  Install `piper_control`:

      ```bash
        pip install "piper_control @ git+https://github.com/Reimagine-Robotics/piper_control.git@main"
      ```

  3.  Install `piper_ros` to your system or active virtual/conda environment:

      ```bash
      pip install .
      ```
