#!/bin/bash

# Script to install piper_ros in a fresh conda environment
# with Python 3.12, ipykernel, and editable installs of piper_ros and piper_control.
#
# Usage:
#   ./conda_install.sh
#
# Assumes:
# - Conda is installed and in PATH.
# - You have sudo access.
# - You run this from the root of the piper_ros repo.

set -e  # Exit on error

ENV_NAME="piper_ros_env"
YAML_FILE="piper_ros_env.yaml"

run_step() {
  "$@"
  local status=$?
  if [ $status -ne 0 ]; then
    echo "❌ Error: $* failed with status $status"
    exit $status
  fi
}

# Prompt for sudo early and keep it alive
echo "🔐 Requesting sudo access upfront..."
sudo -v
# Keep-alive: refresh sudo timestamp every minute in the background
while true; do sudo -n true; sleep 60; kill -0 "$$" || exit; done 2>/dev/null &

echo "🔧 Updating/creating conda environment: $ENV_NAME"
run_step conda env update -n "$ENV_NAME" --file "$YAML_FILE" --prune
echo "✅ Conda environment '$ENV_NAME' ready."

echo "🔧 Initializing conda shell"
run_step eval "$(conda shell.bash hook)"
echo "✅ Conda shell initialized."

echo "🔧 Activating environment: $ENV_NAME"
run_step conda activate "$ENV_NAME"
echo "✅ Environment '$ENV_NAME' activated."

echo "🔧 Installing system dependencies (can-utils)"
run_step sudo apt install -y can-utils
echo "✅ can-utils installed."

echo "🔧 Installing 'piper_control' in editable mode (DEV ONLY)"
run_step pip install -e ../piper_control
echo "✅ piper_control installed."

echo "🔧 Building ROS workspace with colcon"
run_step colcon build --symlink-install
echo "✅ ROS workspace built."

# Final usage reminders
cat <<EOF

🎉 Setup complete!

To get started:
  conda activate $ENV_NAME
  source install/setup.bash
  ros2 run piper_control_node run_piper_control_node

EOF
