#!/bin/bash

# Get the absolute path of the script's directory (where the script is located)
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Set the ROS 2 workspace path and parent directory
ROS_WORKSPACE_PATH="$SCRIPT_DIR"  # Base workspace is drone-software
SRC_DIR="$ROS_WORKSPACE_PATH/src"
PARENT_DIR=$(dirname "$ROS_WORKSPACE_PATH")  # Directory containing drone-software

# Check if the src directory exists, create it if it doesn't
if [ ! -d "$SRC_DIR" ]; then
  echo "Creating src directory at $SRC_DIR..."
  mkdir -p "$SRC_DIR"
fi

# Navigate to the src directory
cd "$SRC_DIR" || exit

# Function to clone a repository if it doesn't already exist
clone_repo_if_not_exists() {
  local repo_url="$1"
  local repo_name=$(basename "$repo_url" .git)

  if [ -d "$repo_name" ]; then
    echo "$repo_name already exists, skipping clone."
  else
    echo "Cloning $repo_name..."
    git clone "$repo_url"
  fi
}

# Clone the px4_msgs repository
clone_repo_if_not_exists "https://github.com/PX4/px4_msgs.git"

# Navigate to the parent directory to check/install Micro-XRCE-DDS-Agent and PX4-Autopilot
cd "$PARENT_DIR" || exit

# Check and install Micro-XRCE-DDS-Agent
if [ -d "Micro-XRCE-DDS-Agent" ]; then
  echo "Micro-XRCE-DDS-Agent is already installed, skipping installation."
else
  echo "Installing Micro-XRCE-DDS-Agent..."
  git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
  cd Micro-XRCE-DDS-Agent
  mkdir build
  cd build
  cmake ..
  make
  sudo make install
  sudo ldconfig /usr/local/lib/
  cd "$PARENT_DIR" || exit
fi

# Check and install PX4-Autopilot
if [ -d "PX4-Autopilot" ]; then
  echo "PX4-Autopilot is already installed, skipping installation."
else
  echo "Installing PX4-Autopilot..."
  git clone https://github.com/PX4/PX4-Autopilot.git --recursive
  cd PX4-Autopilot
  bash ./Tools/setup/ubuntu.sh
  make px4_sitl
  cd "$PARENT_DIR" || exit
fi

echo "Workspace setup completed! Ready for build. Please build it now to use it :)"