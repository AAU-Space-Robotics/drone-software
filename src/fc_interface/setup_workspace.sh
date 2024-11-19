#!/bin/bash

#chmod +x ~/drone/src/setup_workspace.sh

# Get the absolute path of the script's directory (where the script is located)
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Set the ROS 2 workspace path based on the script's location
ROS_WORKSPACE_PATH=$(realpath "$SCRIPT_DIR/..")  # Moves up one level from 'src' to 'drone'
SRC_DIR="$ROS_WORKSPACE_PATH/src"

# Check if the workspace exists
if [ ! -d "$ROS_WORKSPACE_PATH" ]; then
  echo "Workspace $ROS_WORKSPACE_PATH does not exist. Please create it first."
  exit 1
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

# Clone the repositories
clone_repo_if_not_exists "https://github.com/PX4/px4_msgs.git"
clone_repo_if_not_exists "https://github.com/PX4/px4_ros_com.git"


echo "Workspace setup completed! Ready for build. Please build it now to use it :)"
