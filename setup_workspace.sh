#!/bin/bash

# Get the absolute path of the script's directory (where the script is located)
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Set the ROS 2 workspace path and parent directory
ROS_WORKSPACE_PATH="$SCRIPT_DIR"  # Base workspace is drone-software
SRC_DIR="$ROS_WORKSPACE_PATH/src"
PARENT_DIR=$(dirname "$ROS_WORKSPACE_PATH")  # Directory containing drone-software
ROS_DISTRO="humble"

# Check if ROS_DISTRO is set
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS distribution not found. Ensure ROS 2 is installed."
    exit 1
fi

# 1. Install Intel RealSense SDK
echo "Checking Intel RealSense SDK installation..."
if ! dpkg -l | grep -q librealsense2-dkms; then
    echo "Installing Intel RealSense SDK..."
    
    # Create keyrings directory if it doesn't exist
    sudo mkdir -p /etc/apt/keyrings
    
    # Add Intel RealSense repository key
    if ! curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null; then
        echo "Error: Failed to download RealSense PGP key"
        exit 1
    fi

    # Install apt-transport-https if not already installed
    if ! dpkg -l | grep -q apt-transport-https; then
        sudo apt-get update
        if ! sudo apt-get install -y apt-transport-https; then
            echo "Error: Failed to install apt-transport-https"
            exit 1
        fi
    fi

    # Add Intel RealSense repository
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | \
        sudo tee /etc/apt/sources.list.d/librealsense.list > /dev/null
    if [ $? -ne 0 ]; then
        echo "Error: Failed to add RealSense repository"
        exit 1
    fi

    # Update package lists
    if ! sudo apt-get update; then
        echo "Error: Failed to update package lists"
        exit 1
    fi

    # Install RealSense packages
    for package in librealsense2-dkms librealsense2-utils librealsense2-dev; do
        if ! sudo apt-get install -y "$package"; then
            echo "Error: Failed to install $package"
            exit 1
        fi
    done
else
    echo "Intel RealSense SDK is already installed."
fi

# 2. Install ROS RealSense wrapper
echo "Checking ROS RealSense wrapper installation..."
if ! dpkg -l | grep -q "ros-$ROS_DISTRO-realsense2-camera"; then
    echo "Installing ROS $ROS_DISTRO RealSense wrapper..."
    
    # Create keyrings directory if it doesn't exist
    sudo mkdir -p /etc/apt/keyrings
    
    # Add ROS 2 repository key
    if ! curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo tee /etc/apt/keyrings/ros2-latest.gpg > /dev/null; then
        echo "Error: Failed to add ROS 2 repository key"
        exit 1
    fi

    # Remove any existing ROS 2 repository files to avoid conflicts
    sudo rm -f /etc/apt/sources.list.d/ros2*.list
    echo "deb [signed-by=/etc/apt/keyrings/ros2-latest.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" | \
        sudo tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null
    if [ $? -ne 0 ]; then
        echo "Error: Failed to add ROS 2 repository"
        exit 1
    fi

    # Update package lists
    if ! sudo apt-get update; then
        echo "Error: Failed to update package lists for ROS packages"
        exit 1
    fi

    # Install ROS RealSense packages
    if ! sudo apt-get install -y "ros-$ROS_DISTRO-realsense2-camera" "ros-$ROS_DISTRO-realsense2-description"; then
        echo "Error: Failed to install ROS RealSense packages"
        exit 1
    fi
else
    echo "ROS $ROS_DISTRO RealSense wrapper is already installed."
fi

# 3. Check if the src directory exists, create it if it doesn't
if [ ! -d "$SRC_DIR" ]; then
  echo "Creating src directory at $SRC_DIR..."
  mkdir -p "$SRC_DIR"
fi

# 4. Navigate to the src directory
cd "$SRC_DIR" || exit

# 5. Function to clone a repository if it doesn't already exist
clone_repo_if_not_exists() {
    local repo_url="$1"
    local target_dir="$2"
    local branch="$3"
    if [ ! -d "$target_dir" ]; then
        echo "Cloning $repo_url into $target_dir..."
        if [ -n "$branch" ]; then
            git clone -b "$branch" "$repo_url" "$target_dir"
        else
            git clone "$repo_url" "$target_dir"
        fi
    else
        echo "$target_dir already exists, skipping clone."
    fi
}

# 6. Clone the Intel realsense wrapper repository
clone_repo_if_not_exists "git@github.com:IntelRealSense/realsense-ros.git" "realsense-ros" "ros2-master"

# 6.5. Initialize rosdep and install dependencies
# if ! command -v rosdep &> /dev/null; then
#   echo "Installing python3-rosdep..."
#   sudo apt-get update
#   sudo apt-get install python3-rosdep -y
# fi

# if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
#   echo "Initializing rosdep..."
#   sudo rosdep init
# fi

# echo "Updating rosdep..."
# rosdep update

# echo "Installing ROS 2 dependencies for workspace (skipping librealsense2)..."
# rosdep install -i --from-path . --rosdistro "$ROS_DISTRO" --skip-keys=librealsense2 -y

# 7. Clone the px4_msgs repository
clone_repo_if_not_exists "git@github.com:AAU-Space-Robotics/px4_msgs_thyra.git" "px4_msgs_thyra"

# 8. Navigate to the parent directory to check/install Micro-XRCE-DDS-Agent and PX4-Autopilot
cd "$PARENT_DIR" || exit

# 9. Check and install Micro-XRCE-DDS-Agent
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

# 10. Check and install PX4-Autopilot
if [ -d "PX4-Autopilot_thyra" ]; then
  echo "PX4-Autopilot_thyra is already installed, skipping installation."
else
  echo "Installing PX4-Autopilot..."
  git clone git@github.com:AAU-Space-Robotics/PX4-Autopilot_thyra.git --recursive
  cd PX4-Autopilot_thyra
  bash ./Tools/setup/ubuntu.sh
  make px4_sitl
  cd "$PARENT_DIR" || exit
fi

echo "Workspace setup completed! Ready for build. Please build it now to use it :)"