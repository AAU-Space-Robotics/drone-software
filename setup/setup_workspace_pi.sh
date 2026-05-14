#!/bin/bash

# Get the absolute path of the script's directory (where the script is located)
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Set the ROS 2 workspace path and parent directory
ROS_WORKSPACE_PATH=$(dirname "$SCRIPT_DIR")  # Base workspace is drone-software (parent of setup/)
SRC_DIR="$ROS_WORKSPACE_PATH/src"
PARENT_DIR=$(dirname "$ROS_WORKSPACE_PATH")  # Directory containing drone-software
ROS_DISTRO="jazzy"

# Check if ROS 2 is installed
if ! command -v ros2 &> /dev/null; then
    echo "WARNING: ROS 2 does not appear to be installed (ros2 command not found)."
    echo "This script requires ROS 2 $ROS_DISTRO to be installed and sourced before running."
    echo ""
    read -p "Are you sure you want to continue anyway? [y/N] " confirm
    if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
        echo "Aborting. Please install ROS 2 $ROS_DISTRO first."
        exit 1
    fi
    echo "Continuing at your own risk..."
else
    echo "ROS 2 found. Continuing..."
fi

# Install Intel RealSense SDK
echo "Installing Intel RealSense SDK..."
if ! command -v rs-enumerate-devices &> /dev/null; then
    # Install dependencies
    sudo apt-get update
    sudo apt-get install -y \
        build-essential \
        cmake \
        libglfw3-dev \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        libusb-1.0-0-dev \
        libssl-dev \
        pkg-config \
        python3 \
        python3-dev \
        linux-headers-$(uname -r)

    # Clone and build librealsense
    cd /tmp
    git clone https://github.com/IntelRealSense/librealsense.git
    cd librealsense
    mkdir build && cd build
    cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=Release
    make -j$(nproc)
    sudo make install

    # Apply kernel patches
    cd ..
    sudo scripts/patch-realsense-ubuntu-lts.sh

    # Verify installation
    if ! command -v rs-enumerate-devices &> /dev/null; then
        echo "Error: Failed to install Intel RealSense SDK"
        exit 1
    fi
else
    echo "Intel RealSense SDK is already installed."
fi

# Install ROS RealSense wrapper
echo "Installing ROS $ROS_DISTRO RealSense wrapper..."
if ! dpkg -l | grep -q "ros-$ROS_DISTRO-realsense2-camera"; then
    # Create keyrings directory
    sudo mkdir -p /etc/apt/keyrings

    # Remove any existing ROS 2 repository files and keys to avoid conflicts
    sudo rm -f /etc/apt/sources.list.d/ros2*.list
    sudo rm -f /etc/apt/sources.list.d/ros2*.sources
    sudo rm -f /etc/apt/keyrings/ros2*.gpg

    # Add ROS 2 repository key
    if ! curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo tee /etc/apt/keyrings/ros2-latest.gpg > /dev/null; then
        echo "Error: Failed to add ROS 2 repository key"
        exit 1
    fi

    # Add ROS 2 repository with explicit architecture
    echo "deb [arch=arm64 signed-by=/etc/apt/keyrings/ros2-latest.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" | \
        sudo tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null
    if [ $? -ne 0 ]; then
        echo "Error: Failed to add ROS 2 repository"
        exit 1
    fi

    # Set correct permissions
    sudo chmod 644 /etc/apt/sources.list.d/ros2-latest.list
    sudo chmod 644 /etc/apt/keyrings/ros2-latest.gpg

    # Clear apt cache
    sudo apt-get clean
    sudo rm -rf /var/lib/apt/lists/*

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

# Install additional ROS dependencies
echo "Installing additional ROS $ROS_DISTRO dependencies..."
if ! dpkg -l | grep -q "ros-$ROS_DISTRO-serial-driver"; then
    if ! sudo apt-get install -y "ros-$ROS_DISTRO-serial-driver"; then
        echo "Error: Failed to install ros-$ROS_DISTRO-serial-driver"
        exit 1
    fi
else
    echo "ros-$ROS_DISTRO-serial-driver is already installed."
fi

if ! dpkg -l | grep -q "ros-$ROS_DISTRO-asio-cmake-module"; then
    if ! sudo apt-get install -y "ros-$ROS_DISTRO-asio-cmake-module"; then
        echo "Error: Failed to install ros-$ROS_DISTRO-asio-cmake-module"
        exit 1
    fi
else
    echo "ros-$ROS_DISTRO-asio-cmake-module is already installed."
fi

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

# Clone the Intel RealSense ROS wrapper repository
clone_repo_if_not_exists "git@github.com:IntelRealSense/realsense-ros.git" "realsense-ros" "ros2-master"

# Clone the px4_msgs repository
clone_repo_if_not_exists "git@github.com:AAU-Space-Robotics/px4_msgs_thyra.git" "px4_msgs_thyra"

# Navigate to the parent directory to check/install Micro-XRCE-DDS-Agent
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

# Grant UART access to the current user
echo "Granting dialout group access for UART..."
sudo usermod -a -G dialout "${SUDO_USER:-$USER}"

# Pin the flight stack to the Pi launch file
PARAMS_SRC="${ROS_WORKSPACE_PATH}/src/thyra/config/thyra_params.yaml"
sed -i "s/flight_stack_launch: .*/flight_stack_launch: thyra_pi/" "$PARAMS_SRC"

# Install the thyra systemd service and restricted sudo rules
SERVICE_USER="${SUDO_USER:-$USER}"
SYSTEMCTL_PATH="$(command -v systemctl)"
REBOOT_PATH="$(command -v reboot)"
ROS_SETUP_FILE="/opt/ros/${ROS_DISTRO}/setup.bash"
WORKSPACE_SETUP_FILE="${ROS_WORKSPACE_PATH}/install/setup.bash"
PARAMS_INSTALLED="${ROS_WORKSPACE_PATH}/install/thyra/share/thyra/config/thyra_params.yaml"
SERVICE_FILE="/etc/systemd/system/thyra.service"
SUDOERS_FILE="/etc/sudoers.d/thyra-system-manager"

if [ -z "$SYSTEMCTL_PATH" ] || [ -z "$REBOOT_PATH" ]; then
        echo "Error: systemctl or reboot command not found."
        exit 1
fi

if [ ! -f "$WORKSPACE_SETUP_FILE" ]; then
        echo "Note: Workspace has not been built yet — building now."
fi

echo "Installing thyra systemd service..."
cat <<EOF | sudo tee "$SERVICE_FILE" > /dev/null
[Unit]
Description=Thyra ROS 2 stack
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${SERVICE_USER}
Group=${SERVICE_USER}
WorkingDirectory=${ROS_WORKSPACE_PATH}
Environment=PYTHONUNBUFFERED=1
ExecStart=/bin/bash -lc 'source ${ROS_SETUP_FILE} && source ${WORKSPACE_SETUP_FILE} && exec ros2 run thyra system_manager.py --ros-args -r __ns:=/asr/thyra --params-file ${PARAMS_INSTALLED}'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

echo "Installing restricted sudoers rules for system_manager..."
cat <<EOF | sudo tee "$SUDOERS_FILE" > /dev/null
${SERVICE_USER} ALL=(root) NOPASSWD: ${SYSTEMCTL_PATH} restart thyra, ${REBOOT_PATH}
EOF
sudo chmod 440 "$SUDOERS_FILE"
sudo systemctl daemon-reload
sudo systemctl enable thyra.service

# Build the workspace
echo "Building workspace..."
cd "$ROS_WORKSPACE_PATH" || exit

source "$ROS_SETUP_FILE"

colcon build \
    --parallel-workers 4 \
    --packages-skip asr_gcs

# Set ROS_DOMAIN_ID in .bashrc to ensure UAV and GCS are on the same ROS 2 domain for communication
echo 'export ROS_DOMAIN_ID=203' >> ~/.bashrc

echo "Workspace setup and build completed! :)"
