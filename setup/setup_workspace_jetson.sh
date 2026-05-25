#!/bin/bash

# Get the absolute path of the script's directory (where the script is located)
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Set the ROS 2 workspace path and parent directory
ROS_WORKSPACE_PATH=$(dirname "$SCRIPT_DIR")  # Base workspace is drone-software (parent of setup/)
SRC_DIR="$ROS_WORKSPACE_PATH/src"
PARENT_DIR=$(dirname "$ROS_WORKSPACE_PATH")  # Directory containing drone-software
ROS_DISTRO="humble"

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
        python3-dev

    # Install kernel headers — Jetson ships a custom -tegra kernel whose headers
    # are not available as linux-headers-$(uname -r) in the standard Ubuntu repos.
    # Use nvidia-l4t-kernel-headers on Jetson, and the standard package elsewhere.
    if uname -r | grep -q "tegra"; then
        echo "Jetson kernel detected. Installing nvidia-l4t-kernel-headers..."
        sudo apt-get install -y nvidia-l4t-kernel-headers
    else
        sudo apt-get install -y "linux-headers-$(uname -r)"
    fi

    # Clone and build librealsense
    cd /tmp
    git clone https://github.com/IntelRealSense/librealsense.git
    cd librealsense
    mkdir build && cd build
    # Note: CUDA support is available on Jetson — enable if needed with -DBUILD_WITH_CUDA=true
    cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=Release
    make -j$(nproc)
    sudo make install

    # Kernel patching is intentionally skipped on Jetson.
    # JetPack ships a pre-patched kernel that already includes the UVC and USB
    # support librealsense needs. The upstream patch script does not support
    # Ubuntu 22.04 (jammy) with the -tegra kernel and will fail if run.

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

# Install Jetson-specific PyTorch, torchvision, and ultralytics.
#
# Supports any JetPack 6.x device (L4T 36.x, aarch64).
# The jp/v61 PyTorch wheel is the latest NVIDIA release and works on
# L4T 36.2 through 36.5 (JetPack 6.0 – 6.2).
install_jetson_pytorch_stack() {
    local cuda_lib_path="/usr/local/cuda-12.6/targets/aarch64-linux/lib:/usr/local/cuda/targets/aarch64-linux/lib"
    local cupti_lib_path="/usr/local/cuda-12.6/extras/CUPTI/lib64:/usr/local/cuda/extras/CUPTI/lib64"
    local cusparselt_lib_path="/usr/local/cusparselt/lib"

    if ! command -v python3 &> /dev/null; then
        echo "WARNING: python3 not found; skipping Jetson PyTorch install."
        return 0
    fi

    # Only run on aarch64 (Jetson). On x86_64, torch+CUDA comes from PyPI.
    if [[ "$(uname -m)" != "aarch64" ]]; then
        echo "Not an aarch64 system; skipping Jetson-specific PyTorch install."
        return 0
    fi

    local l4t_version
    l4t_version="$(dpkg-query -W -f='${Version}' nvidia-l4t-core 2>/dev/null | cut -d- -f1)"
    local l4t_major
    l4t_major="$(echo "$l4t_version" | cut -d. -f1)"

    if [[ "$l4t_major" != "36" ]]; then
        echo "L4T $l4t_version is not a JetPack 6.x device (L4T 36.x required); skipping."
        return 0
    fi

    # Install CUDA runtime libraries that PyTorch depends on.
    if ! ldconfig -p | grep -q 'libnvToolsExt.so.1'; then
        echo "Installing CUDA NVTX support required by Jetson PyTorch..."
        sudo apt-get update -qq
        sudo apt-get install -y cuda-nvtx-12-6
        sudo ldconfig
    fi

    if ! ldconfig -p | grep -q 'libcupti.so.12'; then
        echo "Installing CUDA CUPTI support required by Jetson PyTorch..."
        sudo apt-get update -qq
        sudo apt-get install -y cuda-cupti-12-6
        sudo ldconfig
    fi

    if [ ! -e /usr/local/cusparselt/lib/libcusparseLt.so.0 ]; then
        echo "Installing cuSPARSELt support required by Jetson PyTorch..."
        local tmp_archive
        tmp_archive="$(mktemp /tmp/libcusparse_lt.XXXXXX.tar.xz)"
        curl -fsSL -o "$tmp_archive" \
            https://developer.download.nvidia.com/compute/cusparselt/redist/libcusparse_lt/linux-aarch64/libcusparse_lt-linux-aarch64-0.8.1.1_cuda12-archive.tar.xz
        sudo mkdir -p /usr/local/cusparselt
        sudo tar -xJf "$tmp_archive" -C /usr/local/cusparselt --strip-components=1
        rm -f "$tmp_archive"
    fi

    if [ ! -f /etc/ld.so.conf.d/cusparselt.conf ] || \
       ! grep -qx '/usr/local/cusparselt/lib' /etc/ld.so.conf.d/cusparselt.conf; then
        echo "Registering cuSPARSELt with the system dynamic loader..."
        echo '/usr/local/cusparselt/lib' | sudo tee /etc/ld.so.conf.d/cusparselt.conf > /dev/null
        sudo ldconfig
    fi

    export LD_LIBRARY_PATH="${cuda_lib_path}:${cupti_lib_path}:${cusparselt_lib_path}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

    # If a CUDA-enabled torch >= 2.3 is already present, nothing to do.
    if python3 - <<'PY' 2>/dev/null
import sys, torch
ok = torch.cuda.is_available() and \
     tuple(int(x) for x in torch.__version__.split('a')[0].split('.')[:2]) >= (2, 3)
sys.exit(0 if ok else 1)
PY
    then
        echo "CUDA-enabled torch >= 2.3 already installed ($(python3 -c 'import torch; print(torch.__version__)')); skipping PyTorch install."
    else
        echo "Installing NVIDIA Jetson PyTorch for L4T $l4t_version..."
        pip3 uninstall -y torch torchvision 2>/dev/null || true

        # jp/v61 is the latest NVIDIA release; compatible with L4T 36.2 – 36.5.
        local torch_url="https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl"
        if ! python3 -m pip install --no-cache-dir "$torch_url"; then
            echo "Error: Failed to install the NVIDIA Jetson PyTorch wheel"
            exit 1
        fi

        if ! python3 -m pip install --no-cache-dir --no-deps "torchvision==0.20.0"; then
            echo "Error: Failed to install torchvision 0.20.0"
            exit 1
        fi

        if ! python3 - <<'PY'
import torch
print(torch.__version__, torch.cuda.is_available())
if not torch.cuda.is_available():
    raise SystemExit(1)
PY
        then
            echo "Error: Jetson PyTorch verification failed"
            exit 1
        fi
    fi

    # ultralytics must be installed with --no-deps so pip does not replace the
    # Jetson-specific torch wheel with an incompatible PyPI build.
    if ! python3 -c "import ultralytics" 2>/dev/null; then
        echo "Installing ultralytics (no-deps to preserve Jetson torch)..."
        if ! python3 -m pip install --no-cache-dir --no-deps ultralytics; then
            echo "Error: Failed to install ultralytics"
            exit 1
        fi
        # ONNX is needed for the intermediate export step during engine generation.
        if ! python3 -m pip install --no-cache-dir --no-deps "onnx>=1.12.0,<2.0.0" "onnxslim>=0.1.71"; then
            echo "Error: Failed to install ONNX export dependencies"
            exit 1
        fi
    else
        echo "ultralytics already installed ($(python3 -c 'import ultralytics; print(ultralytics.__version__)'))."
    fi
}

# Install Python dependencies for perception
echo "Installing Python dependencies for asr_perception..."
install_jetson_pytorch_stack

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

# Disable nvgetty to free UART for PX4 link
echo "Disabling nvgetty serial console to free UART..."
sudo systemctl stop nvgetty 2>/dev/null || true
sudo systemctl disable nvgetty 2>/dev/null || true

# Grant UART access to the current user
echo "Granting dialout group access for UART..."
sudo usermod -a -G dialout "${SUDO_USER:-$USER}"

# Install udev rules for stable USB device symlinks
echo "Installing udev rules for USB device identification..."
UDEV_RULES_FILE="/etc/udev/rules.d/99-drone-usb.rules"
cat <<EOF | sudo tee "$UDEV_RULES_FILE" > /dev/null
# PX4 bridge (Silicon Labs CP2102N USB-to-UART)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="px4"

# SiK radio module (FTDI FT231X)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", SYMLINK+="sik"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger

# Pin the flight stack to the Jetson launch file
PARAMS_SRC="${ROS_WORKSPACE_PATH}/src/thyra/config/uav/thyra_params.yaml"
sed -i "s/flight_stack_launch: .*/flight_stack_launch: thyra_jetson/" "$PARAMS_SRC"

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
Environment=LD_LIBRARY_PATH=/usr/local/cuda-12.6/targets/aarch64-linux/lib:/usr/local/cuda/targets/aarch64-linux/lib:/usr/local/cuda-12.6/extras/CUPTI/lib64:/usr/local/cuda/extras/CUPTI/lib64:/usr/local/cusparselt/lib
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

chmod +x "$ROS_WORKSPACE_PATH/src/asr_perception/scripts/detect_probe.py"

source "$ROS_SETUP_FILE"

colcon build \
    --parallel-workers 4 \
    --packages-skip asr_gcs

# Set ROS_DOMAIN_ID in .bashrc to ensure UAV and GCS are on the same ROS 2 domain for communication
echo 'export ROS_DOMAIN_ID=203' >> ~/.bashrc


echo "Workspace setup and build completed! :)"