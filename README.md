# AAU Space Robotics: Drone
This repository contains the code for the AAU Space Robotics Drone.

# 1. General Setup of Systems

## Hardware Setup
Establish a serial connection between the Raspberry Pi 4 (RPi4) and the PX4 Cube Orange using the Telem 2 port (Telem 1 is for GPS). Use three jumper wires: female Dupont connectors for RPi4 GPIO pins and a 6-pin JST-GH connector for PX4 Telem 2.

### Pin Assignments
| **RPi4 Pin** | **PX4 Telem 2 Pin** | **Connection**         |
|-----------------------|---------------------|------------------------|
| 8                     | Pin 3               | RPi4 TX → PX4 RX       |
| 10                    | Pin 2               | RPi4 RX → PX4 TX       |
| 9                     | Pin 6               | RPi4 GND → PX4 GND     |

*Note:* PX4 Telem 2 uses a 6-pin JST-GH connector; pins 1, 4, and 5 are unused. For details, see [Cube Orange Overview](https://ardupilot.org/copter/docs/common-thecubeorange-overview.html).

## Software Setup

### Raspberry Pi Configuration
Before using the Raspberry Pi with ROS 2, you need to flash a microSD card with an operating system (OS) that the Pi can boot from. This section outlines the steps to set up a Raspberry Pi 4 with Ubuntu 22.04 Server (64-bit).

1. **Flash the MicroSD Card**
   - Download and install the [Raspberry Pi Imager](https://www.raspberrypi.com/software/) on your computer.
   - Insert a microSD card into your computer (via a card reader if needed).
   - Open the Raspberry Pi Imager and select the following:
     - **OS Category**: Choose "General-purpose OS."
     - **Operating System**: Select "Ubuntu" > "Ubuntu Server 22.04 (64-bit)."
       - *Important*: Use the 64-bit version. ROS 2 is not compatible with the 32-bit image.
   - Choose your microSD card as the storage device.
   - **Additional Configuration**: 
     - Click the cog icon (⚙️) in the Raspberry Pi Imager to access advanced settings. On Windows, press `Ctrl+Shift+X` to open this menu.
     - Configure the following options for convenience:
       - **Wi-Fi**: Set up a Wi-Fi connection by entering your network’s SSID and password.
       - **Username**: Create a user (e.g., `drone`) instead of the default.
       - **SSH**: Enable SSH access to allow remote connections.
     - Save these settings before proceeding.
   - Click "Write" to flash the image to the microSD card. This process may take a few minutes.

   *Why Ubuntu Server?* The server image lacks a desktop environment, which conserves resources on the Raspberry Pi. A graphical interface isn’t necessary for this ROS 2 application and can be replaced with tools like remote editors or desktops (see below).

2. **Recommended Tools for Interaction**
   - Since the server image has no GUI, use a code editor or remote desktop for a user-friendly experience:
     - **VS Code with Remote SSH**: Install [Visual Studio Code](https://code.visualstudio.com/docs/remote/ssh) on your computer and connect to the Pi via SSH for editing and debugging.
     - Alternatively, use a terminal-based editor (e.g., `nano` or `vim`) directly on the Pi via SSH.
   - These tools provide all necessary functionality without consuming extra Pi resources.

3. **Ensure `serial0` is free; disable conflicting services (e.g., console). Edit `/boot/firmware/cmdline.txt`:**
   ```
   sudo nano /boot/firmware/cmdline.txt
   ```
   Remove any `serial0` references. Save and exit.

4. **Edit `/boot/firmware/config.txt` and add:**  
   ```
   enable_uart=1
   ```

5. **Grant serial port access (Ubuntu restricts it by default):**  
   Add your user to the `dialout` group:  
   ```
   sudo usermod -a -G dialout $(whoami)
   ```

6. **Reboot the Raspberry Pi:**  
   ```
   sudo reboot
   ```

7. **Test the Serial Port:**  
   After rebooting, verify the serial port works:  
   - Connect pin 8 (TX) to pin 10 (RX) using a female-to-female DuPont wire to loopback the signal.  
   - Install Minicom:  
     ```
     sudo apt update && sudo apt install minicom -y
     ```
   - Open Minicom:  
     ```
     minicom -b 115200 -o -D /dev/ttyS0
     ```
   - In Minicom, press `Ctrl + A`, then `Z`, then `E` to enable echo. Type characters—they should appear on the screen if the port is working. If not, troubleshoot the connection or configuration.

8. **Install ROS 2 Humble:**  
   Follow the [official ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). Select the **Base** image (not Desktop, as visualization isn’t needed and Desktop consumes more resources). Include the developer tools during installation.

9. **Set Up Micro XRCE-DDS Agent:**  
   Follow the "Setup Micro XRCE-DDS Agent & Client" section in the [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide.html). Perform **only the Agent setup**—skip the Client step.
   *Note:* A version is specified in the download code. Consider updating to a newer version, such as the `master` branch, to avoid potential issues later.
   
### PX4 Configuration (Cube Orange)
To configure the flight controller (Cube Orange) with PX4, follow these steps:

1. **Download QGroundControl:**  
   Install QGroundControl from the [official guide](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html).

2. **Connect and Check Firmware:**  
   Connect the drone to your computer via micro-USB. In QGroundControl, go to **Vehicle Setup > Firmware**. Verify the firmware is PX4 v1.15.2 (update if needed).

3. **Configure Parameters:**  
   In **Vehicle Setup > Parameters**, set the following:
  - **Enable TELEM2:**
     ```
     MAV_1_CONFIG = TELEM 2
     ```
    Reboot the flight controller, and then proceed with the following steps:
   
   - **Telemetry on TELEM2:**  
     ```
     SER_TEL2_BAUD = 921600
     ```
   - **UXRCE-DDS on TELEM2:**  
     ```
     UXRCE_DDS_CFG = TELEM2
     ```

5. **Save and Reboot:**  
   Save the changes and restart the flight controller.

# 2. Using Packages in the Drone Software Workspace

This section explains how to set up and build the necessary packages in your workspace (e.g., `~/drone-software`) so you can control the drone. Following these instructions will install and make the package which has been written by the AAU space robotics team, usable.

## Step 1: Clone the Repositories

To get started, you’ll need to clone two key repositories onto your Raspberry Pi. These provide the drone control code and the message definitions for communication with the PX4 firmware via Micro XRCE-DDS.

### Clone the Drone Software Repository
Clone the main repository containing the drone control code. You can do this from any directory where you start your terminal:

```bash
git clone https://github.com/AAU-Space-Robotics/drone-software.git
```

Next, navigate into the workspace:

```bash
cd ~/drone-software
```

### Clone the PX4 Message Definitions
Inside the workspace, clone the `px4_msgs` repository, which defines the messages used to send data over the Micro XRCE-DDS agent to the PX4 firmware:

```bash
git clone -b release/1.15 https://github.com/PX4/px4_msgs.git
```

**Important Note:**  
The version of `px4_msgs` *must* match the PX4 firmware version on your flight controller. For this setup, use the `release/1.15` branch because it aligns with PX4 firmware v1.15.x (e.g., v1.15.2). If your firmware is a different version, ensure the `px4_msgs` branch or tag matches it exactly. Mismatched message definitions will cause communication errors, like thrust values overwriting quaternion fields!

## Step 2: Build the Workspace

With the repositories cloned, build the workspace to compile the code and message definitions.

Run this command from the root of the workspace (`~/drone-software`):

```bash
colcon build --packages-skip gcs
```
## Step 3: Source the Workspace

After building, source the workspace to make the compiled packages available in your terminal session:

```bash
source install/setup.bash
```

This step ensures your ROS 2 environment recognizes the drone software and message definitions. You’ll need to run this command in every new terminal session unless you add it to your `~/.bashrc` for automatic sourcing.

## fc_interface Package

Currently, `fc_interface` is the only package containing nodes that must be run directly on the onboard computer. This package includes a set of custom software stacks that enable drone control using the methods described below.

### 1. Establish Connection with the Flight Controller

For the `fc_interface` package to function, it requires the ROS 2 communication channel established by the `MicroXRCEAgent`. If the setup steps in Section 1 have not been performed, return and complete them. If setup is complete, use the following command to open the node, which publishes and subscribes to a specific set of ROS 2 topics:

```bash
MicroXRCEAgent serial --dev /dev/serial0 -b 921600
```

#### 1.1 (Optional) Establish Connection with a Simulation

To support drone stack development, a Gazebo simulation is available from the PX4 community. Read more here: [PX4](https://docs.px4.io/main/en/ros2/user_guide.html#install-px4). If the simulation is built correctly, navigate to its root directory (likely `PX4-Autopilot`) and run:

```bash
cd PX4-Autopilot/ && make px4_sitl gz_x500
```

As with the real flight controller, the `MicroXRCEAgent` is required. Start it with:

```bash
MicroXRCEAgent udp4 -p 8888
```

### 2. Ground Control Station

To operate either the real or simulated drone, a ground control station must communicate with it. This is where QGroundControl comes in. If the application is not installed, follow the setup guide in Section 1 to install it. Start it once installed.

### 3. Flight Control Interface Node (Custom Node from This Package)

With the preliminary setup complete, the interface node can be called. This allows the drone system to accept commands from the ROS 2 network based on custom modes. The drone must be armed before it can perform any actions. Use the following command to arm it:

```bash
ros2 action send_goal /thyra/in/drone_command interfaces/action/DroneCommand "{command_type: 'arm', target_pose: [], yaw: 0.0}"
```

Run this command from the workspace root (e.g., `~/drone-software`). Ensure the package is sourced with `source install/setup.bash` beforehand, or the ROS 2 network will not recognize the package. Once armed, send a new command within approximately 10 seconds, or the drone will disarm due to a PX4 safety feature (not implemented in this package). The following commands are currently supported:

- **Takeoff:**
   ```bash
   ros2 action send_goal /thyra/in/drone_command interfaces/action/DroneCommand "{command_type: 'takeoff', target_pose: [-2], yaw: 0.0}"
   ```
   - Initiates automatic takeoff to a specified altitude.
      - **target_pose**: A single value representing the desired altitude in meters (e.g., `-2` for 2 meters above the origin; negative values indicate upward movement).
      - **yaw**: Desired orientation in radians.
   - The command will be rejected if the altitude is not provided.

- **Go To:**
   ```bash
   ros2 action send_goal /thyra/in/drone_command interfaces/action/DroneCommand "{command_type: 'goto', target_pose: [1, 1, -2], yaw: 0.0}"
   ```
   - Moves the drone to the specified `[x, y, z]` position in meters.
      - **x, y, z**: Target coordinates. All three values are required.
      - **z**: Altitude, with negative values indicating upward movement (e.g., `-2` for 2 meters above the origin).
      - **yaw**: Desired orientation in radians (0 to 2π).
   - The command will be rejected if any coordinate is missing.

- **Spin:**
   ```bash
   ros2 action send_goal /thyra/in/drone_command interfaces/action/DroneCommand "{command_type: 'spin', target_pose: [1.57, 2.0, 0], yaw: 0.0}"
   ```
   - Requires three arguments in `target_pose`: `[yaw, num_rotations, use_longest_path]`.
      - **yaw**: Target orientation in radians (0 to 2π).
      - **num_rotations**: Number of full rotations to perform (0 for direct movement to target, 1 or more for additional rotations).
      - **use_longest_path**: Path selection (0 for shortest path, 1 for longest path).
   - The command will be rejected if any argument is missing.

- **Manual Mode:**
   ```bash
   ros2 action send_goal /thyra/in/drone_command interfaces/action/DroneCommand "{command_type: 'manual', target_pose: [], yaw: 0.0}"
   ```
   - Switches the drone to manual control mode. In this mode, the flight controller listens for manual input messages on the `drone/in/manual_input` topic.
   - Manual inputs can be sent from a node in the `gcs` package, such as those using a PS4 controller.

- **Set New Origin:**
   ```bash
   ros2 action send_goal /thyra/in/drone_command interfaces/action/DroneCommand "{command_type: 'set_origin', target_pose: [], yaw: 0.0}"
   ```
   - Updates the drone’s local origin reference. This command can only be executed when the drone is disarmed.
   - Use this to reset the coordinate system before starting a new mission or after relocation.


### other stuff
Information about the RTK system, such as survery time and minimum accuraccy.
- https://hamishwillee.github.io/px4_vuepress/en/gps_compass/rtk_gps.html#rtk-connection-process

Check for available devices on network
- nmap -sP 192.168.10.1/23

Issues with px4 agent:
-https://discuss.px4.io/t/issue-with-attitude-target-thrust-and-quaternions-are-conflicting/43515/10

sudo nano /boot/firmware/config.txt
In top insert after first paragraph:
dtoverlay=uart0-pi5

MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600


Lidar:
#install
sudo apt-get update
sudo apt-get upgrade -y
sudo apt install -y i2c-tools

Simulation:
src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake

Tools/simulation/gz/worlds/erc.sdf

https://github.com/IntelRealSense/realsense-ros
ros2 run realsense2_camera realsense2_camera_node

ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30 align_depth.enable:=true
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30



colcon build --cmake-args -DBUILD_PROBE_PERCEPTION=ON

docker build -t probe_perception:latest -f docker/dockerfile_perception .

%Vertiual enviroment
source ~/drone-software/venv/bin/activate



Matlab:
ros2genmsg('/home/daroe/drone-software/src', CreateShareableFile=true);