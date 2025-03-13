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
1. Ensure `serial0` is free; disable conflicting services (e.g., console). Edit `/boot/firmware/cmdline.txt`:  
   ```
   sudo nano /boot/firmware/cmdline.txt
   ```
   Remove any `serial0` references. Save and exit.

2. Edit `/boot/firmware/config.txt` and add:  
   ```
   enable_uart=1
   ```

3. Grant serial port access (Ubuntu restricts it by default):  
   Add your user to the `dialout` group:  
   ```
   sudo usermod -a -G dialout $(whoami)
   ```

4. Reboot the Raspberry Pi:  
   ```
   sudo reboot
   ```

5. **Test the Serial Port:**  
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
Here's an improved version of the additional steps. I've made them concise, consistent with the previous format, and clearer while addressing technical details:

6. **Install ROS 2 Humble:**  
   Follow the [official ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). Select the **Base** image (not Desktop, as visualization isn’t needed and Desktop consumes more resources). Include the developer tools during installation.

7. **Set Up Micro XRCE-DDS Agent:**  
   Follow the "Setup Micro XRCE-DDS Agent & Client" section in the [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide.html). Perform **only the Agent setup**—skip the Client step.

8. **Clone the Repository:**  
   Clone this repository to obtain the code for controlling the drone:  
   ```
   git clone git@github.com:AAU-Space-Robotics/drone-software.git
   ```
   `cd` into the workspace, and use `colcon build` to build the repository.
   
   
### PX4 Configuration (Cube Orange)
To configure the flight controller (Cube Orange) with PX4, follow these steps:

1. **Download QGroundControl:**  
   Install QGroundControl from the [official guide](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html).

2. **Connect and Check Firmware:**  
   Connect the drone to your computer via micro-USB. In QGroundControl, go to **Vehicle Setup > Firmware**. Verify the firmware is PX4 v1.15.2 (update if needed).

3. **Configure Parameters:**  
   In **Vehicle Setup > Parameters**, set the following:  
   - **Telemetry on TELEM2:**  
     ```
     SER_TEL2_BAUD = 921600
     ```
   - **UXRCE-DDS on TELEM2:**  
     ```
     UXRCE_DDS_CFG = TELEM2
     ```

4. **Save and Reboot:**  
   Save the changes and restart the flight controller.

# 2. Use of Packages

Build the package in the root of the workspace (e.g., `~/drone-software`) to be able to use it. Then source it with:

```bash
source install/setup.bash
```

## fc_interface Package

Currently, `fc_interface` is the only package containing nodes that must be run directly on the onboard computer. This package includes a set of custom software stacks that enable drone control using the methods described below.

### 1. Establish Connection with the Flight Controller

For the `fc_interface` package to function, it requires the ROS 2 communication channel established by the `MicroXRCEAgent`. If the setup steps in Section 1 have not been performed, return and complete them. If setup is complete, use the following command to open the node, which publishes and subscribes to a specific set of ROS 2 topics:

```bash
MicroXRCEAgent serial --dev /dev/serial0 -b 921600
```

#### 1.1 (Optional) Establish Connection with a Simulation

To support drone stack development, a Gazebo simulation is available from the PX4 community. Read more here: [link placeholder]. If the simulation is built correctly, navigate to its root directory (likely `PX4-Autopilot`) and run:

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
ros2 action send_goal /fmu/in/drone_command interfaces/action/DroneCommand "{command_type: 'arm', target_pose: [], yaw: 0.0}"
```

Run this command from the workspace root (e.g., `~/drone-software`). Ensure the package is sourced with `source install/setup.bash` beforehand, or the ROS 2 network will not recognize the package. Once armed, send a new command within approximately 10 seconds, or the drone will disarm due to a PX4 safety feature (not implemented in this package). The following commands are currently supported:

- **Takeoff:**
  ```bash
  ros2 action send_goal /fmu/in/drone_command interfaces/action/DroneCommand "{command_type: 'takeoff', target_pose: [-10], yaw: 0.0}"
  ```
  - Requires one argument: the z-coordinate (altitude) to reach (e.g., `-10` for 10 meters upward). Negative values indicate upward movement.

- **Go To:**
  ```bash
  ros2 action send_goal /fmu/in/drone_command interfaces/action/DroneCommand "{command_type: 'goto', target_pose: [5,5,-10], yaw: 0.0}"
  ```
  - Requires an `[x, y, z]` coordinate. The command will be rejected without all three values. Altitude is specified with negative values for higher altitudes.

- **Manual Mode:**
  ```bash
  ros2 action send_goal /fmu/in/drone_command interfaces/action/DroneCommand "{command_type: 'manual', target_pose: [], yaw: 0.0}"
  ```
  - Switches the controller to listen for manual control inputs on the `drone/in/manual_input` topic. A node in the `gcs` package can publish control values from a PS4 controller.


### other stuff
Information about the RTK system, such as survery time and minimum accuraccy.
- https://hamishwillee.github.io/px4_vuepress/en/gps_compass/rtk_gps.html#rtk-connection-process

Check for available devices on network
- nmap -sP 192.168.0.1/23

Issues with px4 agent:
-https://discuss.px4.io/t/issue-with-attitude-target-thrust-and-quaternions-are-conflicting/43515/10




