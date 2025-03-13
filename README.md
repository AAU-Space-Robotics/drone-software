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
