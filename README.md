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
1. Ensure `serial0` is available; disable conflicting services (e.g., console).
2. Edit `/boot/config.txt` to add:
   ```
   enable_uart=1
   ```
3. Reboot the Raspberry Pi.

### PX4 Configuration (Cube Orange)
1. Set telemetry on TELEM2:
   ```
   SER_TEL2_BAUD = 921600
   ```
2. Configure UXRCE-DDS on TELEM2:
   ```
   UXRCE_DDS_CFG = TELEM2
   ```
3. Save and restart PX4.
