# AAU Space Robotics: ASR Drone Software

[![Build Jazzy 24.04](https://img.shields.io/github/actions/workflow/status/AAU-Space-Robotics/drone-software/build.yml?job=build-jazzy&label=Jazzy%2024.04)](https://github.com/AAU-Space-Robotics/drone-software/actions/workflows/build.yml)
[![Build Humble 22.04](https://img.shields.io/github/actions/workflow/status/AAU-Space-Robotics/drone-software/build.yml?job=build-humble&label=Humble%2022.04)](https://github.com/AAU-Space-Robotics/drone-software/actions/workflows/build.yml)
[![Version](https://img.shields.io/github/v/tag/AAU-Space-Robotics/drone-software?label=version)](https://github.com/AAU-Space-Robotics/drone-software/tags)

## Setup
To set up the drone software, refer to the [wiki](https://github.com/AAU-Space-Robotics/drone-software/wiki/Old-guide).

Two setup scripts are available in the `setup/` directory:
- `setup_workspace.sh` — for the development machine (includes PX4 SITL simulation)
- `setup_pi.sh` — for the onboard Raspberry Pi (includes RealSense camera, no simulation)

## How to Use the Drone Software

### 1. Ground Control Station
A ground control station must be running to communicate with either a real or simulated drone. Install and start [QGroundControl](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html) before proceeding.

### 2. Starting the Drone
From `~/drone-software`, source the workspace:
```bash
source install/setup.bash
```

Then launch the appropriate configuration:

**Physical drone:**
```bash
ros2 launch thyra thyra.launch.py
```

**Simulated drone:**
```bash
ros2 launch thyra thyra_sim.launch.py
```

### 3. Sending Commands
All commands are sent via the `/asr/thyra/in/drone_command` action. The drone must be **armed** before it can accept flight commands. After arming, send the next command within ~10 seconds or PX4 will automatically disarm the drone.

> Ensure the workspace is sourced with `source install/setup.bash` from `~/drone-software` before running any of the commands below.

---

- **Arm:**
```bash
  ros2 action send_goal /asr/thyra/in/drone_command interfaces/action/DroneCommand "{command_type: 'arm', target_pose: [], yaw: 0.0}"
```

- **Takeoff:**
```bash
  ros2 action send_goal /asr/thyra/in/drone_command interfaces/action/DroneCommand "{command_type: 'takeoff', target_pose: [-2], yaw: 0.0}"
```
  - `target_pose`: Single value representing altitude in meters. Negative = upward (e.g. `-2` = 2 m above origin). Required.
  - `yaw`: Desired orientation in radians.

- **Go To:**
```bash
  ros2 action send_goal /asr/thyra/in/drone_command interfaces/action/DroneCommand "{command_type: 'goto', target_pose: [1, 1, -2], yaw: 0.0}"
```
  - `target_pose`: `[x, y, z]` in meters. All three values required. Negative z = upward.
  - `yaw`: Desired orientation in radians (0 to 2π).

- **Velocity:**
```bash
  ros2 action send_goal /asr/thyra/in/drone_command interfaces/action/DroneCommand "{command_type: 'velocity', target_pose: [0, 0, -2], yaw: 0.0}"
```
  - `target_pose`: `[vx, vy, vz]` velocity in m/s. All three values required. Negative z = upward.
  - `yaw`: Desired orientation in radians (0 to 2π).

- **Spin:**
```bash
  ros2 action send_goal /asr/thyra/in/drone_command interfaces/action/DroneCommand "{command_type: 'spin', target_pose: [1.57, 2.0, 0], yaw: 0.0}"
```
  - `target_pose`: `[yaw, num_rotations, use_longest_path]`. All three values required.
    - `yaw`: Target orientation in radians (0 to 2π).
    - `num_rotations`: Number of full rotations before reaching target (0 = direct).
    - `use_longest_path`: `0` for shortest path, `1` for longest path.

- **Manual Mode:**
```bash
  ros2 action send_goal /asr/thyra/in/drone_command interfaces/action/DroneCommand "{command_type: 'manual', target_pose: [], yaw: 0.0}"
```
  - Switches to manual control. The flight controller will listen for input on `/asr/thyra/in/manual_input`, e.g. from a PS4 controller node in the `gcs` package.

- **Set New Origin:**
```bash
  ros2 action send_goal /asr/thyra/in/drone_command interfaces/action/DroneCommand "{command_type: 'set_origin', target_pose: [], yaw: 0.0}"
```
  - Resets the local coordinate origin. Can only be used while the drone is **disarmed**. Useful before starting a new mission or after relocation.



# Restart the drone:

ros2 action send_goal /system_control interfaces/action/SystemControl "{command_type: 'RESTART_NODES'}"

## Full system reboot
ros2 action send_goal /system_control interfaces/action/SystemControl "{command_type: 'REBOOT_PI'}"

# Bonus command:
ros2 run asr_comms comms_uav --ros-args -p serial_port:=auto