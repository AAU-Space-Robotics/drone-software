cmake_minimum_required(VERSION 3.5)
project(asr_comms)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate all custom interfaces (formerly the interfaces package)
rosidl_generate_interfaces(${PROJECT_NAME}
  "ros/msg/ManualControlInput.msg"
  "ros/msg/DroneCommand.msg"
  "ros/msg/MotionCapturePose.msg"
  "ros/msg/DroneScope.msg"
  "ros/msg/GcsHeartbeat.msg"
  "ros/msg/ProbeLocations.msg"
  "ros/msg/ProbeSegmentation.msg"
  "ros/msg/ProbeGlobalLocations.msg"
  "ros/msg/AttitudeSetpointRPY.msg"
  "ros/msg/ServoCommand.msg"
  "ros/msg/command/UAVCommand.msg"
  "ros/msg/command/CommandAck.msg"
  "ros/msg/telemetry/TelemetryPosition.msg"
  "ros/msg/telemetry/TelemetryAttitude.msg"
  "ros/msg/telemetry/TelemetryBattery.msg"
  "ros/msg/telemetry/TelemetryGPS.msg"
  "ros/msg/telemetry/TelemetryStatus.msg"
  "ros/action/DroneCommand.action"
  "ros/action/MissionComand.action"
  "ros/action/SystemControl.action"
  "ros/srv/Clock.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs action_msgs
)

set(MAVLINK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mavlink)

set(COMMS_DEPS rclcpp px4_msgs std_msgs)

# --- comms_uav: runs on the drone ---
add_executable(comms_uav
  src/comms_uav.cpp
)
ament_target_dependencies(comms_uav ${COMMS_DEPS} rclcpp_action)
target_include_directories(comms_uav PRIVATE include)
target_include_directories(comms_uav SYSTEM PRIVATE ${MAVLINK_INCLUDE_DIR})

# --- comms_gcs: runs on the ground station ---
add_executable(comms_gcs
  src/comms_gcs.cpp
)
ament_target_dependencies(comms_gcs ${COMMS_DEPS})
target_include_directories(comms_gcs PRIVATE include)
target_include_directories(comms_gcs SYSTEM PRIVATE ${MAVLINK_INCLUDE_DIR})

# Link both executables against this package's generated typesupport.
# Must use rosidl_get_typesupport_target when generator and consumer are the same package.
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(comms_uav "${cpp_typesupport_target}")
target_link_libraries(comms_gcs "${cpp_typesupport_target}")

install(TARGETS
  comms_uav
  comms_gcs
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
