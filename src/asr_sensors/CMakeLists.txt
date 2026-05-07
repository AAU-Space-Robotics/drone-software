cmake_minimum_required(VERSION 3.5)

project(asr_sensors)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial_driver REQUIRED)
find_package(diagnostic_msgs REQUIRED)


# Include directories
include_directories(include)

# Create the C++ executable
add_executable(lidar src/lidar.cpp)

# Add dependencies for lidar executable
ament_target_dependencies(lidar
  rclcpp
  sensor_msgs
  px4_msgs
)

# Install C++ executables
install(TARGETS
  lidar
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  script/camera/image_republisher.py
  script/camera/image_republisher_pose.py
  script/camera/px4_to_cam.py
  script/camera/static_rgbd_test_node.py
  script/camera/probe_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  script/led/LED.py
  script/led/status_led.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  script/rtk_base/rtcm_reader.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install data files
install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Mark the package as complete
ament_package()