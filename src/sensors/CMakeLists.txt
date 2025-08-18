cmake_minimum_required(VERSION 3.5)

project(sensors)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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
  script/image_republisher.py
  script/image_republisher_pose.py
  script/px4_to_cam.py
  script/static_rgbd_test_node.py
  script/probe_republisher.py
  script/LED.py
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