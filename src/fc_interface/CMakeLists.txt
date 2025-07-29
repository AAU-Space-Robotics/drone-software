cmake_minimum_required(VERSION 3.5)

# ------------------------------------------------------------------------------
project(fc_interface)

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
find_package(rclcpp_action REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(interfaces REQUIRED)

# Include directories
include_directories(include)

# Create the C++ executable
add_executable(fci src/fci_main.cpp src/fci_controller.cpp src/fci_state_manager.cpp src/fci_transformations.cpp src/fci_path_planner.cpp)

# Add dependencies for fci executable
ament_target_dependencies(fci
  rclcpp
  rclcpp_action
  interfaces
  px4_msgs
)

# Install C++ executables
install(TARGETS
  fci
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables (adjust as needed)
install(PROGRAMS
  #src/dds_test.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Mark the package as complete
ament_package() 