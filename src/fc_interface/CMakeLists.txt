cmake_minimum_required(VERSION 3.5)

# --- Fetch Serial -------------------------------------------------
include(FetchContent)
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
set(FETCHCONTENT_QUIET OFF)

# Fetch serial-ros2
FetchContent_Declare(serial
  GIT_REPOSITORY https://github.com/RoverRobotics-forks/serial-ros2.git
  GIT_TAG        master
  GIT_PROGRESS   TRUE
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(serial)

# ------------------------------------------------------------------------------
project(fc_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(interfaces REQUIRED)
find_package(px4_msgs REQUIRED)

# Include directories
include_directories(include)

# Create Cpp executable - Controller_node
add_executable(FCI src/FCI.cpp)  # Change name of the node as needed

# Link libraries for FCI executable
target_link_libraries(FCI
  serial  # Link the serial library
)

# Add dependencies for FCI executable
ament_target_dependencies(FCI
  rclcpp
  interfaces
  px4_msgs
)

# Install Cpp executables
install(TARGETS
  FCI
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS  
  DESTINATION lib/${PROJECT_NAME}  # Change name of the node in Python
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
