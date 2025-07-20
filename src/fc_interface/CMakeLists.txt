cmake_minimum_required(VERSION 3.5)

# Fetch Serial Library
include(FetchContent)
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
set(FETCHCONTENT_QUIET OFF)

FetchContent_Declare(serial
  GIT_REPOSITORY https://github.com/RoverRobotics-forks/serial-ros2.git
  GIT_TAG        master
  GIT_PROGRESS   TRUE
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(serial)

FetchContent_Declare(GeographicLib
  GIT_REPOSITORY https://github.com/geographiclib/geographiclib.git
  GIT_TAG        main
  GIT_PROGRESS   TRUE
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(GeographicLib)


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

# Link libraries for fci executable
target_link_libraries(fci serial GeographicLib)

# Add dependencies for fci executable
ament_target_dependencies(fci
  rclcpp
  rclcpp_action
  interfaces
  px4_msgs
)



# Ensure the `serial` library headers are available
target_include_directories(fci PRIVATE ${serial_SOURCE_DIR}/include ${GeographicLib_SOURCE_DIR}/include)


# Install C++ executables
install(TARGETS
  fci
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables (adjust as needed)
install(PROGRAMS  
  #scripts/example.py
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
