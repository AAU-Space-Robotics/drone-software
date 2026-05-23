cmake_minimum_required(VERSION 3.5)
project(asr_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(asr_comms REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)

# --- probe_filter: C++ node ---
add_executable(probe_filter src/probe_filter.cpp)
ament_target_dependencies(probe_filter
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  asr_comms
  message_filters
)
target_include_directories(probe_filter SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS})

install(TARGETS probe_filter
  DESTINATION lib/${PROJECT_NAME}
)

# --- detect_probe: Python node ---
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/detect_probe.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()
