cmake_minimum_required(VERSION 3.5)
project(asr_comms)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(MAVLINK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mavlink)

set(COMMS_DEPS rclcpp interfaces px4_msgs std_msgs)

# --- comms_uav: runs on the drone ---
add_executable(comms_uav
  src/comms_uav.cpp
)
ament_target_dependencies(comms_uav ${COMMS_DEPS})
target_include_directories(comms_uav PRIVATE include)
target_include_directories(comms_uav SYSTEM PRIVATE ${MAVLINK_INCLUDE_DIR})

# --- comms_gcs: runs on the ground station ---
add_executable(comms_gcs
  src/comms_gcs.cpp
)
ament_target_dependencies(comms_gcs ${COMMS_DEPS})
target_include_directories(comms_gcs PRIVATE include)
target_include_directories(comms_gcs SYSTEM PRIVATE ${MAVLINK_INCLUDE_DIR})

install(TARGETS
  comms_uav
  comms_gcs
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
