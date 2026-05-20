cmake_minimum_required(VERSION 3.5)
project(asr_logger)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(asr_comms REQUIRED)
find_package(std_msgs REQUIRED)

set(ULOG_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ulog_cpp)

add_executable(asr_logger
  src/logger_node.cpp
  ${ULOG_CPP_DIR}/messages.cpp
  ${ULOG_CPP_DIR}/writer.cpp
  ${ULOG_CPP_DIR}/simple_writer.cpp
)

ament_target_dependencies(asr_logger rclcpp asr_comms std_msgs)

target_include_directories(asr_logger PRIVATE
  include
  ${ULOG_CPP_DIR}
)

install(TARGETS asr_logger DESTINATION lib/${PROJECT_NAME})

ament_package()
