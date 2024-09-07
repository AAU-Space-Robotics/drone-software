	cmake_minimum_required(VERSION 3.5)
# --- Fetch Serial --------------------------------------------------------------
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
# ------------------------------------------------------------------------------
	project(drone)

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
	# find_package(serial REQUIRED)
	find_package(drone_interfaces REQUIRED)

	# Include Cpp "include" directory
	include_directories(include)

	# Create Cpp executables - Controller_node
	add_executable(GPS src/GPS.cpp) # change name of node
	target_include_directories(serial PUBLIC
	  "$<BUILD_INTERFACE:${serial_SOURCE_DIR}/include>"
	  "$<INSTALL_INTERFACE:include/serial>")
	target_link_libraries(GPS serial)
	ament_target_dependencies(GPS rclcpp  drone_interfaces )

	# Install Cpp executables
	install(TARGETS
	Sensor
	DESTINATION lib/${PROJECT_NAME}
	)

	# Install Python modules
	ament_python_install_package(${PROJECT_NAME})

	# Install Python executables
	install(PROGRAMS
	scripts/fc_cmd.py
	scripts/camera.py
	scripts/clock.py
	scripts/save_image.py
	scripts/manual_cmd.py
	scripts/GCS.py
	scripts/viconData.py
	scripts/GPS.py
	scripts/emulateViconData.py
	scripts/Parse.py
	scripts/imu.py
	DESTINATION lib/${PROJECT_NAME} # change name of node in python
	)

	# Install launch files.
	install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/
	)

	ament_package()


