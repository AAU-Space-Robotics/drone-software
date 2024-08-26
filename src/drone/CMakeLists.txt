	cmake_minimum_required(VERSION 3.5)
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
	find_package(serial REQUIRED)
	find_package(drone_interfaces REQUIRED)

	# Include Cpp "include" directory
	include_directories(include)

	# Create Cpp executables - Controller_node
	add_executable(Sensor src/Sensor.cpp) # change name of node
	ament_target_dependencies(Sensor rclcpp serial drone_interfaces)

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


