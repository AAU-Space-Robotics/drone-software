cmake_minimum_required(VERSION 3.5)
project(drone_interfaces)

	# Default to C++14
	if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
	endif()

	if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
	endif()


	
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
if(NOT builtin_interfaces_FOUND)
message(FATAL_ERROR "Could not find builtin_interfaces package")
endif()

# Generate message files
rosidl_generate_interfaces(${PROJECT_NAME}
"msg/DroneCommand.msg"
"msg/DroneControlData.msg"
"msg/DroneStatus.msg"
"msg/DroneVideo.msg"
"msg/ViconData.msg"
"msg/DroneIMU.msg"
"srv/Clock.srv"
"msg/SensorData.msg"
DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs)




# find dependencies
find_package(ament_cmake REQUIRED)


ament_package()
