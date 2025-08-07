from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start RealSense camera node
        ExecuteProcess(
            cmd=['ros2', 'launch', 'realsense2_camera', 'rs_launch.py', 
                 'depth_module.profile:=640x480x30', 'rgb_camera.profile:=640x480x30'],
            output='screen',
        ),
        
        # Start image republisher node
        Node(
            package='sensors',
            executable='image_republisher.py',
            name='image_republisher',
            output='screen',
        ),
    ])