from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    with_camera  = LaunchConfiguration('with_camera')
    with_relay   = LaunchConfiguration('with_relay')
    params_path  = LaunchConfiguration('params_path')

    return LaunchDescription([
        DeclareLaunchArgument('with_camera', default_value='true',
                              description='Launch the Intel RealSense camera driver'),
        DeclareLaunchArgument('with_relay',  default_value='true',
                              description='Launch the camera relay (rate-limiter / republisher)'),
        DeclareLaunchArgument('params_path', default_value='',
                              description='Optional path to a ROS params YAML for the relay node'),

        ExecuteProcess(
            condition=IfCondition(with_camera),
            cmd=[
                'ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
                'depth_module.profile:=640x480x30',
                'rgb_camera.profile:=640x480x30',
            ],
            output='screen',
        ),

        # Relay without params — uses node defaults (5 Hz, JPEG quality 50).
        Node(
            condition=LaunchConfigurationEquals('params_path', ''),
            package='asr_drivers',
            executable='camera_relay.py',
            name='camera_relay',
            output='screen',
        ),

        # Relay with caller-supplied params (e.g. thyra_params.yaml).
        Node(
            condition=LaunchConfigurationNotEquals('params_path', ''),
            package='asr_drivers',
            executable='camera_relay.py',
            name='camera_relay',
            output='screen',
            parameters=[params_path],
        ),
    ])
