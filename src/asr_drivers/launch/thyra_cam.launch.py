from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_path = PathJoinSubstitution(
        [FindPackageShare('thyra'), 'config', 'uav', 'thyra_params.yaml']
    )

    return LaunchDescription([
        # RealSense camera at full rate — relay node controls the output rate
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
                'depth_module.profile:=640x480x30',
                'rgb_camera.profile:=640x480x30',
            ],
            output='screen',
        ),

        # Camera relay: compresses + rate-limits frames for perception and GCS streaming.
        # fps and jpeg_quality are set in thyra_params.yaml under camera.stream.
        Node(
            package='asr_drivers',
            executable='camera_relay.py',
            name='camera_relay',
            output='screen',
            parameters=[params_path],
        ),
    ])
