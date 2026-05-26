from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    with_camera     = LaunchConfiguration('with_camera')
    with_perception = LaunchConfiguration('with_perception')

    params_path = PathJoinSubstitution(
        [FindPackageShare('thyra'), 'config', 'uav', 'thyra_params.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument('with_camera',     default_value='true',
                              description='Launch the Intel RealSense camera and relay'),
        DeclareLaunchArgument('with_perception', default_value='true',
                              description='Launch the YOLO probe detector'),

        # PushRosNamespace only affects Node actions — the rs_launch.py ExecuteProcess
        # inside intel_cam.launch.py is unaffected, so the camera driver keeps its own
        # /camera namespace while camera_relay lands under asr/thyra.
        GroupAction(
            condition=IfCondition(with_camera),
            actions=[
                PushRosNamespace('asr/thyra'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [FindPackageShare('asr_drivers'), 'launch', 'intel_cam.launch.py']
                        )
                    ),
                    launch_arguments={
                        'params_path': params_path,
                    }.items(),
                ),
            ],
        ),

        Node(
            condition=IfCondition(with_perception),
            package='asr_perception',
            executable='detect_probe.py',
            name='probe_detector',
            namespace='asr/thyra',
            output='screen',
            parameters=[params_path],
        ),
    ])
