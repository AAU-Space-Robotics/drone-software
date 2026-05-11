import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    mission_pkg_share = FindPackageShare('thyra')
    params_path = PathJoinSubstitution([mission_pkg_share, 'config', 'uav', 'thyra_params.yaml'])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    position_source = LaunchConfiguration('position_source', default='px4')

    return LaunchDescription([

        # optional: allow overriding from command line
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'position_source',
            default_value='px4',
            description='Position source selection'
        ),

        Node(
            package='thyra',
            executable='thyra_mission_executor',
            name='thyra_mission_executor',
            output='screen',
            parameters=[
                params_path,
                {
                    'use_sim_time': use_sim_time,
                    'position_source': position_source,
                }
            ],
        ),
    ])
