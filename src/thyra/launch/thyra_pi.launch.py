from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    thyra_pkg_share = FindPackageShare('thyra')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    position_source = LaunchConfiguration('position_source', default='px4')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'position_source',
            default_value='px4',
            description='Position source: px4 or mocap',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([thyra_pkg_share, 'launch', 'thyra.launch.py'])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'position_source': position_source,
            }.items(),
        ),
        Node(
            package='thyra',
            executable='system_manager.py',
            name='system_manager',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
    ])