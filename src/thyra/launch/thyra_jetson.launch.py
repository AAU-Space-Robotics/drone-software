from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    thyra_pkg_share = FindPackageShare('thyra')
    sensors_pkg_share = FindPackageShare('asr_sensors')

    params_path = PathJoinSubstitution([thyra_pkg_share, 'config', 'thyra_params.yaml'])
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
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyTHS1', '-b', '921600'],
            output='log',
        ),
        Node(
            package='asr_sensors',
            executable='LED.py',
            name='led',
            namespace='asr/thyra',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([sensors_pkg_share, 'launch', 'thyra_cam.launch.py'])
            ),
        ),
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='asr_autopilot',
                    executable='asr_autopilot',
                    name='autopilot',
                    namespace='asr/thyra',
                    output='screen',
                    parameters=[
                        params_path,
                        {'use_sim_time': use_sim_time},
                        {'position_source': position_source},
                    ],
                ),
            ],
        ),
    ])
