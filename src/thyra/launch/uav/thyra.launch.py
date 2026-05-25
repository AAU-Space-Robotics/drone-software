from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    thyra_pkg_share = FindPackageShare('thyra')

    params_path = PathJoinSubstitution([thyra_pkg_share, 'config', 'uav', 'thyra_params.yaml'])
    comms_path  = PathJoinSubstitution([thyra_pkg_share, 'config', 'comms', 'thyra_comms.yaml'])

    hardware         = LaunchConfiguration('hardware')
    use_sim_time     = LaunchConfiguration('use_sim_time')
    position_source  = LaunchConfiguration('position_source')
    with_comms       = LaunchConfiguration('with_comms')
    with_camera      = LaunchConfiguration('with_camera')
    with_perception  = LaunchConfiguration('with_perception')
    autopilot_delay  = LaunchConfiguration('autopilot_delay')
    use_led          = LaunchConfiguration('use_led')

    # Jetson uses USB-to-TTL adapter (udev symlink), Pi uses native UART
    serial_device = PythonExpression([
        '"/dev/px4" if "', hardware, '" == "jetson" else "/dev/ttyAMA0"'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware',
            default_value='jetson',
            description='Hardware target: jetson or pi',
        ),
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
        DeclareLaunchArgument(
            'with_comms',
            default_value='true',
            description='Launch comms_uav SiK radio bridge node',
        ),
        DeclareLaunchArgument(
            'with_camera',
            default_value='true',
            description='Launch the Intel RealSense camera and relay',
        ),
        DeclareLaunchArgument(
            'with_perception',
            default_value='true',
            description='Launch the YOLO probe detector',
        ),
        DeclareLaunchArgument(
            'autopilot_delay',
            default_value='15.0',
            description='Seconds to wait before launching the autopilot node',
        ),
        DeclareLaunchArgument(
            'use_led',
            default_value='false',
            description='Use LED node',
        ),
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial', '--dev', serial_device, '-b', '921600'],
            output='log',
        ),

        Node(
            condition=IfCondition(use_led),
            package='asr_drivers',
            executable='LED.py',
            name='led',
            namespace='asr/thyra',
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([thyra_pkg_share, 'launch', 'uav', 'thyra_perception.launch.py'])
            ),
            launch_arguments={
                'with_camera':     with_camera,
                'with_perception': with_perception,
            }.items(),
        ),

        Node(
            condition=IfCondition(with_comms),
            package='asr_comms',
            executable='comms_uav',
            name='comms_uav',
            namespace='asr/thyra',
            output='screen',
            parameters=[comms_path],
        ),

        Node(
            package='asr_logger',
            executable='asr_logger',
            name='asr_logger',
            namespace='asr/thyra',
            output='screen',
            parameters=[params_path],
        ),

        TimerAction(
            period=autopilot_delay,
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
