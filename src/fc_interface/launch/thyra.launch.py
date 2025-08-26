import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Define workspace directory (one level up from package)
    pkg_share = FindPackageShare('fc_interface')
    sensors_pkg_share = FindPackageShare('sensors')
    
    # Path to the simulation config file
    params_path = PathJoinSubstitution([pkg_share, 'config', 'thyra_params.yaml'])
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    position_source = LaunchConfiguration('position_source', default='px4')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'position_source',
            default_value='px4',
            description='Position source: px4 or mocap'
        ),
        
        # Start MicroXRCEAgent (output suppressed)
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyAMA0', '-b', '921600'],
            output='log',
        ),
        
        # Start Lidar node
        Node(
            package='sensors',
            executable='lidar',
            name='lidar_node',
            output='screen',
            remappings=[
                ('/fmu/in/distance_sensor', '/thyra/out/distance_sensor'),
            ],
        ),

        # Start Lidar node
        Node(
            package='sensors',
            executable='LED.py',
            name='led_node',
            output='screen',
        ),

        # Include thyra_cam launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([sensors_pkg_share, 'launch', 'thyra_cam.launch.py'])
            ),
        ),

        # Delay and launch FlightControllerInterface node
        TimerAction(
            period=15.0,  # Delay in seconds
            actions=[
                Node(
                    package='fc_interface',
                    executable='fci',
                    name='flight_controller_interface',
                    remappings=[
                        ('/fmu/out/vehicle_status', '/fmu/out/vehicle_status_v1'),
                        ('/fmu/in/vehicle_attitude_setpoint', '/fmu/in/vehicle_attitude_setpoint_v1'),
                    ],
                    output='screen',
                    parameters=[
                        params_path,
                        {'use_sim_time': use_sim_time},
                        {'position_source': position_source}
                    ]
                )
            ]
        ),
    ])