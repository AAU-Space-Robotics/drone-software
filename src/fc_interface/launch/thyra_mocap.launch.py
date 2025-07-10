import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction


def generate_launch_description():
    # Define workspace directory (one level up from package)
    workspace_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    pkg_share = FindPackageShare('fc_interface')

    
    # directory which workspace is located in
    general_dir = os.path.abspath(os.path.join(workspace_dir, '..', '..', '..'))
    px4_dir = os.path.join(general_dir, 'PX4-Autopilot')
    
    # Path to the simulation config file
    controller_params_path = PathJoinSubstitution([pkg_share, 'config', 'controller_params_sim.yaml'])
    safety_params_path = PathJoinSubstitution([pkg_share, 'config', 'safety_params_sim.yaml'])
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    position_source = LaunchConfiguration('position_source', default='mocap')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'position_source',
            default_value='mocap',
            description='Position source: px4 or mocap'
        ),
        
        # Start MicroXRCEAgent (output suppressed)
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyAMA0', '-b', '921600'],
            output='log',
        ),

        # Delay and launch FlightControllerInterface node
        TimerAction(
            period=20.0,  # Delay in seconds
            actions=[
                Node(
                    package='fc_interface',
                    executable='fci',
                    name='flight_controller_interface',
                    output='screen',
                    parameters=[
                        controller_params_path,
                        safety_params_path,
                        {'use_sim_time': use_sim_time},
                        {'position_source': position_source}
                    ]
                )
            ]
        ),
    ])