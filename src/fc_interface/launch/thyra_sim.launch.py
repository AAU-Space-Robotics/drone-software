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
    controller_params_path = PathJoinSubstitution([pkg_share, 'config', 'controller_gains_sim.yaml'])
    safety_params_path = PathJoinSubstitution([pkg_share, 'config', 'safety_params_sim.yaml'])
   
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    position_source = LaunchConfiguration('position_source', default='px4')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'position_source',
            default_value='px4',
            description='Position source: px4 or mocap'
        ),

        # Start PX4 SITL with Gazebo (output suppressed)
        ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'''
            echo "Trying to cd into: {px4_dir}" && \
            cd {px4_dir} && \
            echo "Now in: $(pwd)" && \
            make px4_sitl gz_x500 > /dev/null 2>&1
            '''
        ],
        shell=True,
        output='screen',
        ),
        
        # Start MicroXRCEAgent (output suppressed)
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
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
    
    
    
    
    
    
    