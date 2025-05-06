import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Define workspace directory (one level up from package)
    workspace_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    
    # directory which workspace is located in
    general_dir = os.path.abspath(os.path.join(workspace_dir, '..', '..', '..'))
    px4_dir = os.path.join(general_dir, 'PX4-Autopilot')
    
    print(f"px4_dir: {px4_dir}")

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

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
        # Start PX4 SITL with Gazebo
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                f'cd {px4_dir} && echo "Now in: $(pwd)" && make px4_sitl gz_x500'
            ],
            output='screen',
        ),

        #Start MicroXRCEAgent (output suppressed)
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='log',
        ),
      
        # Launch FlightControllerInterface node
        Node(
            package='fc_interface',
            executable='fci',
            name='flight_controller_interface',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])