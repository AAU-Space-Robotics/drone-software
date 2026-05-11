import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction


def generate_launch_description():
    pkg_share = FindPackageShare('asr_autopilot')
    thyra_pkg_share = FindPackageShare('thyra')

    px4_dir = os.path.expanduser('~/PX4-Autopilot_thyra')
    
    # Path to the thyra simulation config file
    params_path = PathJoinSubstitution([thyra_pkg_share, 'config', 'uav', 'thyra_params_sim.yaml'])
    
    # Launch arguments
    use_sim_time    = LaunchConfiguration('use_sim_time',    default='true')
    position_source = LaunchConfiguration('position_source', default='px4')
    autopilot_delay = LaunchConfiguration('autopilot_delay', default='20.0')

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
        DeclareLaunchArgument(
            'autopilot_delay',
            default_value='20.0',
            description='Seconds to wait before launching the autopilot node',
        ),

        # ExecuteProcess(
        #     cmd=[
        #         'bash', '-c',
        #         f'''
        #         echo "Trying to cd into: {px4_dir}" && \
        #         cd {px4_dir} && \
        #         echo "Now in: $(pwd)" && \
        #         export GAZEBO_RESOURCE_PATH=~/PX4-Autopilot_thyra/Tools/simulation/gz/worlds:$GAZEBO_RESOURCE_PATH && \
        #         echo "Now running PX4 SITL with Gazebo X500 world" && \
        #         make px4_sitl gz_x500_erc > /dev/null 2>&1
        #         '''
        #     ],
        #     shell=True,
        #     output='screen',
        # ),
         ExecuteProcess(
            cmd=[
                'bash', '-c',
                f'''
                echo "Trying to cd into: {px4_dir}" && \
                cd {px4_dir} && \
                echo "Now in: $(pwd)" && \
                export GAZEBO_RESOURCE_PATH=~/PX4-Autopilot_thyra/Tools/simulation/gz/worlds:$GAZEBO_RESOURCE_PATH && \
                echo "Now running PX4 SITL with Gazebo X500 world" && \
                make px4_sitl gz_x500_lidar_down_windy > /dev/null 2>&1
                '''
            ],
            output='screen',
        ),
        
        # Start MicroXRCEAgent (output suppressed)
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='log',
        ),
        
        # MAVLink bridge — UAV side (UDP loopback to comms_gcs)
        Node(
            package='asr_comms',
            executable='comms_uav',
            name='comms_uav',
            namespace='asr/thyra',
            output='screen',
            # Uses 14561/14560 to avoid clashing with QGroundControl (14550/14551)
            parameters=[{
                'bind_port':   14561,
                'target_port': 14560,
            }],
        ),

        # Delay and launch FlightControllerInterface node
        TimerAction(
            period=autopilot_delay,
            actions=[
                Node(
                    package='asr_autopilot',
                    executable='asr_autopilot',
                    name='autopilot',
                    namespace='asr/thyra',
                    remappings=[
                        ('/asr/thyra/out/distance_sensor', '/fmu/out/distance_sensor'),
                    ],
                    parameters=[
                        params_path,
                        {'use_sim_time': use_sim_time},
                        {'position_source': position_source}
                    ],
                    output='screen'
                )
            ]
        ),
    ])
    
    
    
    
    
    
    