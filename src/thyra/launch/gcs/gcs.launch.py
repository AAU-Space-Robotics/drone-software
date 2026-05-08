from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='auto')
    baud_rate   = LaunchConfiguration('baud_rate',   default='57600')
    rtk_port    = LaunchConfiguration('rtk_port',    default='/dev/ttyACM0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='auto',
            description='Serial device for SiK radio (auto to detect /dev/ttyUSB* or /dev/ttyACM*)',
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='57600',
            description='Baud rate for the SiK radio',
        ),
        DeclareLaunchArgument(
            'rtk_port',
            default_value='/dev/ttyACM0',
            description='Serial device for the u-blox RTK base station',
        ),

        # MAVLink bridge — GCS side
        Node(
            package='asr_comms',
            executable='comms_gcs',
            name='comms_gcs',
            namespace='asr/gcs',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'baud_rate':   baud_rate,
            }],
        ),

        # RTK base station — streams RTCM corrections to /rtcm
        Node(
            package='asr_drivers',
            executable='rtcm_reader.py',
            name='rtcm_reader',
            namespace='asr/gcs',
            output='screen',
            parameters=[{
                'port': rtk_port,
            }],
        ),

        # Ground control station GUI
        Node(
            package='asr_gcs',
            executable='GUI.py',
            name='gcs_gui',
            namespace='asr/gcs',
            output='screen',
        ),
    ])
