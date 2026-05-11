from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    thyra_pkg_share = FindPackageShare('thyra')
    comms_path = PathJoinSubstitution([thyra_pkg_share, 'config', 'comms', 'gcs_comms.yaml'])

    return LaunchDescription([
        # MAVLink bridge — GCS side
        Node(
            package='asr_comms',
            executable='comms_gcs',
            name='comms_gcs',
            namespace='asr/gcs',
            output='screen',
            parameters=[comms_path],
        ),

        # RTK base station — streams RTCM corrections to /rtcm
        Node(
            package='asr_drivers',
            executable='rtcm_reader.py',
            name='rtcm_reader',
            namespace='asr/gcs',
            output='screen',
            parameters=[comms_path],
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
