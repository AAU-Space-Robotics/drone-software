from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # MAVLink bridge — GCS side, UDP loopback to comms_uav
        # Uses 14560/14561 to avoid clashing with QGroundControl (14550)
        Node(
            package='asr_comms',
            executable='comms_gcs',
            name='comms_gcs',
            namespace='asr/gcs',
            output='screen',
            parameters=[{
                'bind_port':   14560,
                'target_port': 14561,
            }],
        ),

        # Ground control station GUI
        Node(
            package='asr_gcs',
            executable='GUI.py',
            name='gcs_gui',
            namespace='asr/gcs',
            output='screen',
            additional_env={
                'LIBGL_ALWAYS_SOFTWARE': '1',
                'WAYLAND_DISPLAY': '',   # force GLFW onto X11/GLX so PyOpenGL can detect the context
            },
        ),
    ])
