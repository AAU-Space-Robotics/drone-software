#!/usr/bin/env python3
"""
Thyra Full System Launch
Launches all ASR components with Thyra-specific configuration.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    thyra_pkg = FindPackageShare('thyra')
    autopilot_pkg = FindPackageShare('asr_autopilot')
    
    # Thyra-specific configuration
    thyra_params = PathJoinSubstitution([thyra_pkg, 'config', 'thyra_params.yaml'])
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    position_source = LaunchConfiguration('position_source', default='px4')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'position_source',
            default_value='px4',
            description='Position source: px4 or mocap'
        ),
        
        # Include ASR autopilot launch (Thyra configuration)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([autopilot_pkg, 'launch', 'thyra.launch.py'])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'position_source': position_source,
            }.items()
        ),
        
        # TODO: Add perception node launch
        # TODO: Add mission-specific nodes
    ])
