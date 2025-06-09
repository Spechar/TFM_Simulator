#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():

    run_slam = LaunchConfiguration('run_slam', default='true')

    return LaunchDescription([
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                  'map_server'
                ]
            }]
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': '/home/spech/tfm_ws/src/rcauto_sim_pkg/mapas/pau_track/mapa_pau_track.yaml',
                'use_sim_time': True,
                'topic_name': '/map'
            }]
        ),
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='localization_slam',
            output='screen',
            parameters=['/home/spech/tfm_ws/src/rcauto_sim_pkg/config/slam_localization.yaml'],
            condition=IfCondition(run_slam)
        )
    ])