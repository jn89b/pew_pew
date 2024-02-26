#!/usr/bin/env python3
import numpy as np
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node


"""
Runs all the trajectories

Should map each of the trajectories to a different topic 
#TODO: Also should become a client where it queries the next target waypoint or location 

For now launch all the nodes and let them run
"""

def generate_launch_description() -> LaunchDescription:
    
    avoid_traj_node = Node(
        package='ros_mpc',
        executable='avoid_traj.py',
        name='avoid_traj',
        output='screen',
        #eventually we will have parameters to map this 
    )
    
    waypoint_traj_node = Node(
        package='ros_mpc',
        executable='waypoint_traj.py',
        name='waypoint_traj',
        output='screen'
    )
    
    directional_traj_node = Node(
        package='ros_mpc',
        executable='directional_traj.py',
        name='directional_traj',
        output='screen'
    )
    
    ld = LaunchDescription([
        avoid_traj_node,
        waypoint_traj_node,
        directional_traj_node
    ])
    
    return ld
    