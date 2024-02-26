#!/usr/bin/env python3
import numpy as np
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node

"""
Uses the rviz_drone package to visualize all the trajectories of the drone
"""

def generate_launch_description() -> LaunchDescription:
    
    ### Avoid Trajectory Visualization #### 
    avoid_traj_config = {
        'scale_size': TextSubstitution(text='0.05'),
        'life_time': TextSubstitution(text='2.0'),
        'parent_frame': TextSubstitution(text='map'),
        'child_frame': TextSubstitution(text='avoid_traj_frame'),
        'rate': TextSubstitution(text='30.0'),
        'ns': TextSubstitution(text='avoid_traj'),
        'red_color': TextSubstitution(text='0.0'),
        'green_color': TextSubstitution(text='0.0'),
        'blue_color': TextSubstitution(text='1.0'),
        'topic_name': '/avoid_trajectory'
    }
    
    avoid_traj_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rviz_drone'),
                'launch',
                'traj.launch.py'
            ])
        ),
        launch_arguments=avoid_traj_config.items()
    )
    

    ### Waypoint Trajectory Visualization ####
    waypoint_traj_config = {
        'scale_size': TextSubstitution(text='0.05'),
        'life_time': TextSubstitution(text='2.0'),
        'parent_frame': TextSubstitution(text='map'),
        'child_frame': TextSubstitution(text='waypoint_traj_frame'),
        'rate': TextSubstitution(text='30.0'),
        'ns': TextSubstitution(text='waypoint_traj'),
        'red_color': TextSubstitution(text='0.0'),
        'green_color': TextSubstitution(text='1.0'),
        'blue_color': TextSubstitution(text='0.0'),
        'topic_name': '/waypoint_trajectory'
    }
    
    waypoint_traj_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rviz_drone'),
                'launch',
                'traj.launch.py'
            ])
        ),
        launch_arguments=waypoint_traj_config.items()
    )
    
    directional_traj_config = {
        'scale_size': TextSubstitution(text='0.05'),
        'life_time': TextSubstitution(text='2.0'),
        'parent_frame': TextSubstitution(text='map'),
        'child_frame': TextSubstitution(text='directional_traj_frame'),
        'rate': TextSubstitution(text='30.0'),
        'ns': TextSubstitution(text='directional_traj'),
        'red_color': TextSubstitution(text='1.0'),
        'green_color': TextSubstitution(text='0.0'),
        'blue_color': TextSubstitution(text='0.0'), 
        'topic_name': '/directional_trajectory'
    }
    
    directional_traj_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rviz_drone'),
                'launch',
                'traj.launch.py'
            ])
        ),
        launch_arguments=directional_traj_config.items()
    )
    
    fixed_wing_vis_node = Node(
        package='rviz_drone',
        executable='aircraft_frame.py',
        name='aircraft_frame',
        output='screen',
        parameters=[
            {'x': 0.0},
            {'y': 0.0},
            {'z': 0.0},
            {'roll': 0.0},
            {'pitch': 0.0},
            {'yaw': 0.0},
            {'parent_frame': 'map'},
            {'child_frame': 'fixed_wing_frame'},
            {'rate': 30.0}
        ]
        )
    
    #visualize obstacle node
    obs_viz_node = Node(
        package='rviz_drone',
        executable='obstacle_vis.py',
        name='obstacle_visualizer',
        output='screen',
        # parameters=[
        #     {'scale_size': 0.05},
        #     {'life_time': 2.0},
        #     {'parent_frame': 'map'},
        #     {'child_frame': 'obstacle_frame'},
        #     {'rate': 30.0},
        #     {'ns': 'obstacle'},
        #     {'topic_name': 'obstacle_marker'},
        #     {'red_color': 0.0},
        #     {'green_color': 0.0},
        #     {'blue_color': 0.0},
        #     {'alpha_color': 0.8}
        # ]
    )
    
    ld = LaunchDescription()
    ld.add_action(avoid_traj_launch)
    ld.add_action(waypoint_traj_launch)
    ld.add_action(directional_traj_launch)
    ld.add_action(fixed_wing_vis_node)
    ld.add_action(obs_viz_node)
    
    return ld

    
    
    