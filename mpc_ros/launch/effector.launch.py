#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    effector_ns = LaunchConfiguration('effector_ns', 
        default='effector_frame')

    return LaunchDescription([
        DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='x position of effector frame'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='y position of effector frame'
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.0',
            description='z position of effector frame'
        ),
        DeclareLaunchArgument(
            'roll',
            default_value='0.0',
            description='roll of effector frame'
        ),
        DeclareLaunchArgument(
            'pitch',
            default_value='0.0',
            description='pitch of effector frame'
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='yaw of effector frame'
        ),
        DeclareLaunchArgument(
            'parent_frame',
            default_value='map',
            description='parent frame of effector frame'
        ),
        DeclareLaunchArgument(
            'child_frame',
            default_value='effector_frame',
            description='child frame of effector frame'
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='30.0',
            description='rate of effector frame'
        ),
        
        Node(
            package='umkc_mpc_ros',
            namespace=effector_ns,
            executable='effector_frame.py',
            parameters=[
                {'x': LaunchConfiguration('x')},
                {'y': LaunchConfiguration('y')},
                {'z': LaunchConfiguration('z')},
                {'roll': LaunchConfiguration('roll')},
                {'pitch': LaunchConfiguration('pitch')},
                {'yaw': LaunchConfiguration('yaw')},
                {'parent_frame': LaunchConfiguration('parent_frame')},
                {'child_frame': LaunchConfiguration('child_frame')},
                {'rate': LaunchConfiguration('rate')},
            ]
        )
    ])