#!/usr/bin/env python3
import numpy as np
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    #### Directional #######
    front_directional_frame = 'front_directional_effector_scaled'
    base_link_frame = 'fixed_wing_frame'
    pkl_file_name = 'front_directional_effector.pkl'
    effector_file_name = 'effector_scaled.launch.py'
    
    front_directional_effector =  {
        'effector_ns': TextSubstitution(text='directional_effector'),
        'x': TextSubstitution(text='1.5'),
        'y': TextSubstitution(text='0.0'),
        'z': TextSubstitution(text='0.0'),
        'roll': TextSubstitution(text='0.0'),
        'pitch': TextSubstitution(text='0.0'),
        'yaw': TextSubstitution(text='0.0'),
        'parent_frame': TextSubstitution(text=base_link_frame),
        'child_frame': TextSubstitution(text=front_directional_frame),
        'rate': TextSubstitution(text='30.0')
    }

    first_effector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('pew_pew'),
                'launch',
                effector_file_name
            ])
        ),
        launch_arguments=front_directional_effector.items()
    )

    ####### LEFT WING ##################
    rotated_yaw = str(np.deg2rad(90.0))
    left_wing_effector_frame = 'left_wing_directional_effector_scaled'

    left_wing_directional_effector_params =  {
        'effector_ns': TextSubstitution(text='directional_effector'),
        'x': TextSubstitution(text='0.0'),
        'y': TextSubstitution(text='2.0'),
        'z': TextSubstitution(text='0.0'),
        'roll': TextSubstitution(text='0.0'),
        'pitch': TextSubstitution(text='0.0'),
        'yaw': TextSubstitution(text=rotated_yaw),
        'parent_frame': TextSubstitution(text=base_link_frame),
        'child_frame': TextSubstitution(text=left_wing_effector_frame),
        'rate': TextSubstitution(text='30.0')
    }

    left_wing_effector_frame_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('pew_pew'),
                'launch',
                'effector_scaled.launch.py'
                # effector_file_name
                # 'effector.launch.py'
            ])
        ),
        launch_arguments=left_wing_directional_effector_params.items()
    )

    # mavros_node = Node(
    #     package='mavros', 
    #     executable='mavros_node',
    #     name='mavros_node'
    #     parameters=[
    #         {'fcu_url': 'udp://127.0.0.1:14570@14557'}]
    # )

    ########### RIGHT WING ################## 
    right_rotated_yaw = str(np.deg2rad(-90.0))
    right_wing_effector_frame = 'right_wing_directional_effector_scaled'
    right_wing_directional_effector_params =  {
        'effector_ns': TextSubstitution(text='directional_effector'),
        'x': TextSubstitution(text='0.0'),
        'y': TextSubstitution(text='-2.0'),
        'z': TextSubstitution(text='0.0'),
        'roll': TextSubstitution(text='0.0'),
        'pitch': TextSubstitution(text='0.0'),
        'yaw': TextSubstitution(text=right_rotated_yaw),
        'parent_frame': TextSubstitution(text=base_link_frame),
        'child_frame': TextSubstitution(text=right_wing_effector_frame),
        'rate': TextSubstitution(text='30.0')
    }

    right_wing_effector_frame_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('pew_pew'),
                'launch',
                'effector_scaled.launch.py'
            ])
        ),
        launch_arguments=right_wing_directional_effector_params.items()
    )

    #launch stuf
    launch_description = LaunchDescription()
    launch_description.add_action(first_effector_launch)
    launch_description.add_action(left_wing_effector_frame_launch)
    launch_description.add_action(right_wing_effector_frame_launch)
    
    return launch_description

