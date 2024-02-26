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
    front_directional_frame = 'front_directional_effector'
    base_link_frame = 'base_link'
    pkl_file_name = 'front_directional_effector.pkl'
    
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
                FindPackageShare('umkc_mpc_ros'),
                'launch',
                'effector.launch.py'
            ])
        ),
        launch_arguments=front_directional_effector.items()
    )

    #run effector_node
    front_directional_node = Node(
        package='umkc_mpc_ros',
        executable='effector_node.py',
        name='effector_node',
        output='screen',
        parameters=[
                {'world_frame': 'map' },
                {'effector_frame': front_directional_frame},
                {'rate': 30.0},
                {'pickle_file_name': pkl_file_name}
        ]
    )

    ####### LEFT WING ##################
    rotated_yaw = str(np.deg2rad(90.0))
    left_wing_effector_frame = 'left_wing_directional_effector'

    left_wing_directional_effector_params =  {
        'effector_ns': TextSubstitution(text='directional_effector'),
        'x': TextSubstitution(text='0.0'),
        'y': TextSubstitution(text='2.0'),
        'z': TextSubstitution(text='0.0'),
        'roll': TextSubstitution(text='0.0'),
        'pitch': TextSubstitution(text='0.0'),
        'yaw': TextSubstitution(text=rotated_yaw),
        'parent_frame': TextSubstitution(text='base_link'),
        'child_frame': TextSubstitution(text=left_wing_effector_frame),
        'rate': TextSubstitution(text='30.0')
    }

    left_wing_effector_frame_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('umkc_mpc_ros'),
                'launch',
                'effector.launch.py'
            ])
        ),
        launch_arguments=left_wing_directional_effector_params.items()
    )

    #run effector_node
    left_directional_node = Node(
        package='umkc_mpc_ros',
        executable='effector_node.py',
        name='effector_node',
        output='screen',
        parameters=[
                {'world_frame': 'map' },
                {'effector_frame': left_wing_effector_frame},
                {'rate': 30.0},
                {'pickle_file_name': 'left_wing_directional_effector.pkl'}
        ]
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
    right_wing_effector_frame = 'right_wing_directional_effector'
    right_wing_directional_effector_params =  {
        'effector_ns': TextSubstitution(text='directional_effector'),
        'x': TextSubstitution(text='0.0'),
        'y': TextSubstitution(text='-2.0'),
        'z': TextSubstitution(text='0.0'),
        'roll': TextSubstitution(text='0.0'),
        'pitch': TextSubstitution(text='0.0'),
        'yaw': TextSubstitution(text=right_rotated_yaw),
        'parent_frame': TextSubstitution(text='base_link'),
        'child_frame': TextSubstitution(text=right_wing_effector_frame),
        'rate': TextSubstitution(text='30.0')
    }

    right_wing_effector_frame_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('umkc_mpc_ros'),
                'launch',
                'effector.launch.py'
            ])
        ),
        launch_arguments=right_wing_directional_effector_params.items()
    )

    right_directional_node = Node(
        package='umkc_mpc_ros',
        executable='effector_node.py',
        name='effector_node',
        output='screen',
        parameters=[
                {'world_frame': 'map' },
                {'effector_frame': right_wing_effector_frame},
                {'rate': 30.0},
                {'pickle_file_name': 'right_wing_directional_effector.pkl'}
        ]
    )

    #launch stuf
    launch_description = LaunchDescription()
    launch_description.add_action(first_effector_launch)
    launch_description.add_action(front_directional_node)

    launch_description.add_action(left_wing_effector_frame_launch)
    launch_description.add_action(left_directional_node)    
    
    launch_description.add_action(right_wing_effector_frame_launch)
    launch_description.add_action(right_directional_node)
    
    return launch_description

