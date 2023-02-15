#!/usr/bin/env python3
## example of launch file
# https://answers.ros.org/question/379101/spawning-multiple-robots-in-gazebo-and-assinging-namespaces/
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction

from launch_ros.actions import PushRosNamespace


from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PythonExpression


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

#https://automaticaddison.com/how-to-load-a-world-file-into-gazebo-ros-2/

def generate_launch_description():
    """launch"""
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world_file_name = 'empty_worlds/' + TURTLEBOT3_MODEL + '.model'
    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                         'worlds', world_file_name)
 
    world = '/home/justin/iq_sim/worlds/vtol.world'

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    print("path directory", get_package_share_directory('turtlebot3_description'))

    # urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    # urdf = os.path.join(
    #     get_package_share_directory('turtlebot3_description'),
    #     'urdf',
    #     urdf_file_name)
    # use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    
    #start gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        )
    
    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
                    )
                )
    


    #add actions
    ld = LaunchDescription()
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    return ld

