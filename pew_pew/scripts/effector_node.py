#!/usr/bin/env python3


# -*- coding: utf-8 -*-

"""

Effector Node, keep track of effector location

check if:
    within range of target
    within theta and psi of target
    if so then send command to effector

    keep track of damage done throughout history of flight

    subscribes to UAV position and orientation
    based on effector profile orientate wrt to UAV position and orientation

https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Adding-A-Frame-Py.html

Map frame of reference to aircraft position 
    if left wing then map frame of reference to left wing 
    if right wing then map frame of reference to right wing
    if nose then map frame of reference to nose 
    set offset locations based on this as well 

https://answers.ros.org/question/365863/ros2-launch-nodes-by-python-script/

https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Time-Travel-With-Tf2-Py.html


"""
import numpy as np 
import rclpy
import pickle as pkl
import pew_pew.rotation_utils as rotation_utils
from ros_mpc.aircraft_config import GOAL_STATE
# from pew_pew.rotation_utils import quaternion_tools
# from ros_mpc.ros_mpc.config import GO
from tf2_ros import TransformBroadcaster
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64

#this might need to be refactored
import mavros
from mavros_msgs.msg import State, AttitudeTarget, PositionTarget
from mavros.base import SENSOR_QOS
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

"""
Relative frame of effector 
"""

class Effector(Node):
    """"""
    def __init__(self, effector_config:dict, 
        node_name='effector_node', hz=30.0):
        
        super().__init__(node_name)

        self.effector_config = effector_config

        self.declare_parameter('world_frame', 'aircraft_frame')
        self.declare_parameter('effector_frame', 'effector_frame')
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('pickle_file_name', 'effector.pkl')

        self.parent_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('effector_frame').get_parameter_value().string_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value

        self.pkl_file_name = self.get_parameter('pickle_file_name').get_parameter_value().string_value

        #publish float32 message to effector topic
        self.effector_pub = self.create_publisher(Float64, '/damage_info', 10)

        if effector_config['effector_type'] == 'directional_3d':
            self.effector_type = 'triangle'
            self.effector_angle = effector_config['effector_angle'] #radians/2
            self.effector_range = effector_config['effector_range'] #meters
            self.effector_power = effector_config['effector_power'] #watts
            self.effector_profile = self.createPyramid(
                self.effector_angle, self.effector_range)    
        
        elif effector_config['effector_type'] == 'omnidirectional':
            self.effector_type = 'circle'
            self.effector_angle = 2*np.pi
            self.effector_range = effector_config['effector_range'] #meters
            self.effector_power = effector_config['effector_power'] #watts
            
        else:
            raise Exception("Effector type not recognized")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0/self.rate, self.refPointCallback)

        self.uav_pose_sub = self.create_subscription(
            mavros.local_position.PoseStamped, 
            '/mavros/local_position/pose', 
            self.uavPoseCallback, 
            qos_profile=SENSOR_QOS)

        # self.uav_pose_sub  # prevent unused variable warning
        
        self.uav_location = None
        self.location = None
        self.orientation = None
        self.rotated_offset_location = None
        self.effector_location = None

        #TODO: have this as a request to update thee change of the goal state 
        self.target_location = [GOAL_STATE[0], GOAL_STATE[1], GOAL_STATE[2]]
    
    def refPointCallback(self): 
        try: 
            t = self.tf_buffer.lookup_transform(
                self.parent_frame, 
                self.child_frame, 
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            self.ref_location = np.array([t.transform.translation.x,
                                    t.transform.translation.y,
                                    t.transform.translation.z])

            self.ref_orientation = np.array([t.transform.rotation.x,
                                    t.transform.rotation.y,
                                    t.transform.rotation.z,
                                    t.transform.rotation.w]) 
                                    
            #compute rotation matrix based on offset location
            self.setEffectorLocation(self.ref_location, self.ref_orientation)

            ref_roll,ref_pitch, ref_yaw = rotation_utils.euler_from_quaternion(
                self.ref_orientation[0], self.ref_orientation[1], 
                self.ref_orientation[2], self.ref_orientation[3])

            #check if target is within effector
            if self.isTargetWithinEffector(self.target_location, self.ref_location, ref_pitch, ref_yaw):
                
                target_distance = np.linalg.norm(self.ref_location - self.target_location)    
                power_density =  self.computePowerDensity(target_distance)

                #create message for Float64
                power_density_msg = Float64()
                power_density_msg.data = power_density
                self.effector_pub.publish(power_density_msg)

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            
            #log parent frame
            self.get_logger().info('parent frame: {}'.format(self.parent_frame))
            self.get_logger().info('child frame: {}'.format(self.child_frame))

        
        return 
            

    def createPyramid(self, angle, distance) -> np.ndarray:
        angle = angle/2
        p_x = distance * np.cos(angle)
        p_y = distance * np.sin(angle)
        p_1 = np.array([p_x, p_y, 0])

        p_y = -distance * np.sin(angle)
        p_2 = np.array([p_x, p_y, 0]) #the other point on the the triangle

        #return as an array of points
        return np.array([p_1, p_2])


    def uavPoseCallback(self, msg:PoseStamped):
        """
        Callback for UAV pose
        """
        #get the reference point
        self.uav_location = [msg.pose.position.x, 
                             msg.pose.position.y, 
                             msg.pose.position.z]


    def setEffectorLocation(self, ref_location:np.ndarray, ref_orientation:np.ndarray) -> None:
        """
        Get the effector location
        """
        ref_roll,ref_pitch, ref_yaw = quaternion_tools.euler_from_quaternion(
            ref_orientation[0], ref_orientation[1], ref_orientation[2], ref_orientation[3])

        effector_points = (quaternion_tools.rot3d(ref_roll,ref_pitch, ref_yaw) @ \
            np.transpose(self.effector_profile)).T + ref_location

        effector_location = np.vstack((ref_location, effector_points))
    
        row,num_points = effector_location.shape
        ref_location = ref_location.reshape(1,num_points)

        #combine effector location with ref location
        self.effector_location = np.vstack((effector_location, ref_location))

        print("effector_location: ", self.effector_location)

    def isTargetWithinEffector(self, target_location:np.ndarray, ref_location:np.ndarray, 
        ref_pitch:float, ref_yaw:float) -> bool:
        """
        Check if target is within effector
        """
        #check if target is within effector

        derror = np.linalg.norm(ref_location - target_location)

        if derror >= self.effector_range:
            return False

        #compute line of sight between target and effector
        dx = target_location[0] - ref_location[0]
        dy = target_location[1] - ref_location[1]
        dz = target_location[2] - ref_location[2]

        #compute the angle between the line of sight and the effector
        los_psi = np.arctan2(dy,dx)
        error_psi = los_psi - ref_yaw        
        los_theta = np.arctan2(dz,dx)
        error_theta = los_theta - ref_pitch

        #check los
        if np.abs(error_psi) >= self.effector_angle or np.abs(error_theta) >= self.effector_angle:
            return False
        
        return True
        
        
    def computePowerDensity(self, target_distance) -> float:
        """
        Compute the power density of the effector
        """
        return  float(self.effector_power / (4*np.pi*target_distance**2))

        
def main(args=None):

    rclpy.init(args=args)

    effector_config = {
        'effector_type': 'directional_3d',
        'effector_range': 20,
        'effector_angle': np.deg2rad(60),
        'effector_power': 100
    }

    #offset location based on uav frame in ENU convention


    effector = Effector(
        effector_config,'effector')
    
    #empty array to store uav location 
    uav_location = []
    effector_location = []
    uav_wing_location = []

    #set timer condition
    t0 = 0
    
    #current time of node
    t = effector.get_clock().now().nanoseconds * 1e-9
    
    t_final = 10 + t
    print("t0: ", t) 

    while rclpy.ok(): #and t < t_final:
        #get current time
        rclpy.spin_once(effector)

        t = effector.get_clock().now().nanoseconds * 1e-9
        
        if effector.effector_location is not None:
            uav_location.append((effector.uav_location))
            effector_location.append((effector.effector_location))

    #save uav location and effector location to pkl file
    info = {
        'uav_location': uav_location,
        'effector_location': effector_location,
        'uav_wing_location': uav_wing_location
    }

    with open(effector.pkl_file_name, 'wb') as f:
        pkl.dump(info, f)

    print("killing node saved to pkl file", effector.pkl_file_name)
    effector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()