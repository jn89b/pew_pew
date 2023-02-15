#!/usr/bin/env python3

import rclpy 
import math as m
import casadi as ca
import numpy as np
import mavros 

from rclpy.node import Node
from mavros.base import SENSOR_QOS

# from geometry_msgs.msg import PoseStamped, \
#     TwistStamped, Pose, Twist, Point, \
#     Quaternion

from mpc_ros import quaternion_tools, MPC, Config
from mpc_ros.CasadiModels import FlatQuadModel

from pymavlink import mavutil
import pickle as pkl


import time

class FlatQuadMPC(MPC.MPC):
    
    def __init__(self, mpc_params:dict, quad_constraint_params:dict):
        super().__init__(mpc_params)
        self.quad_constraint_params = quad_constraint_params
        self.initDecisionVariables()
        self.defineBoundaryConstraints()
        self.addAdditionalConstraints()

    def warmUpSolution(self, start:list, goal:list) -> tuple:
        self.initDecisionVariables()
        self.defineBoundaryConstraints()
        self.addAdditionalConstraints()
        self.reinitStartGoal(start, goal)
        self.initSolver()
        self.computeCost()

        projected_controls,projected_states = self.solveMPCRealTimeStatic(start,goal)
        
        return projected_controls,projected_states


    def addAdditionalConstraints(self) -> None:
        self.lbx['U'][0,:] = self.quad_constraint_params['thrust_min']
        self.ubx['U'][0,:] = self.quad_constraint_params['thrust_max']

        self.lbx['U'][1,:] = self.quad_constraint_params['thrust_min']
        self.ubx['U'][1,:] = self.quad_constraint_params['thrust_max']

        self.lbx['U'][2,:] = self.quad_constraint_params['thrust_min']
        self.ubx['U'][2,:] = self.quad_constraint_params['thrust_max']

        self.lbx['U'][3,:] = self.quad_constraint_params['thrust_min']
        self.ubx['U'][3,:] = self.quad_constraint_params['thrust_max']

        self.lbx['X'][4,:] = self.quad_constraint_params['vx_min']
        self.ubx['X'][4,:] = self.quad_constraint_params['vx_max']

        self.lbx['X'][5,:] = self.quad_constraint_params['vy_min']
        self.ubx['X'][5,:] = self.quad_constraint_params['vy_max']

        self.lbx['X'][6,:] = self.quad_constraint_params['vz_min']
        self.ubx['X'][6,:] = self.quad_constraint_params['vz_max']

        self.lbx['X'][7,:] = self.quad_constraint_params['psi_dot_min']
        self.ubx['X'][7,:] = self.quad_constraint_params['psi_dot_max']


class QuadNode(Node):
    def __init__(self):
        super().__init__('quad_node')

        self.get_logger().info('QuadNode has been initialized')

        self.state_sub = self.create_subscription(mavros.local_position.Odometry,
                                                  'mavros/local_position/odom', self.position_callback, qos_profile=SENSOR_QOS)

        self.state_sub = self.create_subscription(mavros.local_position.PoseStamped,
                                                  'mavros/local_position/pose', self.pose_callback, qos_profile=SENSOR_QOS)

        self.state_info = [0, #x 
                           0, #y
                           0, #z
                           0, #psi
                           0, #vx
                           0, #vy
                           0, #vz
                           0] #psi_dot

    def position_callback(self,msg):
        """get state info between two positions"""
        self.state_info[0] = msg.pose.pose.position.x
        self.state_info[1] = msg.pose.pose.position.y
        self.state_info[2] = msg.pose.pose.position.z

        self.state_info[3] = quaternion_tools.quaternion_to_euler(msg.pose.pose.orientation)[2]

        self.state_info[4] = msg.twist.twist.linear.x
        self.state_info[5] = msg.twist.twist.linear.y
        self.state_info[6] = msg.twist.twist.linear.z

        self.state_info[7] = msg.twist.twist.angular.z
        
    def computeError(self, state:list, desired_state:list) -> np.ndarray:
        """compute the error between start and desired states"""
        error = np.zeros((8,1))
        error[0] = desired_state[0] - state[0]
        error[1] = desired_state[1] - state[1]
        error[2] = desired_state[2] - state[2]
        error[3] = desired_state[3] - state[3]
        error[4] = desired_state[4] - state[4]
        error[5] = desired_state[5] - state[5]
        error[6] = desired_state[6] - state[6]
        error[7] = desired_state[7] - state[7]

        return error

def initQuadMPC():
    #should allow user to map these parameters to a yaml file
    quad_constraint_params = {
        'vx_max': 15.0, #m/s
        'vx_min': -15.0, #m/s

        'vy_max': 15.0, #m/s
        'vy_min': -15.0, #m/s

        'vz_max': 5.0, #m/s
        'vz_min': -5.0, #m/s

        'psi_dot_max': np.rad2deg(5.0),#rad/s
        'psi_dot_min': np.rad2deg(-5.0),#rad/s

        'z_min': 5.0, #m
        'z_max': 20.0, #m

        'thrust_max': 10.0, #N
        'thrust_min': 0.0, #N
    }
    
    flat_quad_model = FlatQuadModel.FlatQuadcopterModel()
    flat_quad_model.set_state_space()

    mpc_quad_params = {
        'model': flat_quad_model,
        'N': 10,
        'dt_val': 0.1,
        'Q': np.diag([1, 1, 1, 1, 1, 1, 1, 1]),
        'R': np.diag([1, 1, 1, 1])    
    }
    quad_mpc = FlatQuadMPC(mpc_quad_params, quad_constraint_params)    

    return quad_mpc

def connectMAVUTIL(ip_string:str):
    master = mavutil.mavlink_connection(ip_string)
    master.wait_heartbeat()
    return master

def convertToNED(x:float, y:float, z:float) -> np.ndarray:
    """converts from ENU to NED"""
    ned = np.zeros((3,1))
    ned[0] = y
    ned[1] = x
    ned[2] = -z
    return ned

def mavSendVelocityCMD(master, vx:float, 
    vy:float, vz:float, yaw_rate:float):
    
    vel_ned = convertToNED(vx, vy, vz)

    master.mav.set_position_target_local_ned_send(
        0, #time_boot_ms
        1, #target_system
        1, #target_component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, #coordinate frame
        0b0000111111000111, #type_mask #enable vx, vy, vz, afx, afy, afz
        0, #x
        0, #y
        0, #z
        vel_ned[0], #vx
        vel_ned[1], #vy
        vel_ned[2], #vz
        0, #afx
        0, #afy
        0, #afz
        0, #yaw
        yaw_rate) #yaw_rate #r

def main(args=None):
    rclpy.init()

    ip_string = '127.0.0.1:14551'
    master = connectMAVUTIL(ip_string)
    quad_mpc = initQuadMPC()

    quad_node = QuadNode()    
    rclpy.spin_once(quad_node)

    #get the current time
    t0 = time.time()
    t_sim_limit = 50.0 #seconds    

    #init the desired state
    current_state = quad_node.state_info
    desired_state = [Config.GOAL_X, 
                        Config.GOAL_Y, 
                        current_state[2], 
                        current_state[3], 
                        current_state[4], 
                        current_state[5], 
                        current_state[6], 
                        current_state[7]]
    
    projected_controls, projected_states = quad_mpc.warmUpSolution(
        current_state, desired_state)

    while rclpy.ok() and time.time() - t0 < t_sim_limit:
        rclpy.spin_once(quad_node)

        if current_state == [0,0,0,0,0,0,0,0]:
            rclpy.spin_once(quad_node)
            continue
        
        #compute the error
        error = quad_mpc.computeError(current_state, desired_state)

        projected_controls, projected_states = quad_mpc.solveMPCRealTimeStatic(
            current_state,
            desired_state)            

        x_traj = projected_states[0,:]
        y_traj = projected_states[1,:]
        z_traj = projected_states[2,:]
        psi_traj = projected_states[3,:]

        vx_traj = projected_states[4,:]
        vy_traj = projected_states[5,:]
        vz_traj = projected_states[6,:]
        psi_dot_traj = projected_states[7,:]



        # #compute the control input
        # u = quad_mpc.computeControlInput(error)

        # #send the control input to the drone
        # master.mav.manual_control_send(master.target_system, 0, 0, 0, u[0], 0)

    # rclpy.spin(node)
    # rclpy.shutdown()

if __name__ == '__main__':
    main()