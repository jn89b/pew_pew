#!/usr/bin/env python3

import rclpy 
import math as m
import casadi as ca
import numpy as np
import mavros 

from rclpy.node import Node
from mavros.base import SENSOR_QOS


from mpc_ros import quaternion_tools, MPC, Config
from mpc_ros.CasadiModels import FlatQuadModel, SimpleQuadModel

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
        self.reinitStartGoal(start, goal)
        self.computeCost()
        self.initSolver()
        self.defineBoundaryConstraints()
        self.addAdditionalConstraints()

        projected_controls,projected_states = self.solveMPCRealTimeStatic(start,goal)
        
        return projected_controls,projected_states

    def addAdditionalConstraints(self) -> None:
        #add additional constraints here
        self.lbx['U'][0,:] = self.quad_constraint_params['vx_min']
        self.ubx['U'][0,:] = self.quad_constraint_params['vx_max']

        self.lbx['U'][1,:] = self.quad_constraint_params['vy_min']
        self.ubx['U'][1,:] = self.quad_constraint_params['vy_max']

        self.lbx['U'][2,:] = self.quad_constraint_params['vz_min']
        self.ubx['U'][2,:] = self.quad_constraint_params['vz_max']

        self.lbx['U'][3,:] = self.quad_constraint_params['psi_dot_min']
        self.ubx['U'][3,:] = self.quad_constraint_params['psi_dot_max']

                                 
class QuadNode(Node):
    def __init__(self):
        super().__init__('quad_node')

        self.get_logger().info('QuadNode has been initialized')

        self.state_sub = self.create_subscription(mavros.local_position.Odometry,
                                                  'mavros/local_position/odom', 
                                                  self.positionCallback, 
                                                  qos_profile=SENSOR_QOS)

        # self.state_sub = self.create_subscription(mavros.local_position.PoseStamped,
        #                                           'mavros/local_position/pose', self.poseCallback, qos_profile=SENSOR_QOS)
        self.state_info = [0, #x 
                           0, #y
                           0, #z
                           0] #psi

        self.control_info = [0, #vx
                             0, #vy
                             0, #vz
                             0] #psi_dot

    def positionCallback(self,msg):
        """get state info between two positions"""
        self.state_info[0] = msg.pose.pose.position.x
        self.state_info[1] = msg.pose.pose.position.y
        self.state_info[2] = msg.pose.pose.position.z

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        roll, pitch, yaw = quaternion_tools.euler_from_quaternion(qx, qy, qz, qw)
        self.state_info[3] = yaw

        self.control_info[0] = msg.twist.twist.linear.x
        self.control_info[1] = msg.twist.twist.linear.y
        self.control_info[2] = msg.twist.twist.linear.z
        self.control_info[3] = msg.twist.twist.angular.z
        
    def computeError(self, state:list, desired_state:list) -> np.ndarray:
        """compute the error between start and desired states"""
        error = np.zeros((4,1))
        error[0] = desired_state[0] - state[0]
        error[1] = desired_state[1] - state[1]
        error[2] = desired_state[2] - state[2]
        error[3] = desired_state[3] - state[3]

        return error

def initQuadMPC():
    #should allow user to map these parameters to a yaml file
    simple_quad_model = SimpleQuadModel.SimpleQuadModel()
    simple_quad_model.set_state_space()

    simple_quad_constraint_params = {
        'vx_max': 15.0, #m/s
        'vx_min': 0.0, #m/s

        'vy_max': 15.0, #m/s
        'vy_min': 0.0, #m/s

        'vz_max': 5.0, #m/s
        'vz_min': 3.0, #m/s

        'psi_dot_max': np.deg2rad(5.0),#rad/s
        'psi_dot_min': np.deg2rad(-5.0),#rad/s

        'z_min': 20.0, #m
        'z_max': 50.0, #m
    }

    simple_mpc_quad_params = {
        'model': simple_quad_model,
        'N': 25,
        'dt_val': 0.1,
        'Q': np.diag([1, 1, 1, 0.1]),
        'R': np.diag([1, 1, 1, 1.0])
    }

    quad_mpc = FlatQuadMPC(simple_mpc_quad_params, simple_quad_constraint_params)

    return quad_mpc

def connectMAVUTIL(ip_string:str):
    master = mavutil.mavlink_connection(ip_string)
    master.wait_heartbeat()
    return master

def convertToNED(x_enu:float, y_enu:float, z_enu:float) -> np.ndarray:
    """converts from ENU to NED"""
    ned = np.zeros((3,1))
    ned[0] = y_enu
    ned[1] = x_enu
    ned[2] = -z_enu
    return ned

def convertToENU(x_ned:float, y_ned:float, z_ned:float) -> np.ndarray:
    """converts from NED to ENU"""
    enu = np.zeros((3,1))
    enu[0] = y_ned
    enu[1] = x_ned
    enu[2] = -z_ned
    return enu

def mavSendVelocityCMD(master, vx:float, 
    vy:float, vz:float, yaw:float):
    """sends velocity commands in NED world frame"""
    vel_ned = convertToNED(vx, vy, vz) #ned wrt to world frame
    
    master.mav.set_position_target_local_ned_send(
        0, #time_boot_ms
        master.target_system, #target_system
        master.target_component, #target_component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, #coordinate frame
        0b0000111111000111, #type_mask #enable vx, vy, vz, afx, afy, afz
        # 0b110111111000, #type_mask #enable position
        0, #x
        0, #y
        0, #z
        vel_ned[0], #vx
        vel_ned[1], #vy
        0, #vz
        0, #afx
        0, #afy
        0, #afz
        yaw, #yaw
        0) #yaw_rate 

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
    desired_state = [Config.GOAL_X, 
                     Config.GOAL_Y, 
                     quad_node.state_info[2]-25.0, 
                     quad_node.state_info[3]]

    projected_controls, projected_states = quad_mpc.warmUpSolution(
        quad_node.state_info, desired_state)

    print("goal state: ", desired_state)

    error_tolerance = 5.0 #meters

    while rclpy.ok() and time.time() - t0 < t_sim_limit:

        rclpy.spin_once(quad_node)

        # if current_state == [0,0,0,0,0,0,0,0]:
        if quad_node.state_info == [0,0,0,0]:
            rclpy.spin_once(quad_node)
            projected_controls, projected_states = quad_mpc.warmUpSolution(
                quad_node.state_info, desired_state)

            continue

        rclpy.spin_once(quad_node)
        projected_controls, projected_states = quad_mpc.solveMPCRealTimeStatic(
            quad_node.state_info,
            desired_state)            

        x_traj = projected_states[0,:]
        y_traj = projected_states[1,:]
        z_traj = projected_states[2,:]
        psi_traj = projected_states[3,:]
 
        control_idx_num = -1 
        vx_control = projected_controls[0, control_idx_num]
        vy_control = projected_controls[1, control_idx_num]
        vz_control = projected_controls[2, control_idx_num]
        yaw_control = projected_controls[3, control_idx_num]

        index_num = -2

        ref_state = [x_traj[index_num],
                    y_traj[index_num],
                    z_traj[index_num],
                    psi_traj[index_num]]

        ref_control = [vx_control, vy_control, vz_control, yaw_control]


        error = quad_node.computeError(quad_node.state_info, ref_state)

        
        # control_error = quad_node.computeError(quad_node.control_info, ref_control)
        goal_error = quad_node.computeError(quad_node.state_info, desired_state)
        # print("goal error: ", goal_error)
        distance_error = np.linalg.norm(goal_error[0:2])
                        
        if distance_error < error_tolerance:
            #shut down the node
            quad_node.destroy_node()
            rclpy.shutdown()
            return 

        print("distance error: ", distance_error)
        # mavSendVelocityCMD(master, 
        #                     goal_error[0], 
        #                     goal_error[1], 
        #                     goal_error[2], 
        #                     goal_error[3])

        # mavSendVelocityCMD(master,
        #                     -control_error[0],
        #                     -control_error[1],
        #                     -control_error[2],
        #                     error[3])

        # mavSendVelocityCMD(master,
        #                     ref_control[0],
        #                     ref_control[1],
        #                     ref_control[2],
        #                     ref_control[3])

        # mavSendVelocityCMD(master, 
        #                     error[0], 
        #                     error[1], 
        #                     -error[2], 
        #                     error[3])

        rclpy.spin_once(quad_node)


if __name__ == '__main__':
    main()