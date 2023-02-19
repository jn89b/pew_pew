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
from mpc_ros.msg import Telem, CtlTraj

from pymavlink import mavutil
import pickle as pkl

import time 

class FlatQuadMPC(MPC.MPC):

    """this class is responsible for defining the MPC problem
    it inherits from the MPC class in mpc_ros package and overrides
    the addAdditionalConstraints method to add the constraints
    specific to the quadcopter
    """

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

    def returnTrajDictionary(self, projected_controls:list,
        projected_states:list) -> dict:
        
        traj_dictionary = {}
        traj_dictionary['x'] = projected_states[0,:]
        traj_dictionary['y'] = projected_states[1,:]
        traj_dictionary['z'] = projected_states[2,:]
        traj_dictionary['yaw']= projected_states[3,:]

        traj_dictionary['vx'] = projected_controls[0,:]
        traj_dictionary['vy'] = projected_controls[1,:]
        traj_dictionary['vz'] = projected_controls[2,:]
        traj_dictionary['yaw_rate'] = projected_controls[3,:]

        return traj_dictionary


class MPCTrajPublisher(Node):
    """
    
    This node is responsible for publishing the trajectory from MPC
    to the controller node. It also subscribes to the state of the
    quadcopter and publishes the trajectory to the controller node.
    """

    def __init__(self):
        super().__init__('mpc_traj_node')

        self.get_logger().info("Starting MPC Trajectory Node")

        #turn this to a parameter later
        self.mpc_traj_freq = 50

        self.state_sub = self.create_subscription(Telem, 
                        'telem', self.stateCallback, 
                        self.mpc_traj_freq)

        self.traj_pub = self.create_publisher(CtlTraj, 
                        'trajectory', 
                        self.mpc_traj_freq)

        self.state_info = [0, #x
                           0, #y
                           0, #z
                           0] #psi

        self.control_info = [0, #vx
                             0, #vy
                             0, #vz
                             0] #psi_dot


    def stateCallback(self,msg:Telem) -> None:
        """stateCallback"""
        enu_coords = quaternion_tools.convertNEDToENU(
            msg.x, msg.y, msg.z)
        
        self.state_info[0] = enu_coords[0]
        self.state_info[1] = enu_coords[1]
        self.state_info[2] = enu_coords[2]
        self.state_info[3] = -msg.yaw

        enu_vel = quaternion_tools.convertNEDToENU(
            msg.vx, msg.vy, msg.vz)
        
        self.control_info[0] = enu_vel[0]
        self.control_info[1] = enu_vel[1]
        self.control_info[2] = enu_vel[2]
        self.control_info[3] = -msg.yaw_rate
    
    def computeError(self, current_state:list, desired_state:list) -> list:
        """computeError"""
        error = []
        for i in range(len(current_state)):
            error.append(desired_state[i] - current_state[i])
        return error

    def publishTrajectory(self, ref_position:list, 
                        ref_velocity:list) -> None:
        """
        publishTrajectory to the controller node , this 
        will take in the ENU reference position and velocity 
        and convert it to NED and publish it to the controller node
        """
        traj_msg = CtlTraj()
        # traj_msg.header.stamp = self.get_clock().now().to_msg()
        # traj_msg.header.frame_id = "map"

        ref_position = [float(i) for i in ref_position]
        ref_velocity = [float(i) for i in ref_velocity]

        ned_position = quaternion_tools.convertENUToNED(
            ref_position[0], ref_position[1], ref_position[2])

        ned_velocity = quaternion_tools.convertENUToNED(
            ref_velocity[0], ref_velocity[1], ref_velocity[2])

        #if ned_position is None or ned_velocity is None:
        traj_msg.x = ned_position[0]
        traj_msg.y = ned_position[1]
        traj_msg.z = ned_position[2]
        print("state_info: ", self.state_info[3])
        traj_msg.yaw = -float(self.state_info[3])

        print("ned_velocity: ", ned_velocity)
        # print("ned_position: ", ned_position)
        traj_msg.vx = ned_velocity[0]#ned_velocity[0]
        traj_msg.vy = ned_velocity[1]#ned_velocity[1]
        traj_msg.vz = ned_velocity[2]#ned_velocity[2]
        traj_msg.yaw_rate = -float(self.state_info[3])
        
        self.traj_pub.publish(traj_msg)

def initQuadMPC():
    #should allow user to map these parameters to a yaml file
    simple_quad_model = SimpleQuadModel.SimpleQuadModel()
    simple_quad_model.set_state_space()

    simple_quad_constraint_params = {
        'vx_max': 8.0, #m/s
        'vx_min': -8.0, #m/s

        'vy_max': 8.0, #m/s
        'vy_min': -8.0, #m/s

        'vz_max': 5.0, #m/s
        'vz_min': 3.0, #m/s

        'psi_dot_max': np.deg2rad(5.0),#rad/s
        'psi_dot_min': np.deg2rad(5.0),#rad/s

        'z_min': 5, #m
        'z_max': 100, #m
    }

    simple_mpc_quad_params = {
        'model': simple_quad_model,
        'N': 25,
        'dt_val': 0.15,
        'Q': np.diag([1.0, 1.0, 1, 0.5]),
        'R': np.diag([1, 1, 1, 1])
    }

    quad_mpc = FlatQuadMPC(simple_mpc_quad_params, simple_quad_constraint_params)

    return quad_mpc


def get_state_control_ref(traj_dictionary:dict, 
    state_idx:int, ctrl_idx:int) -> tuple:
    """get_state_control_ref"""
    x_ref = traj_dictionary['x'][state_idx]
    y_ref = traj_dictionary['y'][state_idx]
    z_ref = traj_dictionary['z'][state_idx]
    psi_ref = traj_dictionary['yaw'][state_idx]

    vx_ref = traj_dictionary['vx'][ctrl_idx]
    vy_ref = traj_dictionary['vy'][ctrl_idx]
    vz_ref = traj_dictionary['vz'][ctrl_idx]
    psi_dot_ref = traj_dictionary['yaw_rate'][ctrl_idx]
    
    return [x_ref, y_ref, z_ref, psi_ref], \
        [vx_ref, vy_ref, vz_ref, psi_dot_ref]


def set_state_control_idx(mpc_params:dict, 
    solution_time:float, idx_buffer:int) -> int:
    """set index based on solution time"""
    time_rounded = round(solution_time, 1)
    
    if time_rounded <= 1:
        time_rounded = 1.0

    control_idx = mpc_params['dt_val']/time_rounded
    idx = int(round(control_idx)) + idx_buffer
    
    return idx

def main(args=None):
    rclpy.init(args=args)

    #turn this into a parameter later

    control_idx = 10
    state_idx = -2
    time_out_sec = 0.1
    dist_error_tol = 5.0
    idx_buffer = 2

    quad_mpc = initQuadMPC()
    mpc_traj_node = MPCTrajPublisher()
    rclpy.spin_once(mpc_traj_node, timeout_sec=time_out_sec)

    desired_state = [Config.GOAL_X, 
                     Config.GOAL_Y, 
                     mpc_traj_node.state_info[2] + 5, 
                     mpc_traj_node.state_info[3]]

    #get the time to find solution 
    start_time = time.time()
    projected_controls, projected_states = quad_mpc.warmUpSolution(
        mpc_traj_node.state_info,
        desired_state)

    traj_dictionary = quad_mpc.returnTrajDictionary(
        projected_controls, projected_states)

    traj_state, traj_control = get_state_control_ref(
        traj_dictionary, state_idx, control_idx)

    ref_state_error = mpc_traj_node.computeError(
        mpc_traj_node.state_info, traj_state)

    ref_control_error = mpc_traj_node.computeError(
        mpc_traj_node.control_info, traj_control)

    end_time = time.time()
    print("Solve time: ", end_time - start_time)

    control_idx = set_state_control_idx(quad_mpc.mpc_params, 
        end_time - start_time, idx_buffer=idx_buffer)

    state_idx = set_state_control_idx(quad_mpc.mpc_params,
        end_time - start_time, idx_buffer=idx_buffer)

    # mpc_traj_node.publishTrajectory(
    #     ref_state_error, ref_control_error)
    mpc_traj_node.publishTrajectory(
        traj_state, traj_control)

    while rclpy.ok():

        rclpy.spin_once(mpc_traj_node, timeout_sec=time_out_sec)
    
        start_time = time.time()
        projected_controls, projected_states = quad_mpc.solveMPCRealTimeStatic(
            mpc_traj_node.state_info,
            desired_state)

        end_time = time.time()

        traj_dictionary = quad_mpc.returnTrajDictionary(
            projected_controls, projected_states)

        traj_state, traj_control = get_state_control_ref(
            traj_dictionary, state_idx, control_idx)

        ref_state_error = mpc_traj_node.computeError(
            mpc_traj_node.state_info, traj_state)

        ref_control_error = mpc_traj_node.computeError(
            mpc_traj_node.control_info, traj_control)

        end_time = time.time()

        control_idx = set_state_control_idx(quad_mpc.mpc_params, 
            end_time - start_time, idx_buffer=idx_buffer)

        state_idx = set_state_control_idx(quad_mpc.mpc_params,
            end_time - start_time, idx_buffer=idx_buffer)

        print("state_idx: ", state_idx)

        mpc_traj_node.publishTrajectory(
            traj_state, traj_control)

        goal_state_error = mpc_traj_node.computeError(
            mpc_traj_node.state_info, desired_state)

        distance_error = np.linalg.norm(goal_state_error[0:2])
        
        print("distance error: ", distance_error)
        
        if distance_error < dist_error_tol:
            #send 0 velocity command
            ref_state_error = [0.0, 0.0, 0.0, 0.0]
            ref_control_error = [0.0, 0.0, 0.0, 0.0]
            mpc_traj_node.publishTrajectory(
                ref_state_error, ref_control_error)
            
            mpc_traj_node.destroy_node()
            rclpy.shutdown()
            return 


        desired_state = [Config.GOAL_X, 
                        Config.GOAL_Y, 
                        mpc_traj_node.state_info[2], 
                        traj_dictionary['yaw'][state_idx]]

        rclpy.spin_once(mpc_traj_node, timeout_sec=time_out_sec)
    #print("goal state: ", mpc_traj_node.state_info)

if __name__ == '__main__':
    main()



        

