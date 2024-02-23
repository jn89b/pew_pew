#!/usr/bin/env python3

import casadi as ca
import rclpy
import numpy as np
import time 

from drone_interfaces.msg import Telem, CtlTraj
from rclpy.node import Node

from ros_mpc.PlaneOptControl import PlaneOptControl
from ros_mpc.Effector import Effector
from ros_mpc.aircraft_config import mpc_params, directional_effector_config, \
    control_constraints, state_constraints, obs_avoid_params
from ros_mpc.aircraft_config import GOAL_STATE, DONE_STATE

import ros_mpc.rotation_utils as rot_utils
from ros_mpc.models.Plane import Plane

"""
- Set Effector Configurations
- Set Aircraft Configurations
- Set MPC Configurations

- Initialize Aircraft Model and 

"""

# class DirectionalNode(Node):
#     def __init__(self):
#         super().__init__('traj_publisher_node')
#         self.declare_parameters(
#             namespace='',
#             parameters=[
#                 ('N', 20),
#                 ('Q', [1, 1, 1, 1, 1, 1]),
#                 ('R', [1, 1]),
#                 ('dt', 0.1),
#             ]
#         )
#         self.get_params()
#         self.init_ros()
#         self.init_mpc()
#         self.init_effector()

class TrajPublisherNode(Node):
    def __init__(self, 
                 pub_freq:int=100, 
                 sub_freq:int=100,
                 save_states:bool=False):
        super().__init__('mpc_traj_fw_publisher')
        self.get_logger().info('Starting MPC Traj FW Publisher')
        
        self.pub_freq = pub_freq
        self.sub_freq = sub_freq
        
        #flag this to save states and cache it for later if needed
        self.save_states = save_states
        
        self.state_info =[
            None, #x
            None, #y
            None, #z
            None, #phi
            None, #theta
            None, #psi
            None, #airspeed
        ]
        
        self.control_info = [
            None, #u_phi
            None, #u_theta
            None, #u_psi
            None  #v_cmd
        ]
        
        self.traj_pub = self.create_publisher(
            CtlTraj, 
            'trajectory', 
            self.pub_freq)
        
        self.state_sub = self.create_subscription(Telem, 
            'telem', 
            self.state_callback, 
            self.sub_freq)
        
        if self.save_states:
            self.init_history()

    def init_history(self) -> None:
        self.x_history = []
        self.y_history = []
        self.z_history = []
        self.phi_history = []
        self.theta_history = []
        self.psi_history = []
        self.u_phi_history = []
        self.u_theta_history = []
        self.u_psi_history = []
        self.v_cmd_history = []
        

    def state_callback(self, msg:Telem) -> None:
        
        enu_coords = rot_utils.convertNEDToENU(
            msg.x, msg.y, msg.z)
        # positions
        self.state_info[0] = enu_coords[0]
        self.state_info[1] = enu_coords[1]
        self.state_info[2] = enu_coords[2]

        #wrap yaw to 0-360
        self.state_info[3] = msg.roll
        self.state_info[4] = -msg.pitch
        self.state_info[5] = msg.yaw #flip the yaw to match ENU frame
        self.state_info[6] = np.sqrt(msg.vx**2 + msg.vy**2 + msg.vz**2)

        #rotate roll and pitch rates to ENU frame   
        self.control_info[0] = msg.roll_rate
        self.control_info[1] = -msg.pitch_rate
        self.control_info[2] = msg.yaw_rate
        self.control_info[3] = np.sqrt(msg.vx**2 + msg.vy**2 + msg.vz**2) 
        
        # positions
        # self.state_info[0] = msg.x
        # self.state_info[1] = msg.y
        # self.state_info[2] = msg.z
        
        # #wrap yaw to 0-360
        # self.state_info[3] = msg.roll
        # self.state_info[4] = msg.pitch
        # self.state_info[5] = msg.yaw #flip the yaw to match ENU frame
        # self.state_info[6] = np.sqrt(msg.vx**2 + msg.vy**2 + msg.vz**2)

        # #rotate roll and pitch rates to ENU frame   
        # self.control_info[0] = msg.roll_rate
        # self.control_info[1] = msg.pitch_rate
        # self.control_info[2] = msg.yaw_rate
        # self.control_info[3] = np.sqrt(msg.vx**2 + msg.vy**2 + msg.vz**2) 

    def publish_trajectory(self, solution_results:dict, idx_step:int) -> None:
        x = solution_results['x']
        y = solution_results['y']
        z = solution_results['z']
        phi = solution_results['phi']
        theta = -solution_results['theta']#have to flip sign to match NED to ENU
        psi = solution_results['psi']
        
        u_phi = solution_results['u_phi']
        u_theta = -solution_results['u_theta']#have to flip sign to match NED to ENU
        u_psi = solution_results['u_psi']
        v_cmd = solution_results['v_cmd']
        
        x_ned = y
        y_ned = x
        z_ned = -z
        
        traj_msg = CtlTraj()
        #make sure its a list
        traj_msg.x = x_ned.tolist()
        traj_msg.y = y_ned.tolist()
        traj_msg.z = z_ned.tolist()
        traj_msg.roll = phi.tolist()
        traj_msg.pitch = theta.tolist()
        traj_msg.yaw = psi.tolist()
        traj_msg.roll_rate = u_phi.tolist()
        traj_msg.pitch_rate = u_theta.tolist()
        traj_msg.yaw_rate = u_psi.tolist()
        traj_msg.vx = v_cmd.tolist()

        traj_msg.idx = idx_step
        
        # x = solution_results['x']
        # y = solution_results['y']
        # z = solution_results['z']
        # phi = solution_results['phi']
        # theta = solution_results['theta']#have to flip sign to match NED to ENU
        # psi = solution_results['psi']
        # print('Psi in degrees: ', np.rad2deg(psi))
        
        # u_phi = solution_results['u_phi']
        # u_theta = solution_results['u_theta']#have to flip sign to match NED to ENU
        # u_psi = solution_results['u_psi']
        # v_cmd = solution_results['v_cmd']
        
        # x_ned = x
        # y_ned = y
        # z_ned = z
        
        # traj_msg = CtlTraj()
        # #make sure its a list
        # traj_msg.x = x_ned.tolist()
        # traj_msg.y = y_ned.tolist()
        # traj_msg.z = z_ned.tolist()
        # traj_msg.roll = phi.tolist()
        # traj_msg.pitch = theta.tolist()
        # traj_msg.yaw = psi.tolist()
        # traj_msg.roll_rate = u_phi.tolist()
        # traj_msg.pitch_rate = u_theta.tolist()
        # traj_msg.yaw_rate = u_psi.tolist()
        # traj_msg.vx = v_cmd.tolist()

        # traj_msg.idx = idx_step
        
        
        self.traj_pub.publish(traj_msg)

    def get_time_idx(self, mpc_params:dict, 
                     solution_time:float, idx_buffer:int=0) -> int:
        time_rounded = round(solution_time, 1)
        
        if time_rounded <= 1:
            time_rounded = 1
        
        ctrl_idx = mpc_params['dt']/time_rounded
        idx = int(round(ctrl_idx)) + idx_buffer
        
        return idx

def main(args=None) -> None:
    rclpy.init(args=args)    

    traj_node = TrajPublisherNode()
    rclpy.spin_once(traj_node)
    plane = Plane()
    plane_mpc = PlaneOptControl(
        control_constraints=control_constraints,
        state_constraints=state_constraints,
        mpc_params=mpc_params,
        casadi_model=plane,
        # use_pew_pew=False,
        # pew_pew_params=directional_effector_config,
        # use_obstacle_avoidance=False,
        # obs_params=obs_avoid_params
    )
    plane_mpc.init_optimization_problem()   

    counter = 0
    print_every = 10

    goal = GOAL_STATE
    idx_buffer = 2
    
    distance_tolerance = 5.0
    
    while rclpy.ok():
        rclpy.spin_once(traj_node)

        start_time = time.time()

        distance_error = np.sqrt(
            (goal[0] - traj_node.state_info[0])**2 + 
            (goal[1] - traj_node.state_info[1])**2 
        )        
        # print('State Info: ', traj_node.state_info)

        goal = [goal[0], goal[1], goal[2], 
                traj_node.state_info[3], 
                traj_node.state_info[4], 
                traj_node.state_info[5], 
                traj_node.state_info[6]]
        
        solution_results,end_time = plane_mpc.get_solution(traj_node.state_info, 
                                                           goal, traj_node.control_info,
                                                           get_cost=True)
        
        
        idx_step = traj_node.get_time_idx(mpc_params, end_time - start_time, idx_buffer)
        
        if counter % print_every == 0:
            print(traj_node.state_info)
            print(idx_step)
            print('Distance Error: ', distance_error)
        
        if distance_error <= distance_tolerance:
            #kill node
            #log 
            traj_node.get_logger().info('Goal Reached Shutting Down Node') 
            traj_node.destroy_node()
            rclpy.shutdown()
            return    
        
        traj_node.publish_trajectory(solution_results, idx_step)
        counter += 1
        # plane_mpc.solve()
        # plane_mpc.publish_trajectory()
        # rclpy.spin_once(traj_node)
    

if __name__=='__main__':
    main()