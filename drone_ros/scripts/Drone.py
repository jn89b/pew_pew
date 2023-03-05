#!/usr/bin/env python3

import rclpy

from pymavlink import mavutil
from rclpy.node import Node
from drone_ros.Commander import Commander
from drone_ros.DroneInfo import DroneInfo
from drone_ros.msg import Telem, CtlTraj
from drone_ros.srv import getGSInfo

from drone_ros.DroneInterfaceModel import DroneInterfaceModel

class GSListenerClient(Node):
    """
    Make this as a service
    """
    def __init__(self) -> None:
        super().__init__('gs_listener_client')

        self.model_info = DroneInterfaceModel()
        self.info_client = self.create_client(getGSInfo, 'gs_listener')

        while not self.info_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
    def sendInfoRequest(self):
        request = getGSInfo.Request()
        future = self.info_client.call_async(request)
        # future.add_done_callback(self.__infoResponseCallback)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()

    def mapInfo(self, response):
        self.model_info.setArmDisarm(response.arm_disarm)
        self.model_info.setGoal(response.goal)
        self.model_info.setTakeoff(response.takeoff, response.height)
        self.model_info.setUASType(response.uas_type)
        

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')
        self.get_logger().info('Drone node has been started')

        #frequency interval
        self.declare_parameter('drone_node_frequency', 50)
        self.drone_node_frequency = self.get_parameter(
            'drone_node_frequency').get_parameter_value().integer_value

        self.__initMasterConnection()        
        self.__initPublishers()

        self.commander = Commander(self.master)
        self.gs_listener = GSListenerClient()
        self.drone_info = DroneInfo(self.master, 
                                    self.telem_publisher,
                                    self.drone_node_frequency)
        
        self.__initSubscribers()

        self.vel_args = {}

    def __initMasterConnection(self) -> None:
        #drone commander initialization 
        self.declare_parameter('mav_connection_string', 'udp:127.0.0.1:14551')
        self.mav_connection_string = self.get_parameter('mav_connection_string')\
            .get_parameter_value().string_value
        self.get_logger().info('mav_connection_string: ' + self.mav_connection_string)

        self.master = mavutil.mavlink_connection(self.mav_connection_string)
        self.master.wait_heartbeat()

    def __initPublishers(self)-> None:
        self.telem_publisher = self.create_publisher(
            Telem, 'telem', self.drone_node_frequency)        

    def __initSubscribers(self) -> None:
        self.telem_sub = self.create_subscription(
            Telem,
            'telem',
            self.__telemCallback,
            self.drone_node_frequency)
        
        self.traj_sub = self.create_subscription(
            CtlTraj,
            'trajectory',
            self.__trajCallback,
            self.drone_node_frequency)

    def __telemCallback(self, msg:Telem) -> None:
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt
        
        self.ground_vel = [msg.vx, 
                           msg.vy, 
                           msg.vz]
        
        self.heading = msg.heading
        
        self.attitudes = [msg.roll, 
                          msg.pitch, 
                          msg.yaw]
        
        self.attitude_rates = [msg.roll_rate, 
                               msg.pitch_rate, 
                               msg.yaw_rate]
        
        self.ned_position = [msg.x,
                             msg.y,
                             msg.z]
        

    def __trajCallback(self, msg: CtlTraj):
        x_traj = msg.x
        y_traj = msg.y
        z_traj = msg.z
        yaw_traj = msg.yaw

        vx_traj = msg.vx
        vy_traj = msg.vy
        vz_traj = msg.vz
        idx_command = msg.idx

        vel_args = {'vx': vx_traj[idx_command],
                    'vy': vy_traj[idx_command],
                    'vz': vz_traj[idx_command],
                    'set_vz': False}
        
        # print(vel_args)        
        self.commander.sendNEDVelocity(vel_args)

    def beginTakeoffLand(self, altitude: float) -> None:
        self.commander.takeoff(altitude)
      
def main(args=None):
    rclpy.init(args=args)
    
    drone_node = DroneNode()
    drone_commander = drone_node.commander
    drone_info = drone_node.drone_info

    arm_args = {'arm_disarm': 1}
    drone_commander.armDisarm(arm_args)
    
    mode_args = {'mode': 'GUIDED'}
    drone_commander.changeFlightMode(mode_args)
    takeoff_args = {'altitude': 15}    
    
    for i in range(100):
        drone_commander.takeoff(takeoff_args)
        rclpy.spin_once(drone_node, timeout_sec=0.1)
        
    vel_args = {'vx': 5, 'vy': 0, 'vz': 0, 'set_vz': False}
    drone_commander.sendNEDVelocity(vel_args)

    # rclpy.spin(drone_node)
    while rclpy.ok():
        drone_info.publishTelemInfo()
        rclpy.spin_once(drone_node, timeout_sec=0.1)


if __name__ == '__main__':
    main()        




        