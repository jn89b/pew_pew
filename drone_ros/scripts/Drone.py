#!/usr/bin/env python3

import rclpy

from pymavlink import mavutil
from rclpy.node import Node
from drone_ros.Commander import Commander 

class DroneInfo():
    def __init__(self, master) -> None:
        pass
    
class GSListener():
    def __init__(self) -> None:
        pass

    def startListening(self) -> None:
        pass

    def getData(self) -> None:
        pass

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')
        self.get_logger().info('Drone node has been started')

        #drone commander initialization 
        self.declare_parameter('mav_connection_string', 'udp:127.0.0.1:14551')
        self.mav_connection_string = self.get_parameter('mav_connection_string')\
            .get_parameter_value().string_value
        self.get_logger().info('mav_connection_string: ' + self.mav_connection_string)

        self.master = mavutil.mavlink_connection(self.mav_connection_string)
        self.master.wait_heartbeat()

        self.__initDroneCommander()
        self.__initDroneInfo()
        self.__initGSListener()

    def __initDroneCommander(self) -> None:
        self.commander = Commander(self.master)

    def __initDroneInfo(self) -> None:
        self.drone_info = DroneInfo(self.master)

    def __initGSListener(self) -> None:
        self.gs_listener = GSListener()
    

def main(args=None):
    rclpy.init(args=args)
    
    drone_node = DroneNode()
    drone_commander = drone_node.commander

    arm_args = {'arm_disarm': 1}
    drone_commander.armDisarm(arm_args)
    
    mode_args = {'mode': 'GUIDED'}
    drone_commander.changeFlightMode(mode_args)

    takeoff_args = {'altitude': 15}    
    drone_commander.takeoff(takeoff_args)
    
    vel_args = {'vx': 5, 'vy': 0, 'vz': 0, 'set_vz': False}
    drone_commander.sendNEDVelocity(vel_args)

    while True:

        # drone_commander.takeoff(takeoff_args)

        # drone_commander.sendLandCMD(args=None)
        rclpy.spin_once(drone_node, timeout_sec=0.1)



if __name__ == '__main__':
    main()        




        