#!/usr/bin/env python3

import rclpy
import math as m
from mpc_ros.msg import Telem

from pymavlink import mavutil
from rclpy.node import Node
import time
import numpy as np

class MAVListener :
    def __init__(self, master, frequency=5):
        self.master = master
        self.frequency = frequency

    def startListening(self):
        self.requestMessageInterval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 
                                    self.frequency)
        self.requestMessageInterval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                                    self.frequency)
        self.requestMessageInterval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                                    self.frequency)
    
    def getData(self):
        
        output = Telem()
        msg = self.master.recv_match(type=['LOCAL_POSITION_NED', 'ATTITUDE', 'GLOBAL_POSITION_INT'], blocking=True)

#        if msg.get_type() == "GLOBAL_POSITION_INT":
        output.lat = self.master.messages['GLOBAL_POSITION_INT'].lat
        output.lon = self.master.messages['GLOBAL_POSITION_INT'].lon
        output.alt = self.master.messages['GLOBAL_POSITION_INT'].alt
        output.heading = self.master.messages['GLOBAL_POSITION_INT'].hdg


        # if msg.get_type() == "ATTITUDE":
        output.roll = self.master.messages['ATTITUDE'].roll
        output.pitch = self.master.messages['ATTITUDE'].pitch
        output.yaw = self.master.messages['ATTITUDE'].yaw


        # print("x: ", output.x, "y: ", output.y, "z: ", output.z)
#        if msg.get_type() == "LOCAL_POSITION_NED":
        output.x = self.master.messages['LOCAL_POSITION_NED'].x
        output.y = self.master.messages['LOCAL_POSITION_NED'].y
        output.z = self.master.messages['LOCAL_POSITION_NED'].z
        output.vx = self.master.messages['LOCAL_POSITION_NED'].vx
        output.vy = self.master.messages['LOCAL_POSITION_NED'].vy
        output.vz = self.master.messages['LOCAL_POSITION_NED'].vz

        return output 

    def requestMessageInterval(self, message_id:int, frequency_hz:int):
        """
        Request MAVLink message in a desired frequency,
        documentation for SET_MESSAGE_INTERVAL:
            https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

        Args:
            message_id (int): MAVLink message ID
            frequency_hz (float): Desired frequency in Hz
        """
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, # The MAVLink message ID
            1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
            0, 0, 0, 0, # Unused parameters
            0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
        )


class DroneNode(Node):
    def __init__(self, master):
        super().__init__('drone_node')

        self.frequency = 10
        self.master = master
        self.mav_listener = MAVListener(self.master, self.frequency)
        self.mav_listener.startListening()
        self.telem_pub = self.create_publisher(Telem, 'telem', self.frequency)

    def publishData(self):
        self.mav_listener.startListening()
        output = self.mav_listener.getData()
        self.telem_pub.publish(output)


def convertToNED(x_enu:float, y_enu:float, z_enu:float) -> np.ndarray:
    """converts from ENU to NED"""
    ned = np.zeros((3,1))
    ned[0] = y_enu
    ned[1] = x_enu
    ned[2] = -z_enu
    return ned


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
    rclpy.init(args=args)

    master = mavutil.mavlink_connection('127.0.0.1:14551')
    master.wait_heartbeat()


    drone = DroneNode(master)
    drone.publishData()

    while True:
        drone.publishData()
        mavSendVelocityCMD(master, -5, -5, 5, 0)
        # time.sleep(0.1)
        rclpy.spin_once(drone, timeout_sec=0.01)

    
if __name__ == '__main__':
    main()



        
        


        
        


        
 


