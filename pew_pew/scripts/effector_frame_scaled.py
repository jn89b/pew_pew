#!/usr/bin/env python3


"""
Broadcast static effector reference frame wrt to base frame of UAV
"""

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from ros_mpc.aircraft_config import GOAL_STATE
from rviz_drone.config import SCALE_SIM
from pew_pew.rotation_utils import get_quaternion_from_euler

class EffectorFrameScaled(Node):
    
        def __init__(self):
            super().__init__('effector_frame_scaled')

            self.br = TransformBroadcaster(self)    
            self.declare_parameter('x', 0.0)
            self.declare_parameter('y', 0.0)
            self.declare_parameter('z', 0.0)

            self.declare_parameter('roll', 0.0)
            self.declare_parameter('pitch', 0.0)
            self.declare_parameter('yaw', 0.0)
            self.declare_parameter('parent_frame', 'aircraft_frame_scaled') 
            self.declare_parameter('child_frame', 'effector_frame_scaled')
            self.declare_parameter('rate', 30.0)

            self.x = self.get_parameter('x').value
            self.y = self.get_parameter('y').value
            self.z = self.get_parameter('z').value

            self.roll = self.get_parameter('roll').value
            self.pitch = self.get_parameter('pitch').value
            self.yaw = self.get_parameter('yaw').value

            self.quaternion = get_quaternion_from_euler(
                float(self.roll), float(self.pitch), float(self.yaw))

            self.parent_frame = self.get_parameter('parent_frame').value
            self.child_frame = self.get_parameter('child_frame').value

            self.rate = self.get_parameter('rate').value
            self.timer = self.create_timer(1/self.rate, self.broadcastTransform)

        def broadcastTransform(self):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.parent_frame
            t.child_frame_id = self.child_frame
            t.transform.translation.x = self.x/SCALE_SIM
            t.transform.translation.y = self.y/SCALE_SIM
            t.transform.translation.z = self.z/SCALE_SIM
            t.transform.rotation.x = self.quaternion[0]
            t.transform.rotation.y = self.quaternion[1]
            t.transform.rotation.z = self.quaternion[2]
            t.transform.rotation.w = self.quaternion[3]
    
            self.br.sendTransform(t)    


def main(args=None):
    rclpy.init(args=args)
    node = EffectorFrameScaled()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

