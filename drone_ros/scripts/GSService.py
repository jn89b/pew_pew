#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from drone_ros.srv import SetArmDisarm

class GSService(Node):
    """
    Provides service for user to input commands and allocate to 
    stack of commands

    """
    def __init__(self):
        super().__init__('gs_service')
        
        self.command_dict = {}
        self.srv = self.create_service(SetArmDisarm, 
                                       'set_arm_disarm', 
                                       self.__setArmDisarmCallback)

    def __validateArmDisarm(self, arm_disarm: int) -> bool:
        if arm_disarm == 0 or arm_disarm == 1:
            return True
        else:
            return False

    def __setArmDisarmCallback(self, request, response):
        if self.__validateArmDisarm(request.arm_disarm) == False:
            response.success = False
        else:
            self.command_dict['arm_disarm'] = request.arm_disarm
            response.success = True
            
        print(self.command_dict)

        return response
    

def main():
    rclpy.init(args=None)
    gs_service = GSService()
    rclpy.spin(gs_service)

    gs_service.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

        
