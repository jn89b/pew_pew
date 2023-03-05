#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from drone_ros.srv import SetArmDisarm, SetGoal, SetTakeOff
from drone_ros.DroneInterfaceModel import DroneInterfaceModel

class GSService(Node):
    """
    Provides service for user to input commands and allocate to 
    stack of commands

    """
    def __init__(self):
        super().__init__('gs_service')
        
        self.drone_model = DroneInterfaceModel()

        self.arm_disarm_srv = self.create_service(SetArmDisarm, 
                                       'set_arm_disarm', 
                                       self.__setArmDisarmCallback)

        self.goal_srv = self.create_service(SetGoal,
                                        'set_goal',
                                        self.__setGoalCallback)
        
        self.takeoff_srv = self.create_service(SetTakeOff,
                                        'set_takeoff',
                                        self.__setTakeOffCallback)
        

    def __setArmDisarmCallback(self, request, response):

        self.drone_model.setArmDisarm(request.arm_disarm)
        #self.command_dict['arm_disarm'] = request.arm_disarm
        response.success = True
        return response
    
    def __setGoalCallback(self, request, response):
        
        self.drone_model.setGoal(request.goal)
        # self.command_dict['goal'] = request.goal
        response.success = True
        return response
    

    def __setTakeOffCallback(self, request, response):
        # self.command_dict['takeoff'] = request.takeoff
        self.drone_model.setTakeoff(request.takeoff, request.height)

        response.success = True
        return response


def main():
    rclpy.init(args=None)
    gs_service = GSService()
    rclpy.spin(gs_service)

    gs_service.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

        
