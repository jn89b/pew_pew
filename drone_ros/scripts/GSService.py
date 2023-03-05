#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from drone_ros.srv import SetArmDisarm, SetGoal, \
    SetTakeOff, SetUASType, getGSInfo

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
        

        self.uas_config_srv = self.create_service(SetUASType,
                                        'set_uas_type',
                                        self.__setUASTypeCallback)
        
        self.gs_info_srv = self.create_service(getGSInfo,
                                        'get_gs_info',
                                        self.__getGSInfoCallback)
                                                  
    def __setArmDisarmCallback(self, request, response):

        self.drone_model.setArmDisarm(request.arm_disarm)
        #self.command_dict['arm_disarm'] = request.arm_disarm
        response.success = True
        return response
    
    def __setGoalCallback(self, request, response):
        
        goal = [request.x, request.y, request.z]
        self.drone_model.setGoal(goal)
        # self.command_dict['goal'] = request.goal
        response.success = True
        return response
    
    def __setTakeOffCallback(self, request, response):
        # self.command_dict['takeoff'] = request.takeoff
        self.drone_model.setTakeoff(request.takeoff, request.height)

        response.success = True
        return response
    
    def __setUASTypeCallback(self, request, response):
        # self.command_dict['uas_type'] = request.uas_type
        self.drone_model.setType(request.uas_config)

        response.success = True
        return response
    
    def __getGSInfoCallback(self, request, response):
        model = self.drone_model
        response.arm_disarm = model.getArmDisarm()
        response.goal = model.getGoal()
        response.takeoff = model.getTakeoff()
        response.uas_type = model.getType()
        response.takeoff_height = model.getTakeoffHeight()
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

        
