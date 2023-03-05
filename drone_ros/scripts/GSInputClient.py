#!/usr/bin/env python3
import rclpy 
import os
import json
import ast
from rclpy.node import Node
from drone_ros.srv import SetArmDisarm, SetGoal, SetTakeOff, SetUASType
from drone_ros.DroneInterfaceModel import DroneInterfaceModel


"""
Model View Controller Interface

Model - The dictionary we can query for information about the drone
View - The CLI that displays information about the drone
Controller - The class that handles user input and sends it to the drone
"""

class GSInputClient(Node):
    def __init__(self):
        super().__init__('gs_client')
        
        self.armdisarm_client = self.create_client(
            SetArmDisarm, 'set_arm_disarm')
        
        self.takeoff_client = self.create_client(
            SetTakeOff, 'set_takeoff')
        
        self.goal_client = self.create_client(
            SetGoal, 'set_goal')
        
        self.uas_config_client = self.create_client(
            SetUASType, 'set_uas_type')
        
    def sendArmDisarmRequest(self, arm_disarm: int):
        self.armdisarm_request = SetArmDisarm.Request()
        self.armdisarm_request.arm_disarm = arm_disarm
        self.future = self.armdisarm_client.call_async(self.armdisarm_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def sendTakeOffRequest(self, takeoff: int, takeoff_height: float):
        self.takeoff_request = SetTakeOff.Request()
        self.takeoff_request.takeoff = takeoff
        self.takeoff_request.height = takeoff_height
        self.future = self.takeoff_client.call_async(self.takeoff_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def sendGoalRequest(self, goal: list):
        self.goal_request = SetGoal.Request()
        self.goal_request.x = float(goal[0])
        self.goal_request.y = float(goal[1])
        self.goal_request.z = float(goal[2])
        self.future = self.goal_client.call_async(self.goal_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def sendUASConfigRequest(self, uas_config: str):
        self.uas_config_request = SetUASType.Request()
        self.uas_config_request.uas_config = uas_config
        self.future = self.uas_config_client.call_async(self.uas_config_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
class UserInputVerifier():
    
    def checkArmInput(self, user_input: str):
        if user_input == "arm":
            return 1
        if user_input == "disarm":
            return 0
        else:
            return -1

    def checkTakeoffInput(self, user_input: str):
        if user_input == "takeoff":
            return 1
        elif user_input == "land":
            return 0
        else:
            return -1

    def checkHeightInput(self, user_input: str):
        try:
            user_input = float(user_input)

            if user_input < 0:
                return -1
            
            return user_input
        except:
            return -1        

    def checkUASConfigInput(self, user_input:str):
        if user_input == "Quadrotor":
            return "Quadrotor"
        elif user_input == "VTOL":
            return "VTOL"
        else:
            return -1

    def checkGoalInput(self, user_input: str):
        #parse user input to get float
        goal_position = [] 

        try:
            goal_position = ast.literal_eval(user_input)
        except:
            print("Invalid input for goal position")
            return -1

        if len(goal_position) != 3:
            return -1
        
        if goal_position[2] < 0:
            return -1
        
        return goal_position
    
        
class InputController(Node):
    def __init__(self):
        super().__init__('gs_client')

        self.user_input_verifier = UserInputVerifier()
        self.gs_input_client = GSInputClient()
        
        self.drone_interface_model = DroneInterfaceModel()
        self.drone_dict_cmds = {}

        self.initInterfacesInfo()
        self.initDroneCMDInterfaces()

        self.view = ViewCommandLine(self.interface_cmds, self.interface_info,
                                    self.drone_cmds, self.drone_cmd_info)
                                    
    def initInterfacesInfo(self) -> None:
        self.interface_cmds = [
            'help',
            'add',
            'remove',
            'show',
            'send'
        ]

        self.interface_info = [
            'help instructions',
            'add drone commands to the database',
            'remove drone commands from the database',
            'show drone commands in the database',
            'send drone commands to the drone'
        ]

    def initDroneCMDInterfaces(self) -> None:
        self.drone_cmds = [
            'arm',
            'disarm',
            'takeoff',
            'land',
            'goal',
            'UAS config',
            'exit'
        ]

        self.drone_cmd_info = [
            'arm the drone',
            'disarm the drone',
            'takeoff the drone, specify your height in meters',
            'land the drone',
            'set a goal for the drone, specify your goal in meters (x, y, z)',
            'set the UAS configuration, specify your configuration (Quadrotor or VTOL)',
            'exit the drone commands interface'
        ]

    def takeInput(self, input_request:str):

        if input_request == self.interface_cmds[0]:
            self.view.displayHelp()
            print('\n')

        if input_request  == self.interface_cmds[1]:
            self.addDroneCommands()
            print('\n')

        if input_request == self.interface_cmds[2]:
            pass
            print('\n')
            #self.removeDroneCommands()

        if input_request == self.interface_cmds[3]:
            model = self.drone_interface_model.getModelInfo()
            self.view.displayDroneInfo(model)

        if input_request == self.interface_cmds[-1]:
            self.sendDroneCommands()
            print('\n')


    def addDroneCommands(self):
        self.view.displayDroneCMDS()

        while True:
        
            user_input = input("Drone Command: ")

            if user_input == self.drone_cmds[0]:
                self.droneArmDisarmUserFunction(user_input)

            if user_input == self.drone_cmds[1]:
                self.droneArmDisarmUserFunction(user_input)

            if user_input == self.drone_cmds[2]:
                self.takeoffUserFunction(user_input)

            if user_input == self.drone_cmds[3]:
                pass
                #self.takeoffUserFunction(user_input)

            if user_input == self.drone_cmds[4]:
                self.uasGoalFunction()
            
            if user_input == self.drone_cmds[5]:
                self.uasConfigFunction(user_input)

            if user_input == self.drone_cmds[-1]:
                break

    def sendDroneCommands(self):
        model = self.drone_interface_model
        model_info = model.getModelInfo()

        # important_keys = ['set_arm_disarm', 'set_takeoff', 'set_goal', 'set_uas_config']
        important_keys = ['set_arm_disarm', 'set_takeoff', 'set_goal', 'set_type']

        for key in important_keys:
            if model_info[key] is None:
                print("Missing command: ", key)
                return

        arm_response = self.gs_input_client.sendArmDisarmRequest(model.getArmDisarm())
        takeoff_response = self.gs_input_client.sendTakeOffRequest(model.getTakeoff(), 
                                                                   model.getTakeoffHeight())        
        goal_response = self.gs_input_client.sendGoalRequest(model.getGoal())
        uas_config_response = self.gs_input_client.sendUASConfigRequest(model.getType())

        print("Arm Disarm Response: ", arm_response.success)
        print("Takeoff Response: ", takeoff_response.success)
        print("Goal Response: ", goal_response.success)
        print("UAS Config Response: ", uas_config_response.success)
        print('\n')

    def droneArmDisarmUserFunction(self, user_input: str) -> None:
        arm_disarm = self.user_input_verifier.checkArmInput(user_input)
        if arm_disarm == -1:
            print("Invalid input")
        else:
            print("Arm Disarm: ", arm_disarm)
            self.drone_interface_model.setArmDisarm(arm_disarm)

    def takeoffUserFunction(self, user_input: str) -> None:
        takeoff_input = self.user_input_verifier.checkTakeoffInput(user_input)
        if takeoff_input == -1:
            print("Invalid input")
        
        print("to exit type 'exit'")

        while True:
            height_input = input("Input takeoff height in meters: ")
            
            if height_input == 'exit':
                return

            height_input = self.user_input_verifier.checkHeightInput(height_input)

            if height_input == -1:
                print("Please input a valid height in meters")
            else:
                self.drone_interface_model.setTakeoff(
                    takeoff_input, height_input)
                print("Takeoff: ", takeoff_input, "Height: ", height_input)
                return
            
    def uasConfigFunction(self, user_input:str) -> None:

        while True:
            
            config_input = input("Input UAS configuration 'Quadrotor' or 'VTOL', you can type 'exit' to leave: ")

            if config_input == 'exit':
                return

            uas_config = self.user_input_verifier.checkUASConfigInput(config_input)    
            if uas_config == -1:
                print("Invalid input")
            
            else:
                self.drone_interface_model.setType(uas_config)
                print("UAS configuration: ", uas_config)
                return
            
    def uasGoalFunction(self) -> None:
        while True:
            goal_input = input("Input goal in meters x, y, z,  you can type 'exit' to leave: ")

            if goal_input == 'exit':
                return

            goal_input = self.user_input_verifier.checkGoalInput(goal_input)

            if goal_input == -1:
                print("Invalid input")
            else:
                self.drone_interface_model.setGoal(goal_input)
                print("Goal: ", goal_input)
                return
                
            
class ViewCommandLine():
    def __init__(self, interface_cmds: list, interface_info: list,
                 drone_cmds: list, drone_cmd_info: list):
        self.cmd_width = 35
        self.description_width = 100
        
        self.interface_cmds = interface_cmds
        self.interface_info = interface_info

        self.drone_cmds = drone_cmds
        self.drone_cmd_info = drone_cmd_info

    def displayHelp(self):
        print('    Ground Station CLI Console Commands:')

        for cmd, info in zip(self.interface_cmds, self.interface_info):
            print(f'        {{:<{self.cmd_width}}}{{:>{self.description_width}}}'\
                .format(cmd, info))

    def displayDroneCMDS(self):
        for cmd, info in zip(self.drone_cmds, self.drone_cmd_info):
            print(f'       {{:<{self.cmd_width}}}{{:>{self.description_width}}}'\
                .format(cmd, info))

    def displayDroneInfo(self, drone_info: dict):
        #print dictionary in a nice format
        print(json.dumps(drone_info, indent=4, sort_keys=True))
        

def main():
    
    rclpy.init(args=None)
    # gs_client = GSInputClient()
    input_controller = InputController()

    while True:
        input_controller.view.displayHelp()
        user_input = input("Enter command: ")
        input_controller.takeInput(user_input)

    # rclpy.spin_once(gs_client)

    # gs_client.destroy_node()
    # rclpy.shutdown()

if __name__ == "__main__":
    main()



