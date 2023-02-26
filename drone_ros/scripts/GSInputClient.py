#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from drone_ros.srv import SetArmDisarm

class GSInputClient(Node):
    def __init__(self):
        super().__init__('gs_client')

        self.armdisarm_client = self.create_client(
            SetArmDisarm, 'set_arm_disarm')
        self.armdisarm_request = SetArmDisarm.Request()

    def sendArmDisarmRequest(self, arm_disarm: int):
        self.armdisarm_request.arm_disarm = arm_disarm
        self.future = self.armdisarm_client.call_async(self.armdisarm_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    
class UserInput():
    pass

    def checkInput(self, user_input: str):
        if user_input == "arm":
            return 1
        elif user_input == "disarm":
            return 0
        else:
            return -1

def main():
    
    rclpy.init(args=None)
    gs_client = GSInputClient()
    
    while True:
        user_input = input("Enter command: ")
        if user_input == "arm":
            arm_disarm = 1
            response = gs_client.sendArmDisarmRequest(arm_disarm)
            print(response.success)
            
        elif user_input == "disarm":
            arm_disarm = 0
            response = gs_client.sendArmDisarmRequest(arm_disarm)
        else:
            print("Invalid command")
            continue
        
    # rclpy.spin_once(gs_client)

    # gs_client.destroy_node()
    # rclpy.shutdown()

if __name__ == "__main__":
    main()



