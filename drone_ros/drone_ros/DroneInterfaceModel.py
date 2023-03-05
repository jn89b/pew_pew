class DroneInterfaceModel():
    def __init__(self):

        self.model = {}
        self.model['set_arm_disarm'] = None
        self.model['set_takeoff'] = None
        self.model['set_takeoff_height'] = None
        self.model['set_type'] = None
        self.model['set_goal'] = None
        
    def getModel(self):
        return self.model

    def setArmDisarm(self, arm_disarm: int):
        self.model['set_arm_disarm'] = arm_disarm

    def setTakeoff(self, takeoff: int, takeoff_height: float):
        self.model['set_takeoff'] = takeoff
        self.model['set_takeoff_height'] = takeoff_height

    def setType(self, uas_type: str):
        self.model['set_type'] = uas_type
    
    def setGoal(self, goal: str):
        self.model['set_goal'] = goal

    def getModelInfo(self):
        return self.model
    
    def getArmDisarm(self):
        return self.model['set_arm_disarm']
    
    def getTakeoff(self):
        return self.model['set_takeoff']
    
    def getTakeoffHeight(self):
        return self.model['set_takeoff_height']
    
    def getGoal(self):
        return self.model['set_goal']

    def getType(self):
        return self.model['set_type']