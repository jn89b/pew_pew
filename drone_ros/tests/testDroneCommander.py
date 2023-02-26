import numpy as np
import pytest
#https://realpython.com/pytest-python-testing/
from drone_ros.Commander import Commander
from pymavlink import mavutil

@pytest.fixture
def setup_commander():
    conn_string = 'udp:127.0.0.1:14551'
    master = mavutil.mavlink_connection(conn_string)
    commander = Commander(master)
    return commander

def testGoodArmDisarmValid(setup_commander):
    """
    send commands that are 0 or 1 
    return True
    """
    commander = setup_commander
    arm_args = {'arm_disarm': 1}
    val = commander.validateArmDisarm(arm_args)

    assert val == True

    arm_args = {'arm_disarm': 0}
    val = commander.validateArmDisarm(arm_args)
    assert val == True

def testBadArmDisarmValid(setup_commander):
    """
    send commands that are not 0 or 1
    return False
    """
    commander = setup_commander
    arm_args = {'arm_disarm': 2}
    val = commander.validateArmDisarm(arm_args)
    assert val == False

    arm_args = {'arm_disarm': -1}
    val = commander.validateArmDisarm(arm_args)
    assert val == False

def testgoodModeChange(setup_commander):
    """
    send commands that are valid return True
    """
    mode = 'GUIDED'
    commander = setup_commander
    val = commander.validateModeChange(mode)
    assert val == True
    
    mode = 'AUTO'
    val = commander.validateModeChange(mode)
    assert val == True

    mode = 'LOITER'
    val = commander.validateModeChange(mode)
    assert val == True

    mode = 'RTL'
    val = commander.validateModeChange(mode)
    assert val == True

def testBadModeChange(setup_commander):
    """
    send commands that are not valid return False
    """
    
    mode = 'GUIDED1'
    commander = setup_commander
    val = commander.validateModeChange(mode)
    assert val == False

    mode = 'AUT0'
    val = commander.validateModeChange(mode)
    assert val == False

    mode = 'LOIT3R1'
    val = commander.validateModeChange(mode)
    assert val == False

def testGoodTakeoff(setup_commander):
    """
    send commands that are valid return True
    """
    commander = setup_commander
    takeoff_args = {'altitude': 10}
    val = commander.validateTakeoff(takeoff_args)
    assert val == True

    takeoff_args = {'altitude': 30}
    val = commander.validateTakeoff(takeoff_args)
    assert val == True

    takeoff_args = {'altitude': 0}
    val = commander.validateTakeoff(takeoff_args)
    assert val == True

def testBadTakeoff(setup_commander):
    """
    send commands that are not valid return False
    """
    commander = setup_commander
    takeoff_args = {'altitude': -10}
    val = commander.validateTakeoff(takeoff_args)
    assert val == False

def testGoodNEDVelocity(setup_commander): 
    """
    send velocity commands that are valid return True
    """
    commander = setup_commander

    ned_velocity_args = {'vx': 0, 
                         'vy': 0, 
                         'vz': 0,
                         'set_vz': True}
    
    val = commander.validateNEDVelocity(ned_velocity_args)
    assert val == True


    ned_velocity_args = {'vx': 0, 
                         'vy': 0, 
                         'vz': 0,
                         'set_vz': False}
    
    val = commander.validateNEDVelocity(ned_velocity_args)
    assert val == True

def testBadNEDVelocity(setup_commander):
    """
    send velocity commands that are not valid return False
    should also consider constraints on velocity
    """
    commander = setup_commander

    ned_velocity_args = {'velocity_x': 'hi',
                         'velocity_y': 'hi',
                         'velocity_z': 'v'}
    
    val = commander.validateNEDVelocity(ned_velocity_args)
    assert val == False

    ned_velocity_args = {'velocity_x': '100',
                         'velocity_y': '2',
                         'velocity_z': '5'}
    
    val = commander.validateNEDVelocity(ned_velocity_args)
    assert val == False

