#!/usr/bin/env python
import rospy

##########
# definition of the state name
INIT_STATE = 'INIT_STATE'
RECHARGE = 'RECHARGE'
REASONER = 'REASONER'
PLAN_PATH_TO_LOCATION = 'PLAN_PATH_TO_LOCATION'
GO_TO_LOCATION_TO_VISIT = 'GO_TO_LOCATION_TO_VISIT'

##########
# definition of the action to change states

BATTERY_OK = 'BATTERY_OK'
BATTERY_LOW = 'BATTERY_LOW'
LOADED_ONTOLOGY = 'LOADED_ONTOLOGY'
REASONED = 'REASONED'
PLANNED_PATH = 'PLANNED_PATH'
LOCATION_REACHED = 'LOCATION_REACHED'

##########
ACTION_PLANNER = 'motion/planner'
ACTION_CONTROLLER = 'motion/controller'
NODE_PLANNER = 'planner'
NODE_CONTROLLER = 'controller'
NODE_ROBOT_STATE = 'robot-state'

SERVER_GET_POSE = 'state/get_pose'
SERVER_SET_POSE = 'state/set_pose'

##########
BATTERY_THRESHOLD = 60
NUMBER_OF_POINTS_PATH = 10
RECHARGING_ROOM = 'E'

##########
# rooms' coordinate
ROOMS = ['E', 'C1', 'C2', 'R1', 'R2', 'R3', 'R4']
COORDINATES = [[0.0,0.0], [2.0,0.0], [3.0,0.0], [1.0,0.0], [1.0,1.0], [4.0,0.0], [4.0,1.0]]

# Function used to label each log with a producer tag.
def tag_log(msg, producer_tag):
    return '@%s>> %s' % (producer_tag, msg)
