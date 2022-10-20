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

# Function used to label each log with a producer tag.
def tag_log(msg, producer_tag):
    return '@%s>> %s' % (producer_tag, msg)
