#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import random
import actionlib
from EXPROBLAB_Assignment1 import name_mapper as nm
from helper import Helper
from init_state import InitState
from reasoner import Reasoner
from recharge import Recharge
from plan_path_to_location import PlanPathToLocation
from go_to_location_to_visit import GoToLocationToVisit

##########

def main():
	rospy.init_node('smach_finite_state_machine')
	
	# Define an helper
	_helper = Helper()

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=[nm.BATTERY_OK, nm.BATTERY_LOW, nm.LOADED_ONTOLOGY, nm.REASONED, nm.PLANNED_PATH, nm.LOCATION_REACHED])
	sm.userdata.sm_counter = 0

	# Open the container
	with sm:
		# Add states to the container
		smach.StateMachine.add(nm.INIT_STATE, InitState(_helper),
								transitions={nm.BATTERY_LOW: nm.INIT_STATE,
											 nm.BATTERY_OK: nm.INIT_STATE,
											 nm.LOADED_ONTOLOGY: nm.REASONER,
											 nm.REASONED: nm.INIT_STATE,
											 nm.PLANNED_PATH: nm.INIT_STATE,
											 nm.LOCATION_REACHED: nm.INIT_STATE},
								remapping={'init_state_counter_in':'sm_counter',
										   'init_state_counter_out':'sm_counter'})
		
		smach.StateMachine.add(nm.RECHARGE, Recharge(_helper),
								transitions={nm.BATTERY_LOW: nm.RECHARGE,
											 nm.BATTERY_OK: nm.REASONER,
											 nm.LOADED_ONTOLOGY: nm.RECHARGE,
											 nm.REASONED: nm.RECHARGE,
											 nm.PLANNED_PATH: nm.RECHARGE,
											 nm.LOCATION_REACHED: nm.RECHARGE},
								remapping={'recharge_counter_in':'sm_counter',
										   'recharge_counter_out':'sm_counter'})
										   
		smach.StateMachine.add(nm.REASONER, Reasoner(_helper),
								transitions={nm.BATTERY_LOW: nm.RECHARGE,
											 nm.BATTERY_OK: nm.REASONER,
											 nm.LOADED_ONTOLOGY: nm.REASONER,
											 nm.REASONED: nm.PLAN_PATH_TO_LOCATION,
											 nm.PLANNED_PATH: nm.REASONER,
											 nm.LOCATION_REACHED: nm.REASONER},
								remapping={'reasoner_counter_in':'sm_counter',
										   'reasoner_counter_out':'sm_counter'})
		
		smach.StateMachine.add(nm.PLAN_PATH_TO_LOCATION, PlanPathToLocation(_helper),
								transitions={nm.BATTERY_LOW: nm.RECHARGE,
											 nm.BATTERY_OK: nm.PLAN_PATH_TO_LOCATION,
											 nm.LOADED_ONTOLOGY: nm.PLAN_PATH_TO_LOCATION,
											 nm.REASONED: nm.PLAN_PATH_TO_LOCATION,
											 nm.PLANNED_PATH: nm.GO_TO_LOCATION_TO_VISIT,
											 nm.LOCATION_REACHED: nm.PLAN_PATH_TO_LOCATION},
								remapping={'plan_path_to_location_counter_in':'sm_counter',
										   'plan_path_to_location_counter_out':'sm_counter'})
										
		smach.StateMachine.add(nm.GO_TO_LOCATION_TO_VISIT, GoToLocationToVisit(_helper),
								transitions={nm.BATTERY_LOW: nm.RECHARGE,
											 nm.BATTERY_OK: nm.GO_TO_LOCATION_TO_VISIT,
											 nm.LOADED_ONTOLOGY: nm.GO_TO_LOCATION_TO_VISIT,
											 nm.REASONED: nm.GO_TO_LOCATION_TO_VISIT,
											 nm.PLANNED_PATH: nm.GO_TO_LOCATION_TO_VISIT,
											 nm.LOCATION_REACHED: nm.REASONER},
								remapping={'go_to_location_to_visit_counter_in':'sm_counter',
										   'go_to_location_to_visit_counter_out':'sm_counter'})
												
	# Definition of the subscribers for the FSM
	

	# Create and start the introspection server for visualization
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	# Execute the state machine
	outcome = sm.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()

##########

if __name__ == '__main__':
	main()


