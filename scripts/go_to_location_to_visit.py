#!/usr/bin/env python

import sys
import roslib
import rospy
import actionlib
import smach
import smach_ros
from actionlib_msgs.msg import GoalStatus
from EXPROBLAB_Assignment1 import name_mapper as nm

class GoToLocationToVisit(smach.State):
	def __init__(self, helper):
		self._helper = helper
		# initialisation function, it should not wait
		smach.State.__init__(self, 
								outcomes=[nm.BATTERY_OK, nm.BATTERY_LOW, nm.LOADED_ONTOLOGY, nm.REASONED, nm.PLANNED_PATH, nm.LOCATION_REACHED],
								input_keys=['go_to_location_to_visit_counter_in'],
								output_keys=['go_to_location_to_visit_counter_out'])
		
	def execute(self, userdata):
		# function called when exiting from the node, it can be blacking
		rospy.loginfo('Executing state ' + nm.GO_TO_LOCATION_TO_VISIT + ' (users = %f)'%userdata.go_to_location_to_visit_counter_in)
		userdata.go_to_location_to_visit_counter_out = userdata.go_to_location_to_visit_counter_in + 1
		
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				if self._helper.action_for_change == nm.BATTERY_LOW:
					self._helper.controller_client.cancel_goal()
					return nm.BATTERY_LOW
				if self._helper.planner_client.get_state() == GoalStatus.SUCCEEDED:
					self._helper.client.manipulation.replace_objectprop_b2_ind('isIn', 'robot', 'C1', 'E')
					pos = self._helper.client.query.objectprop_b2_ind('isIn','robot')
					print("Robot is in " + str(pos))
					self._helper.action_for_change = nm.LOCATION_REACHED
					return nm.LOCATION_REACHED
			finally:
				self._helper.mutex.release()
			rospy.sleep(0.3)
