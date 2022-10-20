#!/usr/bin/env python

import sys
import roslib
import rospy
import actionlib
import smach
import smach_ros
from EXPROBLAB_Assignment1 import name_mapper as nm

class PlanPathToLocation(smach.State):
	def __init__(self, helper):
		self._helper = helper
		# initialisation function, it should not wait
		smach.State.__init__(self, 
								outcomes=[nm.BATTERY_OK, nm.BATTERY_LOW, nm.LOADED_ONTOLOGY, nm.REASONED, nm.PLANNED_PATH, nm.LOCATION_REACHED],
								input_keys=['plan_path_to_location_counter_in'],
								output_keys=['plan_path_to_location_counter_out'])
		
	def execute(self, userdata):
		# function called when exiting from the node, it can be blacking
		rospy.loginfo('Executing state ' + nm.PLAN_LOCATION_TO_VISIT + ' (users = %f)'%userdata.plan_path_to_location_counter_in)
		userdata.plan_path_to_location_counter_out = userdata.plan_path_to_location_counter_in + 1
		
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				print("plan_path")
			finally:
				self._helper.mutex.release()
			rospy.sleep(0.3)
