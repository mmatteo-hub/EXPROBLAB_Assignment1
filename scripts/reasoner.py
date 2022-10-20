#!/usr/bin/env python

import sys
import roslib
import rospy
import actionlib
import smach
import smach_ros
import random
from EXPROBLAB_Assignment1 import name_mapper as nm

class Reasoner(smach.State):
	def __init__(self, helper):
		self._helper = helper
		
		self.client = self._helper.client.query
		
		# initialisation function, it should not wait
		smach.State.__init__(self, 
								outcomes=[nm.BATTERY_OK, nm.BATTERY_LOW, nm.LOADED_ONTOLOGY, nm.REASONED, nm.PLANNED_PATH, nm.LOCATION_REACHED],
								input_keys=['reasoner_counter_in'],
								output_keys=['reasoner_counter_out'])
		
	def execute(self, userdata):
		# function called when exiting from the node, it can be blacking
		rospy.loginfo('Executing state ' + nm.REASONER + ' (users = %f)'%userdata.reasoner_counter_in)
		userdata.reasoner_counter_out = userdata.reasoner_counter_in + 1
		
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				if self._helper.action_for_change == nm.BATTERY_LOW:
					planner_client.cancel_goal()
					return nm.BATTERY_LOW
				self._helper.choice = self._check_accessible_location()
				return nm.REASONED
			finally:
				self._helper.mutex.release()
			rospy.sleep(0.3)

	def _check_accessible_location(self):
		reachable_locations = self.client.objectprop_b2_ind('canReach','robot')
		reachable_locations = self._string_adjust(reachable_locations)
		return self._choose_destination(reachable_locations)
		
	def _string_adjust(self, location):
		for el in range(len(location)):
			index = location[el].index('#')
			location[el] = location[el][index+1:-1]
		return location
		
	def _choose_destination(self, locations):
		return random.choice(locations)
