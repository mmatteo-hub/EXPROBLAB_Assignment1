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
					self._helper.planner_client.cancel_goal()
					return nm.BATTERY_LOW
				if self._helper.action_for_change == nm.LOADED_ONTOLOGY or self._helper.action_for_change == nm.LOCATION_REACHED:
					self._reason_changes()
					self._helper.choice = self._check_accessible_location()
					self._helper.old_loc = self._helper._string_adjust(self._helper.client.query.objectprop_b2_ind('isIn','Robot1'))
					self._helper.old_loc = self._helper.old_loc[0]
					return nm.REASONED
			finally:
				self._helper.mutex.release()
			rospy.sleep(0.3)
	
	def _reason_changes(self):
		self._helper.client.utils.apply_buffered_changes()
		self._helper.client.utils.sync_buffered_reasoner()

	def _check_accessible_location(self):
		reachable_locations = self._helper.client.query.objectprop_b2_ind('canReach','Robot1')
		reachable_locations = self._helper._string_adjust(reachable_locations)
		return self._choose_destination(reachable_locations)
		
	def _choose_destination(self, locations):
		return random.choice(locations)
