#!/usr/bin/env python

import sys
import roslib
import rospy
import actionlib
import smach
import smach_ros
from EXPROBLAB_Assignment1 import name_mapper as nm

class Recharge(smach.State):
	def __init__(self, helper):
		self._helper = helper
		# initialisation function, it should not wait
		smach.State.__init__(self, 
								outcomes=[nm.BATTERY_OK, nm.BATTERY_LOW, nm.LOADED_ONTOLOGY, nm.REASONED, nm.PLANNED_PATH, nm.LOCATION_REACHED],
								input_keys=['recharge_counter_in'],
								output_keys=['recharge_counter_out'])
		
	def execute(self, userdata):
		# function called when exiting from the node, it can be blacking
		rospy.loginfo('Executing state ' + nm.RECHARGE + ' (users = %f)'%userdata.recharge_counter_in)
		userdata.recharge_counter_out = userdata.recharge_counter_in + 1
		
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				print("recharge")
			finally:
				self._helper.mutex.release()
			rospy.sleep(0.3)
