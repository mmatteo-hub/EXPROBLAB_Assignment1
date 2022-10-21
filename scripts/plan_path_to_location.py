#!/usr/bin/env python

import sys
import roslib
import rospy
import actionlib
import smach
import smach_ros
import random
from EXPROBLAB_Assignment1.msg import Point, PlanAction, PlanGoal, ControlAction
from actionlib_msgs.msg import GoalStatus
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
		rospy.loginfo('Executing state ' + nm.PLAN_PATH_TO_LOCATION + ' (users = %f)'%userdata.plan_path_to_location_counter_in)
		userdata.plan_path_to_location_counter_out = userdata.plan_path_to_location_counter_in + 1
		self._plan_location()
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				if self._helper.action_for_change == nm.BATTERY_LOW: # higher priority
					self._help.planner_client.cancel_goal()
					return nm.BATTERY_LOW
				if self._helper.planner_client.get_state() == GoalStatus.SUCCEEDED:
					return nm.PLANNED_PATH
			finally:
				self._helper.mutex.release()
			rospy.sleep(0.3)
	
	def _plan_location(self):
		goal = PlanGoal()
		goal.start = self._start_position()
		goal.target = Point(x = random.uniform(0, 10), y = random.uniform(0, 10))
		self._helper.planner_client.send_goal(goal)
	
	def _start_position(self):
		pos = self._helper.client.query.objectprop_b2_ind('isIn','Robot1')
		#print("NOW ROBOT IS IN " + str(pos))
		return Point(x = 1, y = 1)
