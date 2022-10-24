#!/usr/bin/env python

import sys
import os
import roslib
import rospy
import actionlib
import smach
import smach_ros
import time
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
				
		# send the plan goal and control goal to the respective action server to move the robot to the recharging location
		#self._plan_to_recharge()
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			self._pos = self._check_rob_pos()
			try:
				if self._helper.action_for_change == nm.BATTERY_OK:
					return nm.BATTERY_OK
				if self._pos == nm.RECHARGING_ROOM:
					self._recharging_method()
					return nm.BATTERY_OK
				if self._pos != nm.RECHARGING_ROOM:
					self._plan_and_go_to_recharge()
			finally:
				self._helper.mutex.release()
			rospy.sleep(0.3)
			
	def _check_rob_pos(self):
		return self._helper.format(self._helper.client.query.objectprop_b2_ind('isIn','Robot1'), '#', '>')[0]

	def _plan_and_go_to_recharge(self):
		_goal = self._helper.plan_location(nm.RECHARGING_ROOM)
		self._helper.planner_client.send_goal(_goal)
		log_msg = f'Planning the robot to Recharge '
		rospy.loginfo(nm.tag_log(log_msg, nm.RECHARGE))
		
		self._helper.planner_client.wait_for_result()
		self._helper.controller_client.send_goal(self._helper.planner_client.get_result())
		
		log_msg = f'Moving the robot to Recharge '
		rospy.loginfo(nm.tag_log(log_msg, nm.RECHARGE))
		
		self._helper.controller_client.wait_for_result()
		
		self._helper.choice = nm.RECHARGING_ROOM
		self._helper.old_loc = self._pos
		
		self._helper.client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', str(self._helper.choice), str(self._helper.old_loc))
		
		self._helper.update_timestamp()
		self._helper.reason_changes()
		
		log_msg = f'Robot in recharging room '
		rospy.loginfo(nm.tag_log(log_msg, nm.RECHARGE))

	def _recharging_method(self):
		log_msg = f'Robot is recharging'
		rospy.loginfo(nm.tag_log(log_msg, nm.RECHARGE))
		
		_battery = []
		for i in range(100):
			if i % 10 == 0:
				_battery.append('=')
			print("[" + ' '.join(_battery) + "] | Battery: " + str(i) + "%", end="\r")
			rospy.sleep(0.05)
			
		self._helper.action_for_change = nm.BATTERY_OK
		self._helper.battery_timestamp = int(time.time())
		
