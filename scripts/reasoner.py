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
		self._helper.reason_changes()
		self._helper.check_battery()
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				if self._helper.action_for_change == nm.BATTERY_LOW:
					self._helper.choice = self._check_recharge_location_available()
					if self._helper.choice != [] or self._helper.format(self._helper.client.query.objectprop_b2_ind('isIn','Robot1'), '#', '>')[0] == nm.RECHARGING_ROOM:
						return nm.BATTERY_LOW
					else: self._helper.action_for_change = nm.RECHARGING_CHECK
						
				if self._helper.action_for_change == nm.LOADED_ONTOLOGY or self._helper.action_for_change == nm.LOCATION_REACHED or self._helper.action_for_change == nm.BATTERY_OK or self._helper.action_for_change == nm.RECHARGING_CHECK:
					self._helper.choice = self._check_accessible_location()
					self._helper.old_loc = self._helper.format(self._helper.client.query.objectprop_b2_ind('isIn','Robot1'), '#', '>')[0]
					return nm.REASONED
			finally:
				self._helper.mutex.release()
			rospy.sleep(0.3)

	def _check_accessible_location(self):
		_reachable_locations = self._helper.format(self._helper.client.query.objectprop_b2_ind('canReach','Robot1'), '#', '>')
		_reachable_corridors = self._check_for_corridors(_reachable_locations)
		_reachable_urgent = self._check_for_urgent_locations(_reachable_locations)
		
		if _reachable_urgent != []:
			return self._choose_destination(_reachable_urgent)
		elif _reachable_corridors != []:
			return self._choose_destination(_reachable_corridors)
		elif _reachable_locations != []:
			return self._choose_destination(_reachable_locations)
		else:
			log_msg = f'No locations reachable from {str(self._helper.choice)} '
			rospy.loginfo(nm.tag_log(log_msg, nm.REASONER))
			
	def _check_recharge_location_available(self):
		_reachable_E_room = self._check_for_E_room(self._helper.format(self._helper.client.query.objectprop_b2_ind('canReach','Robot1'), '#', '>'))
		if _reachable_E_room == nm.RECHARGING_ROOM:
			return nm.RECHARGING_ROOM
		else: return []
			
	def _check_for_corridors(self, _reachable_locations):
		_corridors = self._helper.format(self._helper.client.query.ind_b2_class('CORRIDOR'), '#', '>')
		_reachable_corridors = []
		for i in range(len(_corridors)):
			if(_corridors[i] in _reachable_locations):
				_reachable_corridors.append(_corridors[i])
				
		return _reachable_corridors
		
	def _check_for_urgent_locations(self, _reachable_locations):
		_urgent_locations = self._helper.format(self._helper.client.query.ind_b2_class('URGENT'), '#', '>')
		_reachable_urgent = []
		for i in range(len(_urgent_locations)):
			if(_urgent_locations[i] in _reachable_locations):
				_reachable_urgent.append(_urgent_locations[i])
				
		return _reachable_urgent
		
	def _check_for_E_room(self, _reachable_locations):
		for i in range(len(_reachable_locations)):
			if _reachable_locations[i] == nm.RECHARGING_ROOM:
				return nm.RECHARGING_ROOM
		
			
	def _choose_destination(self, locations):
		return random.choice(locations)
