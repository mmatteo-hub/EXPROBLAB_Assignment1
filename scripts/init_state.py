#!/usr/bin/env python

import sys
import roslib
import rospy
import actionlib
import smach
import smach_ros
from os.path import dirname, realpath
from EXPROBLAB_Assignment1 import name_mapper as nm

from armor_api.armor_client import ArmorClient

class InitState(smach.State):
	def __init__(self, helper):
		self._helper = helper
		# initialisation function, it should not wait
		smach.State.__init__(self, 
								outcomes=[nm.BATTERY_OK, nm.BATTERY_LOW, nm.LOADED_ONTOLOGY, nm.REASONED, nm.PLANNED_PATH, nm.LOCATION_REACHED],
								input_keys=['init_state_counter_in'],
								output_keys=['init_state_counter_out'])
								
	def execute(self, userdata):
		# function called when exiting from the node, it can be blacking
		rospy.loginfo('Executing state ' + nm.INIT_STATE + ' (users = %f)'%userdata.init_state_counter_in)
		userdata.init_state_counter_out = userdata.init_state_counter_in + 1
		
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				self._ontology_initialization()
				self._helper.action_for_change = nm.LOADED_ONTOLOGY
				return nm.LOADED_ONTOLOGY
			finally:
				self._helper.mutex.release()
			rospy.sleep(0.3)
			
	def _ontology_initialization(self):
		path = dirname(realpath(__file__))
		path = path + "/../topology/"

		self._helper.client.utils.load_ref_from_file(path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23",
										True, "PELLET", True, False)
		self._helper.client.utils.mount_on_ref()
		self._helper.client.utils.set_log_to_terminal(True)

		self._helper.client.manipulation.add_ind_to_class('robot', "Robot")

		self._helper.client.manipulation.add_objectprop_to_ind("hasDoor", 'E', "D5")
		self._helper.client.manipulation.add_objectprop_to_ind("hasDoor", 'E', "D6")

		self._helper.client.manipulation.add_objectprop_to_ind("hasDoor", 'C1', "D1")
		self._helper.client.manipulation.add_objectprop_to_ind("hasDoor", 'C1', "D2")
		self._helper.client.manipulation.add_objectprop_to_ind("hasDoor", 'C1', "D5")
		self._helper.client.manipulation.add_objectprop_to_ind("hasDoor", 'C1', "D7")

		self._helper.client.manipulation.add_objectprop_to_ind("hasDoor", 'C2', "D3")
		self._helper.client.manipulation.add_objectprop_to_ind("hasDoor", 'C2', "D4")
		self._helper.client.manipulation.add_objectprop_to_ind("hasDoor", 'C2', "D5")
		self._helper.client.manipulation.add_objectprop_to_ind("hasDoor", 'C2', "D6")

		self._helper.client.manipulation.add_objectprop_to_ind("hasDoor", 'R1', "D1")
		self._helper.client.manipulation.add_objectprop_to_ind("hasDoor", 'R2', "D2")
		self._helper.client.manipulation.add_objectprop_to_ind("hasDoor", 'R3', "D3")
		self._helper.client.manipulation.add_objectprop_to_ind("hasDoor", 'R4', "D4")

		self._helper.client.manipulation.add_objectprop_to_ind("isIn", 'robot', "E")

		self._helper.client.call('DISJOINT', 'IND', '', ['E','C1','C2','R1','R2','R3','R4','D1','D2','D3','D4','D5','D6','D7'])

		self._helper.client.manipulation.add_dataprop_to_ind('visitedAt', 'R1', 'Long', '123')
		self._helper.client.manipulation.add_dataprop_to_ind('visitedAt', 'R2', 'Long', '123')
		self._helper.client.manipulation.add_dataprop_to_ind('visitedAt', 'R3', 'Long', '123')
		self._helper.client.manipulation.add_dataprop_to_ind('visitedAt', 'R4', 'Long', '123')
		self._helper.client.manipulation.add_dataprop_to_ind('visitedAt', 'C1', 'Long', '123')
		self._helper.client.manipulation.add_dataprop_to_ind('visitedAt', 'C2', 'Long', '123')
		self._helper.client.manipulation.add_dataprop_to_ind('visitedAt', 'E', 'Long', '123')

		self._helper.client.utils.apply_buffered_changes()
		self._helper.client.utils.sync_buffered_reasoner()

		self._helper.client.utils.save_ref_with_inferences(path + "test_ontology_3.owl")
