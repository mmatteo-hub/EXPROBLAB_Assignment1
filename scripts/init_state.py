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
				return nm.LOADED_ONTOLOGY
			finally:
				self._helper.mutex.release()
			rospy.sleep(0.3)
			
	def _ontology_initialization(self):
		path = dirname(realpath(__file__))
		path = path + "/../topology/"

		client = ArmorClient("armor_client", "reference")
		client.utils.load_ref_from_file(path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23",
										True, "PELLET", True, False)
		client.utils.mount_on_ref()
		client.utils.set_log_to_terminal(True)

		client.manipulation.add_ind_to_class('robot', "Robot")

		client.manipulation.add_objectprop_to_ind("hasDoor", 'E', "D5")
		client.manipulation.add_objectprop_to_ind("hasDoor", 'E', "D6")

		client.manipulation.add_objectprop_to_ind("hasDoor", 'C1', "D1")
		client.manipulation.add_objectprop_to_ind("hasDoor", 'C1', "D2")
		client.manipulation.add_objectprop_to_ind("hasDoor", 'C1', "D5")
		client.manipulation.add_objectprop_to_ind("hasDoor", 'C1', "D7")

		client.manipulation.add_objectprop_to_ind("hasDoor", 'C2', "D3")
		client.manipulation.add_objectprop_to_ind("hasDoor", 'C2', "D4")
		client.manipulation.add_objectprop_to_ind("hasDoor", 'C2', "D5")
		client.manipulation.add_objectprop_to_ind("hasDoor", 'C2', "D6")

		client.manipulation.add_objectprop_to_ind("hasDoor", 'R1', "D1")
		client.manipulation.add_objectprop_to_ind("hasDoor", 'R2', "D2")
		client.manipulation.add_objectprop_to_ind("hasDoor", 'R3', "D3")
		client.manipulation.add_objectprop_to_ind("hasDoor", 'R4', "D4")

		client.manipulation.add_objectprop_to_ind("isIn", 'robot', "E")

		client.call('DISJOINT', 'IND', '', ['E','C1','C2','R1','R2','R3','R4','D1','D2','D3','D4','D5','D6','D7'])

		client.manipulation.add_dataprop_to_ind('visitedAt', 'R1', 'Long', '123')
		client.manipulation.add_dataprop_to_ind('visitedAt', 'R2', 'Long', '123')
		client.manipulation.add_dataprop_to_ind('visitedAt', 'R3', 'Long', '123')
		client.manipulation.add_dataprop_to_ind('visitedAt', 'R4', 'Long', '123')
		client.manipulation.add_dataprop_to_ind('visitedAt', 'C1', 'Long', '123')
		client.manipulation.add_dataprop_to_ind('visitedAt', 'C2', 'Long', '123')
		client.manipulation.add_dataprop_to_ind('visitedAt', 'E', 'Long', '123')

		client.utils.apply_buffered_changes()
		client.utils.sync_buffered_reasoner()

		client.utils.save_ref_with_inferences(path + "test_ontology_3.owl")
