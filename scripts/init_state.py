#!/usr/bin/env python

import sys
import roslib
import rospy
import actionlib
from os.path import dirname, realpath

from armor_api.armor_client import ArmorClient

path = dirname(realpath(__file__))
path = path + "/../topology/"

client = ArmorClient("armor_client", "reference")
client.utils.load_ref_from_file(path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23",
								True, "PELLET", True, False)
client.utils.mount_on_ref()
client.utils.set_log_to_terminal(True)

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
#client.manipulation.add_objectprop_to_ind("isIn", 'Robot1', "E")

client.call('DISJOINT', 'IND', '', ['E', 'C1', 'C2', 'R1', 'R2', 'D1', 'D2', 'D3', 'D4', 'D5', 'D6', 'D7'])

client.manipulation.add_dataprop_to_ind('visitedAt', 'R1', 'Long', '123')
client.manipulation.add_dataprop_to_ind('visitedAt', 'R2', 'Long', '123')
client.manipulation.add_dataprop_to_ind('visitedAt', 'R3', 'Long', '123')
client.manipulation.add_dataprop_to_ind('visitedAt', 'R4', 'Long', '123')
client.manipulation.add_dataprop_to_ind('visitedAt', 'C1', 'Long', '123')
client.manipulation.add_dataprop_to_ind('visitedAt', 'C2', 'Long', '123')
client.manipulation.add_dataprop_to_ind('visitedAt', 'E', 'Long', '123')


client.utils.apply_buffered_changes()
client.utils.sync_buffered_reasoner()
client.utils.save(path + "test_topology.owl")


