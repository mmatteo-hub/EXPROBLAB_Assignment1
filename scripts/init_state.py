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

client.manipulation.add_ind_to_class("C1", "Room")

client.utils.apply_buffered_changes()
client.utils.sync_buffered_reasoner()

client.utils.save_ref_with_inferences(path + "test.owl")
