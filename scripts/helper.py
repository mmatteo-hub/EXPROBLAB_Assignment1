#!/usr/bin/env python

import sys
import roslib
import rospy
import actionlib
import smach
import smach_ros
from EXPROBLAB_Assignment1 import name_mapper as nm
from EXPROBLAB_Assignment1.msg import Point, PlanAction, PlanGoal, ControlAction
from armor_api.armor_client import ArmorClient

from threading import Thread
from threading import Lock

class Helper:
	def __init__(self, done_callback = None, feedback_callback = None, mutex = None):
		if mutex is None:
			self.mutex = Lock()
		else:
			self.mutex = mutex
		self.action_for_change = ''
		# client for the arMOR server
		self.client = ArmorClient("armor_client", "reference")
		self.planner_client = actionlib.SimpleActionClient('motion/planner', PlanAction)
		self.controller_client = actionlib.SimpleActionClient('motion/controller', ControlAction)
	
