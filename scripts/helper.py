#!/usr/bin/env python

import sys
import roslib
import rospy
import actionlib
import smach
import smach_ros
import random
import time
import re
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
		self.robot_timestamp_value = -1
		
		# client for the arMOR server
		self.client = ArmorClient('armor_client', "reference")
			
		# client for the planner server
		self.planner_client = actionlib.SimpleActionClient('motion/planner', PlanAction)
		
		# client for the controller server
		self.controller_client = actionlib.SimpleActionClient('motion/controller', ControlAction)
		
	def reason_changes(self):
		self.client.utils.apply_buffered_changes()
		self.client.utils.sync_buffered_reasoner()
		
	def check_battery(self):
		if (int(time.time()) - self.battery_timestamp) > nm.BATTERY_THRESHOLD:
			self.action_for_change = nm.BATTERY_LOW
	
	def string_adjust(self, location):
		for el in range(len(location)):
			index = location[el].index('#')
			location[el] = location[el][index+1:-1]
		return location
		
	def format(self, oldlist, start, end):
		# Function to format a list of strings.
		# It cuts the elements of oldlist between the strings start and end, returning the formatted list.
		newlist = []
		for string in oldlist:
			newlist.append(re.search(start + '(.+?)' + end, string).group(1))
		return newlist

	def plan_location(self, location):
		goal = PlanGoal()
		goal.start = self._robot_pos()
		goal.target = self._target_coordiantes(location)
		return goal

	def _robot_pos(self):
		_pos = self.format(self.client.query.objectprop_b2_ind('isIn','Robot1'), '#', '>')[0]
		for i in range(len(nm.ROOMS)):
			if _pos == nm.ROOMS[i]:
				return Point(x = nm.COORDINATES[i][0], y = nm.COORDINATES[i][1])
				
	def _target_coordiantes(self, _location):
		for i in range(len(nm.ROOMS)):
			if _location == nm.ROOMS[i]:
				return Point(x = nm.COORDINATES[i][0], y = nm.COORDINATES[i][1])

	def _robot_timestamp_value(self):
		timestamp = self.client.query.dataprop_b2_ind('now', 'Robot1')
		return str(self.format(timestamp, '"', '"')[0])
				
	def _location_old_timestamp(self, location):
		timestamp = self.client.query.dataprop_b2_ind('visitedAt', location)
		return str(self.format(timestamp, '"', '"')[0])

	def update_timestamp(self):
		self.client.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', str(int(time.time())), self._robot_timestamp_value())
		self.client.manipulation.replace_dataprop_b2_ind('visitedAt', str(self.choice), 'Long', self._robot_timestamp_value(), self._location_old_timestamp(str(self.choice)))
					
