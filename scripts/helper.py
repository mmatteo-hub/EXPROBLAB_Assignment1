#!/usr/bin/env python

import sys
import roslib
import rospy
import actionlib
import smach
import smach_ros
from EXPROBLAB_Assignment1 import name_mapper as nm

from threading import Thread
from threading import Lock

class Helper:
	def __init__(self, done_callback = None, feedback_callback = None, mutex = None):
		if mutex is None:
			self.mutex = Lock()
		else:
			self.mutex = mutex
