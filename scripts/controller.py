#! /usr/bin/env python

import random
import rospy
# Import constant name defined to structure the architecture.
from EXPROBLAB_Assignment1 import name_mapper as nm
# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from EXPROBLAB_Assignment1.msg import ControlFeedback, ControlResult
from EXPROBLAB_Assignment1.srv import SetPose
import EXPROBLAB_Assignment1  # This is required to pass the ControlAction` type for instantiating the `SimpleActionServer`.
from helper import Helper
from armor_api.armor_client import ArmorClient

# A tag for identifying logs producer.
LOG_TAG = nm.NODE_CONTROLLER


# An action server to simulate motion controlling.
# Given a plan as a set of via points, it simulate the movements
# to reach each point with a random delay. This server updates
# the current robot position stored in the `robot-state` node.
class ControllingAction(object):

    def __init__(self):
        # Get random-based parameters used by this server
        #self._random_motion_time = rospy.get_param(anm.PARAM_CONTROLLER_TIME, [0.1, 2.0])
        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(nm.ACTION_CONTROLLER,
                                      EXPROBLAB_Assignment1.msg.ControlAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self._as.start()
        # Log information.
        #log_msg = (f'`{nm.ACTION_CONTROLLER}` Action Server initialised. It will navigate trough the plan with a delay ' 
         #          f'between each via point spanning in [{self._random_motion_time[0]}, {self._random_motion_time[1]}).')
        #rospy.loginfo(nm.tag_log(log_msg, LOG_TAG))

    # The callback invoked when a client set a goal to the `controller` server.
    # This function requires a list of via points (i.e., the plan), and it simulate
    # a movement through each point with a delay spanning in 
    # ['self._random_motion_time[0]`, `self._random_motion_time[1]`).
    # As soon as each via point is reached, the related robot position is updated
    # in the `robot-state` node.
    def execute_callback(self, goal):
        # Check if the provided plan is processable. If not, this service will be aborted.
        if goal is None or goal.via_points is None or len(goal.via_points) == 0:
            rospy.logerr(nm.tag_log('No via points provided! This service will be aborted!', LOG_TAG))
            self._as.set_aborted()
            return

        # Construct the feedback and loop for each via point.
        feedback = ControlFeedback()
        rospy.loginfo(nm.tag_log('Server is controlling...', LOG_TAG))
        for point in goal.via_points:
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                rospy.loginfo(nm.tag_log('Service has been cancelled by the client!', LOG_TAG))
                # Actually cancel this service.
                self._as.set_preempted()
                return
                # Wait before to reach the following via point. This is just for testing purposes.
            #delay = random.uniform(self._random_motion_time[0], self._random_motion_time[1])
            delay = 0.25
            rospy.sleep(delay)
            # Publish a feedback to the client to simulate that a via point has been reached. 
            feedback.reached_point = point
            self._as.publish_feedback(feedback)
            # Set the new current position into the `robot-state` node.
            #_set_pose_client(point)
            # Log current robot position.
            log_msg = f'Reaching point ({point.x}, {point.y}).'
            rospy.loginfo(nm.tag_log(log_msg, LOG_TAG))

        # Publish the results to the client.
        result = ControlResult()
        result.reached_point = feedback.reached_point
        rospy.loginfo(nm.tag_log('Motion control successes.', LOG_TAG))
        self._as.set_succeeded(result)
        return  # Succeeded.

if __name__ == '__main__':
    # Initialise the node, its action server, and wait.   
    rospy.init_node(nm.NODE_CONTROLLER, log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()
