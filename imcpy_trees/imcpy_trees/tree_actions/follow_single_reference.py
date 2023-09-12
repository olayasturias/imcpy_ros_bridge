#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://github.com/splintered-reality/py_trees_ros_tutorials/raw/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Mocks a simple action server that rotates the robot 360 degrees.
"""


##############################################################################
# Imports
##############################################################################

import argparse
import math
import threading
from .actions import GenericServer
import py_trees_ros_interfaces.action as py_trees_actions
from imc_ros_msgs.action import FollowSingleReference
from .imcpy_single_ref import FollowSingleRef
import rclpy
import sys

##############################################################################
# Class
##############################################################################


class FollowSingleReferenceServer(GenericServer):
    """
    Simple server that controls a full rotation of the robot.

    Node Name:
        * **rotation_controller**

    Action Servers:
        * **/rotate** (:class:`py_trees_ros_interfaces.action.Dock`)

          * motion primitives - rotation server

    Args:
        rotation_rate (:obj:`float`): rate of rotation (rad/s)
    """
    def __init__(self, rotation_rate: float=1.57):
        super().__init__(node_name="follow_reference_server",
                         action_name="FollowSingleReference",
                         action_type=FollowSingleReference,
                         generate_feedback_message=self.generate_feedback_message,
                         duration=2.0 * math.pi / rotation_rate
                         )

    def generate_feedback_message(self):
        """
        Create a feedback message that populates the percent completed.

        Returns:
            :class:`py_trees_actions.Rotate.Feedback`: the populated feedback message
        """
        # TODO: send some feedback message
        feedback_msg = FollowSingleReference.Feedback()
        feedback_msg.state = 0
        # dune_actor = FollowSingleRef(target_name='lauv-simulator-1')
        # def initialize_imcpy_task():
        #     dune_actor = FollowSingleRef(target_name='lauv-simulator-1')
        #     dune_actor.run_async_function()

        # # with self.goal_lock:
        # thread = threading.Thread(target = initialize_imcpy_task)
        # thread.start()
        # thread.join()
        # msg.percentage_completed = self.percent_completed
        # msg.angle_rotated = 2*math.pi*self.percent_completed/100.0
        return feedback_msg


def main():
    """
    Entry point for the mock rotation controller node.
    """
    parser = argparse.ArgumentParser(description='Follow a single reference point')
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser.parse_args(command_line_args)
    rclpy.init()  # picks up sys.argv automagically internally
    follow = FollowSingleReferenceServer()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(follow.node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        follow.abort()
        # caveat: often broken, with multiple spin_once or shutdown, error is the
        # mysterious:
        #   The following exception was never retrieved: PyCapsule_GetPointer
        #   called with invalid PyCapsule object
        executor.shutdown()  # finishes all remaining work and exits
        rclpy.try_shutdown()
