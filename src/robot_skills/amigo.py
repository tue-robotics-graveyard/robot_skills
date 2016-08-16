#! /usr/bin/env python

import robot


class Amigo(robot.Robot):
    """docstring for Amigo"""
    def __init__(self, dontInclude=[], wait_services=False):
        import rospy
        s = rospy.Time.now()
        super(Amigo, self).__init__(robot_name="amigo")
        rospy.logwarn("Construction took {} seconds".format((rospy.Time.now() - s).to_sec()))
