#! /usr/bin/env python

# ROS
import rospy

# Body parts
import base
import torso
import arms
import head

# Human Robot Interaction
import speech
from hmi_server.api import Api
import ears
import ebutton
import lights

# tf
import tf_server

# Reasoning/world modeling
import world_model_ed
import reasoner

# Misc: do we need this???
import geometry_msgs
from collections import OrderedDict

from .util.ros_connections import wait_for_connections


class Robot(object):
    """
    Interface to all parts of the robot.
    """
    def __init__(self, robot_name=""):

        self.robot_name = robot_name
        self.tf_listener = tf_server.TFClient()

        # Body parts
        self.base = base.Base(self.robot_name, self.tf_listener)
        self.torso = torso.Torso(self.robot_name, self.tf_listener)
        self.spindle = self.torso
        self.leftArm = arms.Arm(self.robot_name, "left", self.tf_listener)
        self.rightArm = arms.Arm(self.robot_name, "right", self.tf_listener)
        self.arms = OrderedDict(left=self.leftArm, right=self.rightArm)
        self.head = head.Head(self.robot_name, self.tf_listener)

        # Human Robot Interaction
        self.lights = lights.Lights(self.robot_name)
        self.speech = speech.Speech(self.robot_name, self.tf_listener,
                                    lambda: self.lights.set_color(1, 0, 0),
                                    lambda: self.lights.set_color(0, 0, 1))
        self.hmi = Api("/" + self.robot_name + '/hmi')
        self.ears = ears.Ears(self.robot_name, self.tf_listener,
                              lambda: self.lights.set_color(0, 1, 0),
                              lambda: self.lights.set_color(0, 0, 1))
        self.ears._hmi = self.hmi  # TODO: when ears is gone, remove this line
        self.ebutton = ebutton.EButton()

        # Reasoning/world modeling
        self.ed = world_model_ed.ED(self.robot_name, self.tf_listener)
        self.reasoner = reasoner.Reasoner(self.robot_name)

        # Miscellaneous
        self.pub_target = rospy.Publisher("/target_location", geometry_msgs.msg.Pose2D, queue_size=10)
        self.base_link_frame = "/"+self.robot_name+"/base_link"

        # Grasp offsets
        # TODO: Don't hardcode, load from parameter server to make robot independent.
        self.grasp_offset = geometry_msgs.msg.Point(0.5, 0.2, 0.0)

        # Wait for connections: make sure all action clients and services are connected
        wait_for_connections(1.0)

    def publish_target(self, x, y):
        self.pub_target.publish(geometry_msgs.msg.Pose2D(x, y, 0))

    def tf_transform_pose(self, ps, frame):
        self.tf_listener.waitForTransform(frame, ps.header.frame_id, rospy.Time(), rospy.Duration(2.0))
        output_pose = self.tf_listener.transformPose(frame, ps)
        return output_pose

    def get_arm(self, side):
        """Get an arm object and a backup for that arm by giving a side as either a string or an Arm-object
        @param side Either string from robot.arms.keys() or Arm from robot.arms.values()
        >>> robot = Robot("dummy")
        >>> arm, backup_arm = robot.get_arm("left")
        >>> assert(arm == robot.leftArm)
        >>> assert(backup_arm == robot.rightArm)"""
        preferred_side = self.arms[self.arms.keys()[0]]

        # Define which arm is which's backup arm (left backs up for right etc)
        backup_arms = self.arms.values()  # Get a *list* of arms i.e. the values of the arm-dict, not the keys
        backup_arms.insert(0, backup_arms.pop())  # Make the last arm the first in the list, so we shift by 1
        backup_str_dict = dict(zip(self.arms.keys(), backup_arms))  # Create a dict again that maps strings to
        # backup-arms
        backup_obj_dict = {self.arms[side]: backup_str_dict[side] for side in self.arms.keys()}  # Create a dict that
        # maps e.g. self.LeftArm to self.rightArm

        if isinstance(side, basestring):
            try:
                preferred_side = self.arms[side]
            except KeyError:
                print "Unknown arm side:" + str(side) + ". Defaulting to 'right'"
                preferred_side = self.arms[self.arms.keys()[0]]
        elif isinstance(side, arms.Arm):
            preferred_side = side
        else:
            print "Unknown arm side:" + str(side) + ". Defaulting to '{0}'".format(preferred_side.side)

        backup_side = backup_obj_dict[preferred_side]
        return preferred_side, backup_side

    def close(self):

        for part in [self.head, self.base, self.torso, self.speech, self.leftArm, self.rightArm,
                     self.ears, self.ebutton, self.lights, self.reasoner]:
            try:
                part.close()
            except:
                pass

    def __enter__(self):
        pass

    def __exit__(self, exception_type, exception_val, trace):
        if any((exception_type, exception_val, trace)):
            rospy.logerr("Robot exited with {0},{1},{2}".format(exception_type, exception_val, trace))
        self.close()

if __name__ == "__main__":
    rospy.init_node("robot")

    import doctest
    doctest.testmod()
