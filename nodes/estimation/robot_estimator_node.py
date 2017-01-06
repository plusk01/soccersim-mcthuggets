#!/usr/bin/env python
import time

import rospy
from geometry_msgs.msg import Pose2D
from mcthuggets.msg import RobotState

import numpy as np

"""
You could do state estimation here for the robots. For now,
this node's job is simply to take the Pose2D vision message
and patch it through as a RobotState message.

Note that one of these nodes is created per robot and all the
plumbing is done in the `robot.launch` file.
"""

_state_pub = None
_we_are_home = True

def _handle_vision_position(msg):
    # Flip the coordinate system so that our side is always the position
    # side. This is nice because we won't have to think about this later
    # in our high-level AI code and elsewhere.
    # This could also be done in the vision node; see original mcthuggets code
    if not _we_are_home:
        msg.x = -1*msg.x
        msg.y = -1*msg.y

        if msg.theta < 180:
            msg.theta += 180
        else:
            msg.theta -= 180

    new_msg = RobotState()
    new_msg.xhat = new_msg.vision_x = new_msg.xhat_future = msg.x
    new_msg.yhat = new_msg.vision_y = new_msg.yhat_future = msg.y
    new_msg.thetahat = new_msg.vision_theta = new_msg.thetahat_future = msg.theta
    new_msg.correction = True
    _state_pub.publish(new_msg)

def main():
    rospy.init_node('robot_estimator', anonymous=False)

    global _we_are_home
    # probably a better way than doing this...
    ns = rospy.get_namespace().split('/')[1]
    _we_are_home = rospy.get_param("/{}/we_are_home".format(ns), True)

    # Sub/Pub
    global _state_pub
    # Use remap in roslaunch file to create separate channels per robot
    _state_pub = rospy.Publisher('robot_state', RobotState, queue_size=10)

    rospy.Subscriber('vision_position', Pose2D, _handle_vision_position)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
