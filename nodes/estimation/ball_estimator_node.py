#!/usr/bin/env python
import time

import rospy
from geometry_msgs.msg import Pose2D
from mcthuggets.msg import BallState

import numpy as np

"""
You could do state estimation here for the ball. For now,
this node's job is simply to take the Pose2D vision message
and patch it through as a BallState message.
"""

_state_pub = None

def _handle_vision_ball_position(msg):
    # Construct ball_state message, BallState
    new_msg = BallState()
    new_msg.vision_x = new_msg.xhat = new_msg.xhat_future = msg.x
    new_msg.vision_y = new_msg.yhat = new_msg.yhat_future = msg.y
    new_msg.vx = 0
    new_msg.vy = 0
    new_msg.predict_forward_seconds = 0
    new_msg.correction = True # actual measurement
    _state_pub.publish(new_msg)

def main():
    rospy.init_node('ball_estimator', anonymous=False)

    # Sub/Pub
    global _state_pub
    _state_pub = rospy.Publisher('ball_state', BallState, queue_size=10)
    
    rospy.Subscriber('vision_position', Pose2D, _handle_vision_ball_position)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
