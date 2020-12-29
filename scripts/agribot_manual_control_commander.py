#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from agribot_msgs.msg import AgribotManualControl

PUB = rospy.Publisher('cmd_vel_teleop', Twist, queue_size=1)
CHASSIS_LENGTH = 2.1
MIN_RADIUS = 2.5
CUTOFF_RADIUS = 11.0

TWIST_MESSAGE = Twist()

def calculate_angular_velocity(linear_velocity, rotation_radius):
    if rotation_radius != 0.0:
        if rotation_radius >= CUTOFF_RADIUS:
            return 0.0
        elif rotation_radius > MIN_RADIUS:
            return linear_velocity/rotation_radius
        elif rotation_radius > 0:
            return linear_velocity/MIN_RADIUS
        elif rotation_radius <= -CUTOFF_RADIUS:
            return 0
        elif rotation_radius < -MIN_RADIUS:
            return linear_velocity/rotation_radius
        else:
            return -linear_velocity/MIN_RADIUS
    else:
        return 0

def callback(msg):
    linear_velocity = msg.linear_velocity
    rotation_radius = msg.circle_radius
    angular_velocity = calculate_angular_velocity(linear_velocity, rotation_radius)
    TWIST_MESSAGE.linear.x = linear_velocity
    TWIST_MESSAGE.angular.z = angular_velocity
    PUB.publish(TWIST_MESSAGE)

def agribot_manual_control_commander():
    global CHASSIS_LENGTH
    rospy.init_node('agribot_manual_control_commander')
    rospy.Subscriber("cmd_vel_manual", AgribotManualControl, callback)
    CHASSIS_LENGTH = rospy.get_param("~chassis_length", 2.1)
    rospy.spin()

if __name__ == "__main__":
    CHASSIS_LENGTH = rospy.get_param("~chassis_length", 2.1)
    MIN_RADIUS = rospy.get_param("~min_radius", 2.5)
    CUTOFF_RADIUS = rospy.get_param("~cutoff_radius", 11.0)
    try:
        agribot_manual_control_commander()
    except rospy.ROSInterruptException:
        pass
