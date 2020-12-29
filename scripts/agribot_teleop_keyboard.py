#!/usr/bin/env python
from __future__ import print_function
import sys
import select
import termios
import tty
import rospy
from agribot_msgs.msg import AgribotManualControl


MSG = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For bigger changes (x10), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

q/z : increase/decrease max steps of linear speed by 0.01 and radius by 0.025
w/x : increase/decrease max step of linear speed by 0.01
e/c : increase/decrease max step of radius by 0.025

CTRL-C to quit
"""

MOVE_BINDINGS = {
    'i':(1, 0, 0, 0),
    'o':(1, 0, 0, -1),
    'j':(0, 0, 0, 1),
    'l':(0, 0, 0, -1),
    'u':(1, 0, 0, 1),
    ',':(-1, 0, 0, 0),
    '.':(-1, 0, 0, 1),
    'm':(-1, 0, 0, -1),
    'I':(10, 0, 0, 0),
    'O':(10, 0, 0, -10),
    'J':(0, 0, 0, 10),
    'L':(0, 0, 0, -10),
    'U':(10, 0, 0, 10),
    '<':(-10, 0, 0, 0),
    '>':(-10, 0, 0, 10),
    'M':(-10, 0, 0, -10),
    }

SPEED_BINDINGS = {
    'q':(0.01, 0.025),
    'z':(-0.01, -0.025),
    'w':(0.01, 0.0),
    'x':(-0.01, 0.0),
    'e':(0.0, 0.025),
    'c':(0.0, -0.025),
    }

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    input_key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, SETTINGS)
    return input_key


def vels(speed, turn):
    if turn != 0:
        return "currently:\tspeed %s[m/s]\tradius %s[m] " % (speed, turn)
    else:
        return "currently:\tspeed %s[m/s]\tradius %s[m] " % (speed, turn)

def steps(speed, turn):
    return "vel steps:\tspeed %s\tradius %s " % (speed, turn)

if __name__ == "__main__":
    SETTINGS = termios.tcgetattr(sys.stdin)

    PUB = rospy.Publisher('cmd_vel_manual', AgribotManualControl, queue_size=1)
    rospy.init_node('agribot_teleop_keyboard')

    LINEAR_SPEED = rospy.get_param("~linear_speed_step", 0.1)
    TURN_RADIUS = rospy.get_param("~turn_radius_step", 0.25)
    LENGTH = rospy.get_param("~chassis_length", 2.1)
    X = 0.0
    R = 0.0
    STATUS = 0

    try:
        print(MSG)
        print(steps(LINEAR_SPEED, TURN_RADIUS))
        while 1:
            KEY = get_key()
            if KEY in MOVE_BINDINGS.keys():
                DX = MOVE_BINDINGS[KEY][0]*LINEAR_SPEED
                DR = MOVE_BINDINGS[KEY][3]*TURN_RADIUS
                X = X+DX
                R = R+DR
                print(vels(X, R))
            elif KEY in SPEED_BINDINGS.keys():
                LINEAR_SPEED = LINEAR_SPEED + SPEED_BINDINGS[KEY][0]
                TURN_RADIUS = TURN_RADIUS + SPEED_BINDINGS[KEY][1]
                print(steps(LINEAR_SPEED, TURN_RADIUS))
                if STATUS == 14:
                    print(MSG)
                STATUS = (STATUS + 1) % 15
            else:
                X = 0.0
                R = 0.0
                print(vels(X, R))
                if KEY == '\x03':
                    break

            MSG = AgribotManualControl()
            MSG.linear_velocity = X
            MSG.circle_radius = R
            PUB.publish(MSG)

    except rospy.ROSInterruptException:
        pass

    finally:
        MSG = AgribotManualControl()
        MSG.linear_velocity = 0
        MSG.circle_radius = 0
        PUB.publish(MSG)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, SETTINGS)
