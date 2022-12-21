#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray, String
from rover_msgs.msg import drive_msg
import sys

is_manual = False
drive_pub = rospy.Publisher('/rover/drive_directives', drive_msg, queue_size=1)
drive_out = drive_msg()


def manual_callback(inp):
    global is_manual
    if inp.mode == "manual" and inp.speed != 0:
        is_manual = True
        drive_out.mode = inp.mode
        drive_out.direction = inp.direction
        drive_out.speed = inp.speed
        drive_pub.publish(drive_out)
    elif inp.speed == 0 and inp.direction == "stop":
        is_manual = False
        drive_out.mode = inp.mode
        drive_out.direction = inp.direction
        drive_out.speed = inp.speed
        drive_pub.publish(drive_out)
    else:
        is_manual = False


def autonomous_callback(inp):
    global is_manual
    if not is_manual:
        drive_out.mode = inp.mode
        drive_out.direction = inp.direction
        drive_out.speed = inp.speed
        drive_pub.publish(drive_out)


if __name__ == "__main__":
    rospy.init_node("Drive_Topic_Node")
    rospy.Subscriber("/rover/drive_directives/autonomous", drive_msg, autonomous_callback)
    rospy.Subscriber("/rover/drive_directives/manual", drive_msg, manual_callback)
    rospy.spin()
