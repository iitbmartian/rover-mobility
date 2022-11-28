#!/usr/bin/env python

from roboclaw_3 import Roboclaw
from drive_commands import Drive
import rospy
from std_msgs.msg import Float64MultiArray, String
from rover_msgs.msg import drive_msg
from serial.serialutil import SerialException as SerialException
import signal
import sys


# SIGINT Handler to escape loops. Use Ctrl-C to exit
def sigint_handler(signal, frame):
    Drive.stop()
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, sigint_handler)

    rospy.init_node("Drive_Node")
    rospy.loginfo("Starting Drive Node")
    iter_time = rospy.Rate(1)

    while True:
        try:
            frontClaw = Roboclaw("/dev/Fdrive", 9600)
            break
        except SerialException:
            rospy.logwarn("Couldn't connect to Front Drive Claw. trying again...")
            iter_time.sleep()
    rospy.loginfo("Connected to Front Drive Claw")

    while True:
        try:
            centerClaw = Roboclaw("/dev/Cdrive", 9600)
            break
        except SerialException:
            rospy.logwarn("Couldn't connect to Center Drive Claw. trying again...")
            iter_time.sleep()
    rospy.loginfo("Connected to Center Drive Claw")

    while True:
        try:
            backClaw = Roboclaw("/dev/Bdrive", 9600)
            break
        except SerialException:
            rospy.logwarn("Couldn't connect to Back Drive Claw. trying again...")
            iter_time.sleep()
    rospy.loginfo("Connected to Back Drive Claw")

    # initialising Drive object-------------------
    Drive = Drive(frontClaw, centerClaw, backClaw)
    Drive.stop()

    rospy.loginfo("Subscribing to /rover/drive_directives")
    rospy.Subscriber("/rover/drive_directives", drive_msg, Drive.drive_callback)
    rospy.loginfo("Subscribed to /rover/drive_directives")
    run_time = rospy.Rate(10)

    while not rospy.is_shutdown():
        Drive.current_limiter()
        if not Drive.current_exceeded:
            Drive.update_steer()
        else:
            rospy.logwarn("Drive stopped due to excess current")
            rospy.loginfo(Drive.currents)
        run_time.sleep()
