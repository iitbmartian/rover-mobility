#!/usr/bin/env python

import os
import signal
import sys

import rospy
from serial.serialutil import SerialException as SerialException

from drive_commands import Drive
from roboclaw_3 import Roboclaw
from rover_msgs.msg import drive_msg


# SIGINT Handler to escape loops. Use Ctrl-C to exit
def sigint_handler(signal, frame):
    Drive.stop()
    sys.exit(0)

def ping(hostname):
    response = os.system("ping -c 1 -t 1 " + hostname)
    if response != 0:
        print(hostname + "'s connection is weak")
        return False
    else:
        return True

if __name__ == "__main__":
    signal.signal(signal.SIGINT, sigint_handler)

    rospy.init_node("Drive_Node")
    rospy.loginfo("Starting Drive Node")
    iter_time = rospy.Rate(1)

    hostname = input("Enter Hostname of Basestation: ")

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

    counter = 0
    while not rospy.is_shutdown():
        Drive.current_limiter()
        if not Drive.current_exceeded:
            if counter == 20:
                for i in range(10):
                    if ping(hostname):
                        Drive.update_steer()
                        break
                    else:
                        if i == 10:
                            Drive.stop()
                            continue_command = input("Resume Operations? ")
                            if continue_command != "y":
                                sys.exit(0)
                            else:
                                Drive.stop()        
                counter = 0
            else:
                Drive.update_steer()
                counter += 1
        else:
            rospy.logwarn("Drive stopped due to excess current")
            rospy.loginfo(Drive.currents)
        run_time.sleep()
