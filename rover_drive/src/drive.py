#!/usr/bin/env python

from roboclaw_3 import Roboclaw
from drive_commands import Drive
import rospy
from std_msgs.msg import Float64MultiArray, String
from rover_msgs.msg import drive_msg
from serial.serialutil import SerialException as SerialException
import signal
import sys

light_pub = rospy.Publisher('/rover/light', String, queue_size=1)
color_string_msg = String()


# SIGINT Handler to escape loops. Use Ctrl-C to exit
def sigint_handler(signal, frame):
    if enb_LED:
        color_string_msg.data = "None"
        light_pub.publish(color_string_msg)
    Drive.stop()
    sys.exit(0)


if __name__ == "__main__":
    enb_LEDq = input("Enable LEDs? ")
    if enb_LEDq == "y" or enb_LEDq == "Y" or \
            enb_LEDq == "yes" or enb_LEDq == "Yes":
        enb_LED = True
    else:
        enb_LED = False
    signal.signal(signal.SIGINT, sigint_handler)

    rospy.init_node("Drive_Node")
    rospy.loginfo("Starting Drive Node")
    iter_time = rospy.Rate(1)

    while True:
        try:
            rightClaw = Roboclaw("/dev/Rdrive", 9600)
            break
        except SerialException:
            rospy.logwarn("Couldn't connect to Drive Right Claw. trying again...")
            iter_time.sleep()
    rospy.loginfo("Connected to Drive Right Claw")

    while True:
        try:
            leftClaw = Roboclaw("/dev/Ldrive", 9600)
            break
        except SerialException:
            rospy.logwarn("Couldn't connect to Drive Left Claw. trying again...")
            iter_time.sleep()
    rospy.loginfo("Connected to Drive Left Claw")

    # initialising Drive object-------------------
    Drive = Drive(rightClaw, leftClaw, enb_LED)
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
