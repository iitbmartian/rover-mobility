#!/usr/bin/env python

from arm_command import Arm
from roboclaw_3 import Roboclaw
import rospy
from std_msgs.msg import Float64MultiArray
from serial.serialutil import SerialException as SerialException
import signal
import sys
import os


# SIGINT handler
def sigint_handler_arm(signal, frame):
    Arm.arm_stop()
    sys.exit(0)


def enable_actuators_motors():
    print()
    enb_all = input("Enable all Actuators/Motors? ")
    if enb_all == "y" or enb_all == "Y" or enb_all == "yes" or enb_all == "Yes":
        enable_shoulder_elbow_actuators = True
        enable_wrist_finger_actuators = True
        enable_base_gripper = True
    else:
        enb_shoulder_elbow_actuators = input("Enable Shoulder Elbow Actuators ")
        if enb_shoulder_elbow_actuators == "y" or enb_shoulder_elbow_actuators == "Y" or \
                enb_shoulder_elbow_actuators == "yes" or enb_shoulder_elbow_actuators == "Yes":
            enable_shoulder_elbow_actuators = True
        else:
            enable_shoulder_elbow_actuators = False

        enb_wrist_finger_actuators = input("Enable Wrist Finger Actuators ")
        if enb_wrist_finger_actuators == "y" or enb_wrist_finger_actuators == "Y" or \
                enb_wrist_finger_actuators == "yes" or enb_wrist_finger_actuators == "Yes":
            enable_wrist_finger_actuators = True
        else:
            enable_wrist_finger_actuators = False

        enb_base_gripper = input("Enable Base Gripper Motors ")
        if enb_base_gripper== "y" or enb_base_gripper == "Y" or \
                enb_base_gripper == "yes" or enb_base_gripper == "Yes":
            enable_base_gripper = True
        else:
            enable_base_gripper = False

    print(enable_shoulder_elbow_actuators, enable_wrist_finger_actuators, enable_base_gripper)
    return enable_shoulder_elbow_actuators, enable_wrist_finger_actuators, enable_base_gripper


if __name__ == "__main__":
    signal.signal(signal.SIGINT, sigint_handler_arm)

    rospy.init_node("Arm_Node")
    rospy.loginfo("Starting Arm_Node")
    iter_time = rospy.Rate(1)
    enable_shoulder_elbow_actuators, enable_wrist_finger_actuators, enable_base_gripper = enable_actuators_motors()

    if enable_shoulder_elbow_actuators:
        while True:
            try:
                shoulder_elbow_actuators = Roboclaw("/dev/shoulder_elbow_actuators", 9600)
                break
            except SerialException:
                rospy.logwarn("Could not connect to Shoulder and Elbow Claw, retrying...")
                iter_time.sleep()
        rospy.loginfo("Connected to Shoulder and Elbow Claw")
    else:
        shoulder_elbow_actuators = None

    if enable_wrist_finger_actuators:
        while True:
            try:
                wrist_finger_actuators = Roboclaw("/dev/wrist_finger_actuators", 9600)
                break
            except SerialException:
                rospy.logwarn("Could not connect to Wrist and Finger Claw, retrying...")
                iter_time.sleep()
        rospy.loginfo("Connected to Wrist and Finger Claw")
    else:
        wrist_finger_actuators = None

    if enable_base_gripper:
        while True:
            try:
                base_gripper = Roboclaw("/dev/base_gripper", 9600)
                break
            except SerialException:
                rospy.logwarn("Could not connect to Base and Gripper Claw, retrying...")
                iter_time.sleep()
        rospy.loginfo("Connected to Base and Gripper Claw")
    else:
        base_gripper = None

    # initialising Arm Object-------------------
    Arm = Arm(shoulder_elbow_actuators, wrist_finger_actuators, base_gripper)
    Arm.arm_stop()

    rospy.loginfo("Subscribing to /rover/arm_directives...")
    rospy.Subscriber("/rover/arm_directives", Float64MultiArray, Arm.arm_callback)
    rospy.loginfo("Subscribed to /rover/arm_directives")
    run_time = rospy.Rate(10)
    while not rospy.is_shutdown():
        Arm.current_limiter()
        if not Arm.current_exceeded:
            Arm.update_arm_steer()
        else:
            rospy.logwarn("Arm stopped due to excess current")
            rospy.loginfo(Arm.currents)
        run_time.sleep()
