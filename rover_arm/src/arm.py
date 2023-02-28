#!/usr/bin/env python

from arm_command import Arm
from roboclaw_3 import Roboclaw
from arduino_rot import Arduino_Rot
import rospy
from std_msgs.msg import Float64MultiArray
from rover_msgs.msg import arm_msg
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
        enable_base_finger_motors = True
        enable_wrist_rotation_motors = True
        enable_carriage_motors = True
    else:
        enb_shoulder_elbow_actuators = input("Enable Shoulder Elbow Actuators? ")
        if enb_shoulder_elbow_actuators == "y" or enb_shoulder_elbow_actuators == "Y" or \
                enb_shoulder_elbow_actuators == "yes" or enb_shoulder_elbow_actuators == "Yes":
            enable_shoulder_elbow_actuators = True
        else:
            enable_shoulder_elbow_actuators = False

        enb_base_finger_motors = input("Enable Base Finger Motors? ")
        if enb_base_finger_motors == "y" or enb_base_finger_motors == "Y" or \
                enb_base_finger_motors == "yes" or enb_base_finger_motors == "Yes":
            enable_base_finger_motors = True
        else:
            enable_base_finger_motors = False

        enb_wrist_rotation_motors = input("Enable Wrist Motor? ")
        if enb_wrist_rotation_motors == "y" or enb_wrist_rotation_motors == "Y" or \
                enb_wrist_rotation_motors == "yes" or enb_wrist_rotation_motors == "Yes":
            enable_wrist_rotation_motors = True
        else:
            enable_wrist_rotation_motors = False

        enb_rotation_arduino = input("Enable Rotation Arduino? ")
        if enb_rotation_arduino == "y" or enb_rotation_arduino == "Y" or \
                enb_rotation_arduino == "yes" or enb_rotation_arduino == "Yes":
            enable_rotation_arduino = True
        else:
            enable_rotation_arduino = False

    return enable_shoulder_elbow_actuators, enable_base_finger_motors, enable_wrist_rotation_motors, enable_rotation_arduino


if __name__ == "__main__":
    signal.signal(signal.SIGINT, sigint_handler_arm)

    rospy.init_node("Arm_Node")
    rospy.loginfo("Starting Arm_Node")
    iter_time = rospy.Rate(1)
    enable_shoulder_elbow_actuators, enable_base_finger_motors, enable_wrist_rotation_motors, enable_rotation_arduino = enable_actuators_motors()

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

    if enable_base_finger_motors:
        while True:
            try:
                base_finger_motors = Roboclaw("/dev/base_finger_motors", 9600)
                break
            except SerialException:
                rospy.logwarn("Could not connect to Base and Finger Claw, retrying...")
                iter_time.sleep()
        rospy.loginfo("Connected to Base and Finger Claw")
    else:
        base_finger_motors = None

    if enable_wrist_rotation_motors:
        while True:
            try:
                wrist_rotation_motors = Roboclaw("/dev/wrist_rotation_motors", 9600)
                break
            except SerialException:
                rospy.logwarn("Could not connect to Wrist and Rotation Claw, retrying...")
                iter_time.sleep()
        rospy.loginfo("Connected to Base and Rotation Claw")
    else:
        wrist_rotation_motors = None

    if enable_rotation_arduino:
        while True:
            try:
                rotation_arduino = Roboclaw("/dev/rotation_arduino", 9600)
                break
            except SerialException:
                rospy.logwarn("Could not connect to Rotation Arduino Adapter, retrying...")
                iter_time.sleep()
        rospy.loginfo("Connected to Rotation Arduino Adapter")
    else:
        rotation_arduino = None

    # initialising Arm Object-------------------
    Arm = Arm(shoulder_elbow_actuators, base_finger_motors, wrist_rotation_motors, rotation_arduino)
    Arm.arm_stop()

    rospy.loginfo("Subscribing to /rover/arm_directives...")
    rospy.Subscriber("/rover/arm_directives", arm_msg, Arm.arm_callback)
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
