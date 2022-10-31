#!/usr/bin/env python

from roboclaw_3 import Roboclaw
import rospy
from std_msgs.msg import Float64MultiArray
from serial.serialutil import SerialException as SerialException
import signal
from tabulate import tabulate
import sys
import os

light_pub = rospy.Publisher('/rover/measure', Float64MultiArray, queue_size=1)
measure_msg = Float64MultiArray()


# SIGINT Handler to escape loops. Use Ctrl-C to exit
def sigint_handler(signal, frame):
    sys.exit(0)


def enable_roboclaws():
    print()
    enb_all = input("Enable all RoboClaws? ")
    if enb_all == "y" or enb_all == "Y" or enb_all == "yes" or enb_all == "Yes":
        enable_shoulder_elbow_actuators = True
        enable_base_finger_motors = True
        enable_wrist_rotation_motors = True
        enable_Rdrive = True
        enable_Ldrive = True
    else:
        enb_Rdrive = input("Enable Right Drive Roboclaw?")
        if enb_Rdrive == "y" or enb_Rdrive == "Y" or \
                enb_Rdrive == "yes" or enb_Rdrive == "Yes":
            enable_Rdrive = True
        else:
            enable_Rdrive = False

        enb_Ldrive = input("Enable Left Drive Roboclaw?")
        if enb_Ldrive == "y" or enb_Ldrive == "Y" or \
                enb_Ldrive == "yes" or enb_Ldrive == "Yes":
            enable_Ldrive = True
        else:
            enable_Ldrive = False

        enb_shoulder_elbow_actuators = input("Enable Shoulder Elbow Actuators Roboclaw? ")
        if enb_shoulder_elbow_actuators == "y" or enb_shoulder_elbow_actuators == "Y" or \
                enb_shoulder_elbow_actuators == "yes" or enb_shoulder_elbow_actuators == "Yes":
            enable_shoulder_elbow_actuators = True
        else:
            enable_shoulder_elbow_actuators = False

        enb_base_finger_motors = input("Enable Base Finger Motors Roboclaw? ")
        if enb_base_finger_motors == "y" or enb_base_finger_motors == "Y" or \
                enb_base_finger_motors == "yes" or enb_base_finger_motors == "Yes":
            enable_base_finger_motors = True
        else:
            enable_base_finger_motors = False

        enb_wrist_rotation_motors = input("Enable Wrist Rotation Motors Roboclaw? ")
        if enb_wrist_rotation_motors == "y" or enb_wrist_rotation_motors == "Y" or \
                enb_wrist_rotation_motors == "yes" or enb_wrist_rotation_motors == "Yes":
            enable_wrist_rotation_motors = True
        else:
            enable_wrist_rotation_motors = False
    return enable_Rdrive, enable_Ldrive, enable_shoulder_elbow_actuators, enable_base_finger_motors, enable_wrist_rotation_motors


if __name__ == "__main__":
    signal.signal(signal.SIGINT, sigint_handler)

    rospy.init_node("Measure_Node")
    rospy.loginfo("Starting Measure Node")
    iter_time = rospy.Rate(1)

    enable_Rdrive, enable_Ldrive, enable_shoulder_elbow_actuators, enable_base_finger_motors, enable_wrist_rotation_motors = enable_roboclaws()

    if enable_Rdrive:
        while True:
            try:
                Rdrive = Roboclaw("/dev/Fdrive", 9600)
                break
            except SerialException:
                rospy.logwarn("Could not connect to Rdrive Claw, retrying...")
                iter_time.sleep()
        rospy.loginfo("Connected to Rdrive Claw")
    else:
        Rdrive = None

    if enable_Ldrive:
        while True:
            try:
                Ldrive = Roboclaw("/dev/Bdrive", 9600)
                break
            except SerialException:
                rospy.logwarn("Could not connect to Ldrive Claw, retrying...")
                iter_time.sleep()
        rospy.loginfo("Connected to Ldrive Claw")
    else:
        Ldrive = None

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
        rospy.loginfo("Connected to Wrist and Rotation Claw")
    else:
        wrist_rotation_motors = None

    claws = [Rdrive, Ldrive, shoulder_elbow_actuators, base_finger_motors, wrist_rotation_motors]
    volt_claws = [0, 0, 0, 0, 0]
    current_claws = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
    run_time = rospy.Rate(1)

    while True:
        os.system('clear')
        for k in range(len(claws)):
            i, volt_claws[k] = claws[k].ReadMainBatteryVoltage(0x80)
            i, a, b = claws[k].ReadCurrents(0x80)
            current_claws[k] = [a, b]
        table = [['Claw', 'Voltage', 'Current-M1', 'Current-M2'],['Right Drive', volt_claws[0], current_claws[0][0], current_claws[0][1]], ['Left Drive', volt_claws[1], current_claws[1][0], current_claws[1][1]], ['Shoulder-Elbow', volt_claws[2], current_claws[2][0], current_claws[2][1]], ['Base-Finger', volt_claws[3], current_claws[3][0], current_claws[3][1]], ['Wrist-Rotation', volt_claws[4], current_claws[4][0], current_claws[4][1]]]
        print(tabulate(table, headers = 'firstrow',tablefmt='fancy_grid'))
        run_time.sleep()
