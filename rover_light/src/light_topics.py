#!/usr/bin/env python

import relay_commands
import rospy
from std_msgs.msg import String
from rover_msgs.msg import drive_msg, arm_msg
import time
import signal
import os

task_completion = 0
drive_status = "none"
light_pub = rospy.Publisher('/rover/light', String, queue_size=1)
light_out = String()


# rostopic pub -1 /rover/tasks_status std_msgs/String "data: completed"


def tasks_status(inp):
    global task_completion, drive_status
    if inp.data == "completed":
        task_completion = 1
        light_out.data = "none"
        light_pub.publish(light_out)
        time.sleep(0.02)
        light_out.data = "green"
        light_pub.publish(light_out)
        time.sleep(0.6)
        light_out.data = "none"
        light_pub.publish(light_out)
        time.sleep(0.6)
        light_out.data = "green"
        light_pub.publish(light_out)
        time.sleep(0.6)
        light_out.data = "none"
        light_pub.publish(light_out)
        time.sleep(0.6)
        if drive_status == "autonomous":
            light_out.data = "red"
            light_pub.publish(light_out)
        elif drive_status == "manual":
            light_out.data = "blue"
            light_pub.publish(light_out)
        task_completion = 0
    else:
        task_completion = 0


def topics_callback(inp):
    global task_completion, drive_status
    if task_completion == 0:
        if inp.mode == "autonomous":
            if drive_status != "autonomous":
                light_out.data = "none"
                light_pub.publish(light_out)
                time.sleep(0.02)
                drive_status = "autonomous"
                light_out.data = "red"
                light_pub.publish(light_out)
        elif inp.mode == "manual":
            if drive_status != "manual":
                light_out.data = "none"
                light_pub.publish(light_out)
                time.sleep(0.02)
                drive_status = "manual"
                light_out.data = "blue"
                light_pub.publish(light_out)


if __name__ == "__main__":
    rospy.init_node("LED_Topic_Node")
    rospy.Subscriber("/rover/drive_directives", drive_msg, topics_callback)
    rospy.Subscriber("/rover/tasks_status", String, tasks_status)
    rospy.spin()
