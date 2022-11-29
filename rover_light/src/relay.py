#!/usr/bin/env python

import relay_commands
import rospy
from std_msgs.msg import String
from rover_msgs.msg import drive_msg, arm_msg
import signal


# Blue - 2, Red - 3, Green - 4

def sigint_handler(signal, frame):
    rb.switchoff(2)
    rb.switchoff(3)
    rb.switchoff(4)


def led_callback(inp):
    if inp.mode == "autonomous":
        rb.switchon(3)  # Red
    elif inp.mode == "manual":
        rb.switchon(2)  # Blue
    elif inp.mode == "none":
        rb.switchon(4)  # Green
    else:
        rb.switchoff(2)
        rb.switchoff(3)
        rb.switchoff(4)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, sigint_handler)

    rospy.init_node("LED_Node")
    rospy.loginfo("Starting LED Node")

    rb = relay_commands.FT245R()
    dev_list = rb.list_dev()
    dev = dev_list[0]
    print(dev)
    rb.connect(dev)
    rb.switchoff(2)
    rb.switchoff(3)
    rb.switchoff(4)
    rospy.Subscriber("/rover/drive_directives", drive_msg, led_callback)
    rospy.spin()
