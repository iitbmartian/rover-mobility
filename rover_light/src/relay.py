#!/usr/bin/env python

import relay_commands
import rospy
from std_msgs.msg import String
import signal

# Blue - 2, Red - 3, Green - 4

def sigint_handler(signal, frame):
    rb.switchoff(2)
    rb.switchoff(3)
    rb.switchoff(4)


def led_callback(color):
    Color = color.data
    if Color == "Red" or Color == "RED" or Color == "R" or Color == "r" or Color == "red":
        rb.switchon(3)
    elif Color == "Blue" or Color == "BLUE" or Color == "B" or Color == "b" or Color == "blue":
        rb.switchon(2)
    elif Color == "Green" or Color == "GREEN" or Color == "G" or Color == "g" or Color == "green":
        rb.switchon(4)
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
    rb.connect(dev)
    rospy.Subscriber("/rover/light", String, led_callback)
    rospy.spin()
