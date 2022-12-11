#!/usr/bin/env python

import relay_commands
import rospy
from std_msgs.msg import String
from rover_msgs.msg import drive_msg, arm_msg
import signal
import os
import time


# Blue - 2, Red - 3, Green - 4

def sigint_handler(signal, frame):
    rb.switchoff(2)
    rb.switchoff(3)
    rb.switchoff(4)


def led_callback(inp):
    if inp.data == "red":
        rb.switchon(3)  # Red
    elif inp.data == "blue":
        rb.switchon(2)  # Blue
    elif inp.data == "green":
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
    os.system('sudo -S chmod 666 /dev/bus/usb/{bus}/{address}'.format(bus=str(dev.bus).zfill(3), address = str(dev.address).zfill(3)))
    rb.connect(dev)
    rb.switchoff(2)
    rb.switchoff(3)
    rb.switchoff(4)
    rospy.Subscriber("/rover/light", String, led_callback)
    rospy.spin()
