#!/usr/bin/env python
import rospy
from std_msgs.msg import String


class Drive:
    def __init__(self, driver1, driver2, enb_LED):
        self.rightClaw = driver1
        self.leftClaw = driver2
        self.direction = "stop"  # Stop:0, Forward:1, Right:2, Backward:3, Left:4
        self.speed = 0
        self.current_exceeded = False
        self.currents = [0, 0, 0, 0]
        self.current_threshold = 400
        self.mode = "manual"
        self.enb_LED = enb_LED
        if self.enb_LED:
            self.light_pub = rospy.Publisher('/rover/light', String, queue_size=1)
            self.color_string_msg = String()
            self.color_string_msg.data = "None"

    def fwd(self):
        self.rightClaw.ForwardM1(0x80, self.speed)
        self.rightClaw.ForwardM2(0x80, self.speed)
        self.leftClaw.ForwardM1(0x80, self.speed)
        self.leftClaw.ForwardM2(0x80, self.speed)

    def bwd(self):
        self.rightClaw.BackwardM1(0x80, self.speed)
        self.rightClaw.BackwardM2(0x80, self.speed)
        self.leftClaw.BackwardM1(0x80, self.speed)
        self.leftClaw.BackwardM2(0x80, self.speed)

    def stop(self):
        self.rightClaw.ForwardM1(0x80, 0)
        self.leftClaw.ForwardM1(0x80, 0)
        self.rightClaw.ForwardM2(0x80, 0)
        self.leftClaw.ForwardM2(0x80, 0)

    def left(self):
        self.rightClaw.ForwardM1(0x80, self.speed)
        self.rightClaw.BackwardM2(0x80, self.speed)
        self.leftClaw.ForwardM1(0x80, self.speed)
        self.leftClaw.BackwardM2(0x80, self.speed)

    def right(self):
        self.rightClaw.BackwardM1(0x80, self.speed)
        self.rightClaw.ForwardM2(0x80, self.speed)
        self.leftClaw.ForwardM1(0x80, self.speed)
        self.leftClaw.BackwardM2(0x80, self.speed)

    def update_steer(self):
        if self.direction == "stop":
            self.stop()
        elif self.direction == "forward":
            self.fwd()
        elif self.direction == "clockwise":
            self.right()
        elif self.direction == "backward":
            self.bwd()
        elif self.direction == "anticlockwise":
            self.left()

    def drive_callback(self, inp):
        data = inp.data
        self.speed, self.direction, self.autonomous = inp.speed, inp.direction, inp.
        if self.enb_LED:
            if self.autonomous == "manual":
                self.color_string_msg.data = "Blue"
                self.light_pub.publish(self.color_string_msg)
            if self.autonomous == "autonomous":
                self.color_string_msg.data = "Red"
                self.light_pub.publish(self.color_string_msg)
        if self.direction == "stop":
            pass
        elif self.direction == "forward":
            rospy.loginfo('Drive: Rover commanded to move Forward')
        elif self.direction == "clockwise":
            rospy.loginfo('Drive: Rover commanded to turn Clockwise')
        elif self.direction == "backward":
            rospy.loginfo('Drive: Rover commanded to move Backward')
        elif self.direction == "anticlockwise":
            rospy.loginfo('Drive: Rover commanded to turn Anti-Clockwise')

    def current_limiter(self):
        (i, self.currents[0], self.currents[1]) = self.rightClaw.ReadCurrents(0x80)
        (i, self.currents[2], self.currents[3]) = self.leftClaw.ReadCurrents(0x80)
        for i in range(4):
            if self.currents[i] > self.current_threshold:
                self.stop()
                self.current_exceeded = True
                return
        self.current_exceeded = False

    def update_current(self):
        (i, self.currents[0], self.currents[1]) = self.rightClaw.ReadCurrents()
        (i, self.currents[2], self.currents[3]) = self.leftClaw.ReadCurrents()
