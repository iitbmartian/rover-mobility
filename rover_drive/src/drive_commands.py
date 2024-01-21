#!/usr/bin/env python
import rospy
from std_msgs.msg import String


class Drive:
    def __init__(self, driver1):
        self.frontClaw = driver1

        self.direction = "stop"
        self.speed = 0
        self.current_exceeded = False
        self.currents = [0, 0, 0, 0, 0, 0]
        self.current_threshold = 1000
        self.mode = "manual"

    def fwd(self):
        self.frontClaw.ForwardM1(0x80, self.speed)
        self.frontClaw.BackwardM2(0x80, self.speed)
        self.frontClaw.ForwardM1(0x81, self.speed)
        self.frontClaw.BackwardM2(0x81, self.speed)
        self.frontClaw.ForwardM1(0x82, self.speed)
        self.frontClaw.BackwardM2(0x82, self.speed)

    def bwd(self):
        self.frontClaw.BackwardM1(0x80, self.speed)
        self.frontClaw.ForwardM2(0x80, self.speed)
        self.frontClaw.BackwardM1(0x81, self.speed)
        self.frontClaw.ForwardM2(0x81, self.speed)
        self.frontClaw.BackwardM1(0x82, self.speed)
        self.frontClaw.ForwardM2(0x82, self.speed)

    def stop(self):
        self.frontClaw.ForwardM1(0x80, 0)
        self.frontClaw.ForwardM2(0x80, 0)
        self.frontClaw.ForwardM1(0x81, 0)
        self.frontClaw.ForwardM2(0x81, 0)
        self.frontClaw.ForwardM1(0x82, 0)
        self.frontClaw.ForwardM2(0x82, 0)

    def right(self):
        self.frontClaw.ForwardM1(0x80, self.speed)
        self.frontClaw.ForwardM2(0x80, self.speed)
        self.frontClaw.ForwardM1(0x81, self.speed)
        self.frontClaw.ForwardM2(0x81, self.speed)
        self.frontClaw.ForwardM1(0x82, self.speed)
        self.frontClaw.ForwardM2(0x82, self.speed)

    def left(self):
        self.frontClaw.BackwardM1(0x80, self.speed)
        self.frontClaw.BackwardM2(0x80, self.speed)
        self.frontClaw.BackwardM1(0x81, self.speed)
        self.frontClaw.BackwardM2(0x81, self.speed)
        self.frontClaw.BackwardM1(0x82, self.speed)
        self.frontClaw.BackwardM2(0x82, self.speed)

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
        self.speed, self.direction, self.mode = int(inp.speed), inp.direction, inp.mode
        if self.direction == "stop":
            print("stop")
        elif self.direction == "forward":
            rospy.loginfo('Drive: Rover commanded to move Forward')
        elif self.direction == "clockwise":
            rospy.loginfo('Drive: Rover commanded to turn Clockwise')
        elif self.direction == "backward":
            rospy.loginfo('Drive: Rover commanded to move Backward')
        elif self.direction == "anticlockwise":
            rospy.loginfo('Drive: Rover commanded to turn Anti-Clockwise')

    def current_limiter(self):
        (i, self.currents[0], self.currents[1]) = self.frontClaw.ReadCurrents(0x80)
        (i, self.currents[2], self.currents[3]) = self.frontClaw.ReadCurrents(0x80)
        (i, self.currents[4], self.currents[5]) = self.frontClaw.ReadCurrents(0x80)
        for i in range(6):
            if self.currents[i] > self.current_threshold:
                self.stop()
                self.current_exceeded = True
                return
        self.current_exceeded = False

    def update_current(self):
        (i, self.currents[0], self.currents[1]) = self.frontClaw.ReadCurrents(0x80)
        (i, self.currents[2], self.currents[3]) = self.frontClaw.ReadCurrents(0x80)
        (i, self.currents[4], self.currents[5]) = self.frontClaw.ReadCurrents(0x80)