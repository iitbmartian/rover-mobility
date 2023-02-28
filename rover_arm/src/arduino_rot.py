#!/usr/bin/env python

import serial
import rospy

class Arduino_Rot(object):

    def __init__(self, serial_port, baud=9600):
        self.serial_port = serial_port
        self.pin9_speed = 90
        self.pin10_speed = 90
        
        self.ser = serial.Serial(
            port=self.serial_port,
            baudrate=baud,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1)
        
        try:
            if not self.ser.is_open():
                self.ser.open()
        except serial.SerialException:
            rospy.logwarn("Cannot connect to given Arduino device")

    def send_cmd(self):
        cmd = self.pin9_speed*1000 + self.pin10_speed
        self.ser.write(str.encode(f"s{cmd}t"))

    # names for backward compatability
    def ForwardM1(self, port, speed):
        # for pin 9
        # maps speed (0-255) to (90 - 0)
        self.pin9_speed = (90/-255)*(speed - 255)
        self.send_cmd()

    # names for backward compatability
    def BackwardM1(self, port, speed):
        # for pin 9
        # maps speed (0-255) to (90 - 180)
        self.pin9_speed = 180 + (90/255)*(speed - 255)
        self.send_cmd()

    # names for backward compatability
    def ForwardM2(self, port, speed):
        # for pin 10
        # maps speed (0-255) to (90 - 0)
        self.pin10_speed = (90/-255)*(speed - 255)
        self.send_cmd()

    # names for backward compatability
    def BackwardM2(self, port, speed):
        # for pin 10
        # maps speed (0-255) to (90 - 180)
        self.pin10_speed = 180 + (90/255)*(speed - 255)
        self.send_cmd()
