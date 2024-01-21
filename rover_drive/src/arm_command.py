#!/usr/bin/env python
import rospy
from time import sleep
import serial
import serial.tools.list_ports as ports
shoulder_elbow_rot_addr=0x83
base_elbow_addr=0x84
wrist_rot_addr=0x85
PORT='/dev/ttyUSB1'
class Arm:

    def __init__(self, claw1):
        self.ser = serial.Serial()
        self.ser.baudrate = 115200
        self.ser.port = PORT
        self.ser.open()
        self.shoulder_elbow_rot_actuators = claw1
        self.base_elbow_motors = claw1
        

        self.current_exceeded = False
        self.currents = [0, 0, 0, 0, 0, 0, 0, 0]
        self.current_threshold = 1000
        self.shoulder_actuator = {'name': "Shoulder Actuator", 'speed': 0,
                                  'direction': "stop"}  # Claw1M1; Stop: 0, Up: 1, Down: -1
        self.elbow_actuator = {'name': "Elbow Actuator", 'speed': 0,
                               'direction': "stop"}  # Claw1M2; Stop: 0, Extend: 1, Contract: -1\
        self.base_motor = {'name': "Base Rotation", 'speed': 0,
                           'direction': "stop"}  # Claw3M1; Stop: 0, Clockwise: 1, Anticlockwise: -1
        # self.finger_motor = {'name': "Finger Actuator", 'speed': 0,
        #                      'direction': "stop"}  # Claw2M1; Stop: 0, Clockwise: 1, Anticlockwise: -1
        self.wrist_actuator = {'name': "Wrist Actuator", 'speed': 0, 'direction': "stop"}  # Claw2M2; Stop: 0, .: 1, .: -1
        # self.rotation_motor = {'name': "Gripper Motor", 'speed': 0,
        #                        'direction': "stop"}  # Arduino - Pin 9
        self.elbow_rotation = {'name': "Elbow Rotation", 'speed': 0,
                               'direction': "stop"}  # Arduino - Pin 10
        self.gripper={'name': "Gripper", 'speed': 0,
                               'direction': "stop"}
        self.gripper_rot={'name': "Gripper rotation", 'speed': 0,
                               'direction': "stop"}
    def update_arm_steer(self):
        if self.shoulder_elbow_rot_actuators is not None:
            self.runclawM1(self.shoulder_elbow_rot_actuators, self.shoulder_actuator,shoulder_elbow_rot_addr)
            self.runclawM2(self.shoulder_elbow_rot_actuators, self.elbow_rotation,shoulder_elbow_rot_addr)
        if self.base_elbow_motors is not None:
            self.runclawM1(self.base_elbow_motors, self.base_motor,base_elbow_addr)
            self.runclawM2(self.base_elbow_motors, self.elbow_actuator,base_elbow_addr)
        if (self.gripper['direction'] == "forward") and (self.gripper_rot=="forward"):
            self.serial_send(22)
        elif (self.gripper['direction'] == "forward") and (self.gripper_rot=="backward"):
            self.serial_send(21)
        elif (self.gripper['direction'] == "backward") and (self.gripper_rot=="forward"):
            self.serial_send(12)
        elif (self.gripper['direction'] == "backward") and (self.gripper_rot=="backward"):
            self.serial_send(11)
        elif (self.gripper['direction'] == "backward") and (self.gripper_rot=="stop"):
            self.serial_send(10)
        elif (self.gripper['direction'] == "stop") and (self.gripper_rot=="backward"):
            self.serial_send(1)
        elif (self.gripper['direction'] == "forward") and (self.gripper_rot=="stop"):
            self.serial_send(20)
        elif (self.gripper['direction'] == "stop") and (self.gripper_rot=="forward"):
            self.serial_send(2)
        elif (self.gripper['direction'] == "stop") and (self.gripper_rot=="stop"):
            self.serial_send(0)

       
        # if self.elbow_rotation is not None:
        #     self.runclawM1(self.rotation_servo, self.elbow_rotation)
        #     self.runclawM1(self.rotation_servo, self.rotation_motor)
    # def serial_send(self,a):
       
    #     command=str(a)
    #     command=command+'\n'
    #     self.ser.write(command.encode())
    #     print("Sent "+command)

    def arm_callback(self, inp):
        self.shoulder_actuator['speed'], self.shoulder_actuator['direction'] = int(inp.shoulder_actuator.speed), inp.shoulder_actuator.direction
        self.elbow_actuator['speed'], self.elbow_actuator['direction'] = int(inp.elbow_actuator.speed), inp.elbow_actuator.direction
        self.base_motor['speed'], self.base_motor['direction'] = int(inp.base_motor.speed), inp.base_motor.direction
        self.gripper['speed'], self.gripper['direction'] = int(inp.gripper.speed), inp.gripper.direction
        self.wrist_actuator['speed'], self.wrist_actuator['direction'] = int(inp.wrist_actuator.speed), inp.wrist_actuator.direction
        self.gripper_rot['speed'], self.gripper_rot['direction'] = int(inp.gripper_rot.speed), inp.gripper_rot.direction
        self.elbow_rotation['speed'], self.elbow_rotation['direction'] = int(inp.elbow_motor.speed), inp.elbow_motor.direction

    def arm_stop(self):
        rospy.loginfo('Arm: ' + "Arm commanded to stop")
        if self.shoulder_elbow_rot_actuators is not None:
            self.shoulder_elbow_rot_actuators.ForwardM1(shoulder_elbow_rot_addr, 0)
            self.shoulder_elbow_rot_actuators.ForwardM2(shoulder_elbow_rot_addr, 0)
        if self.base_elbow_motors is not None:
            self.base_elbow_motors.ForwardM1(base_elbow_addr, 0)
            self.base_elbow_motors.ForwardM2(base_elbow_addr, 0)
        # if self.wrist_rotation_motors is not None:
        #     self.wrist_rotation_motors.ForwardM1(wrist_rot_addr, 0)
        #     self.wrist_rotation_motors.ForwardM2(wrist_rot_addr, 0)
        # if self.rotation_servo is not None:
        #     self.rotation_motor.ForwardM1(0x80, 0)
        #     self.elbow_rotation.ForwardM2(0x80, 0)

    def runclawM1(self, claw, cmd_dict,address):
        if cmd_dict['direction'] == "stop":
            claw.ForwardM1(address, 0)
        if cmd_dict['direction'] == "forward":
            claw.ForwardM1(address, cmd_dict['speed'])
            rospy.loginfo('Arm: ' + cmd_dict['name'] + ' commanded to move in Direction = 1')
        if cmd_dict['direction'] == "backward":
            claw.BackwardM1(address, cmd_dict['speed'])
            rospy.loginfo('Arm: ' + cmd_dict['name'] + ' commanded to move in Direction = -1')

    def runclawM2(self, claw, cmd_dict,address):
        if cmd_dict['direction'] == "stop":
            claw.ForwardM2(address, 0)
        if cmd_dict['direction'] == "forward":
            claw.ForwardM2(address, cmd_dict['speed'])
            rospy.loginfo('Arm: ' + cmd_dict['name'] + ' commanded to move in Direction = 1')
        if cmd_dict['direction'] == "backward":
            claw.BackwardM2(address, cmd_dict['speed'])
            rospy.loginfo('Arm: ' + cmd_dict['name'] + ' commanded to move in Direction = -1')

    # def current_limiter(self):
    #     if self.shoulder_elbow_actuators is not None:
    #         i, self.currents[0], self.currents[1] = self.shoulder_elbow_actuators.ReadCurrents(0x80)
    #     if self.base_finger_motors is not None:
    #         i, self.currents[2], self.currents[3] = self.base_finger_motors.ReadCurrents(0x80)
    #     if self.wrist_rotation_motors is not None:
    #         i, self.currents[4], self.currents[5] = self.wrist_rotation_motors.ReadCurrents(0x80)
    #     # No current measurement in arduino
    #     # if self.carriage_motors is not None:
    #     #     i, self.currents[6], self.currents[7] = self.carriage_motors.ReadCurrents(0x80)
    #     for i in range(8):
    #         if self.currents[i] > self.current_threshold:
    #             self.arm_stop()
    #             self.current_exceeded = True
    #             return
    #     self.current_exceeded = False

    # def update_current(self):
    #     if self.shoulder_elbow_actuators is not None:
    #         i, self.currents[0], self.currents[1] = self.shoulder_elbow_actuators.ReadCurrents(0x80)
    #     if self.base_finger_motors is not None:
    #         i, self.currents[2], self.currents[3] = self.base_finger_motors.ReadCurrents(0x80)
    #     if self.wrist_rotation_motors is not None:
    #         i, self.currents[4], self.currents[5] = self.wrist_rotation_motors.ReadCurrents(0x80)
    #     # No current measurement in arduino
    #     # if self.carriage_motors is not None:
    #     #     i, self.currents[6], self.currents[7] = self.carriage_motors.ReadCurrents(0x80)
