#!/usr/bin/env python
import rospy


class Arm:

    def __init__(self, claw1, claw2, claw3, arduinoR):
        self.shoulder_elbow_actuators = claw1
        self.base_finger_motors = claw2
        self.wrist_rotation_motors = claw3
        self.rotation_servo = arduinoR
        self.current_exceeded = False
        self.currents = [0, 0, 0, 0, 0, 0, 0, 0]
        self.current_threshold = 1000
        self.shoulder_actuator = {'name': "Shoulder Actuator", 'speed': 0,
                                  'direction': "stop"}  # Claw1M1; Stop: 0, Up: 1, Down: -1
        self.elbow_actuator = {'name': "Elbow Actuator", 'speed': 0,
                               'direction': "stop"}  # Claw1M2; Stop: 0, Extend: 1, Contract: -1\
        self.base_motor = {'name': "Base Rotation", 'speed': 0,
                           'direction': "stop"}  # Claw3M1; Stop: 0, Clockwise: 1, Anticlockwise: -1
        self.finger_motor = {'name': "Finger Actuator", 'speed': 0,
                             'direction': "stop"}  # Claw2M1; Stop: 0, Clockwise: 1, Anticlockwise: -1
        self.wrist_actuator = {'name': "Wrist Actuator", 'speed': 0, 'direction': "stop"}  # Claw2M2; Stop: 0, .: 1, .: -1
        self.rotation_motor = {'name': "Gripper Motor", 'speed': 0,
                               'direction': "stop"}  # Arduino - Pin 9
        self.elbow_rotation = {'name': "Elbow Rotation", 'speed': 0,
                               'direction': "stop"}  # Arduino - Pin 10

    def update_arm_steer(self):
        if self.shoulder_elbow_actuators is not None:
            self.runclawM1(self.shoulder_elbow_actuators, self.shoulder_actuator)
            self.runclawM2(self.shoulder_elbow_actuators, self.elbow_actuator)
        if self.base_finger_motors is not None:
            self.runclawM1(self.base_finger_motors, self.base_motor)
            self.runclawM2(self.base_finger_motors, self.finger_motor)
        if self.wrist_rotation_motors is not None:
            self.runclawM1(self.wrist_rotation_motors, self.wrist_actuator)
            # self.runclawM2(self.wrist_rotation_motors, self.rotation_motor)
        if self.elbow_rotation is not None:
            self.runclawM1(self.rotation_servo, self.elbow_rotation)
            self.runclawM1(self.rotation_servo, self.rotation_motor)

    def arm_callback(self, inp):
        self.shoulder_actuator['speed'], self.shoulder_actuator['direction'] = int(inp.shoulder_actuator.speed), inp.shoulder_actuator.direction
        self.elbow_actuator['speed'], self.elbow_actuator['direction'] = int(inp.elbow_actuator.speed), inp.elbow_actuator.direction
        self.base_motor['speed'], self.base_motor['direction'] = int(inp.base_motor.speed), inp.base_motor.direction
        self.finger_motor['speed'], self.finger_motor['direction'] = int(inp.finger_motor.speed), inp.finger_motor.direction
        self.wrist_actuator['speed'], self.wrist_actuator['direction'] = int(inp.wrist_actuator.speed), inp.wrist_actuator.direction
        self.rotation_motor['speed'], self.rotation_motor['direction'] = int(inp.rotation_motor.speed), inp.rotation_motor.direction
        self.elbow_rotation['speed'], self.elbow_rotation['direction'] = int(inp.elbow_rotation.speed), inp.elbow_rotation.direction

    def arm_stop(self):
        rospy.loginfo('Arm: ' + "Arm commanded to stop")
        if self.shoulder_elbow_actuators is not None:
            self.shoulder_elbow_actuators.ForwardM1(0x80, 0)
            self.shoulder_elbow_actuators.ForwardM2(0x80, 0)
        if self.base_finger_motors is not None:
            self.base_finger_motors.ForwardM1(0x80, 0)
            self.base_finger_motors.ForwardM2(0x80, 0)
        if self.wrist_rotation_motors is not None:
            self.wrist_rotation_motors.ForwardM1(0x80, 0)
            self.wrist_rotation_motors.ForwardM2(0x80, 0)
        if self.rotation_servo is not None:
            self.rotation_motor.ForwardM1(0x80, 0)
            self.elbow_rotation.ForwardM2(0x80, 0)

    def runclawM1(self, claw, cmd_dict):
        if cmd_dict['direction'] == "stop":
            claw.ForwardM1(0x80, 0)
        if cmd_dict['direction'] == "forward":
            claw.ForwardM1(0x80, cmd_dict['speed'])
            rospy.loginfo('Arm: ' + cmd_dict['name'] + ' commanded to move in Direction = 1')
        if cmd_dict['direction'] == "backward":
            claw.BackwardM1(0x80, cmd_dict['speed'])
            rospy.loginfo('Arm: ' + cmd_dict['name'] + ' commanded to move in Direction = -1')

    def runclawM2(self, claw, cmd_dict):
        if cmd_dict['direction'] == "stop":
            claw.ForwardM2(0x80, 0)
        if cmd_dict['direction'] == "forward":
            claw.ForwardM2(0x80, cmd_dict['speed'])
            rospy.loginfo('Arm: ' + cmd_dict['name'] + ' commanded to move in Direction = 1')
        if cmd_dict['direction'] == "backward":
            claw.BackwardM2(0x80, cmd_dict['speed'])
            rospy.loginfo('Arm: ' + cmd_dict['name'] + ' commanded to move in Direction = -1')

    def current_limiter(self):
        if self.shoulder_elbow_actuators is not None:
            i, self.currents[0], self.currents[1] = self.shoulder_elbow_actuators.ReadCurrents(0x80)
        if self.base_finger_motors is not None:
            i, self.currents[2], self.currents[3] = self.base_finger_motors.ReadCurrents(0x80)
        if self.wrist_rotation_motors is not None:
            i, self.currents[4], self.currents[5] = self.wrist_rotation_motors.ReadCurrents(0x80)
        # No current measurement in arduino
        # if self.carriage_motors is not None:
        #     i, self.currents[6], self.currents[7] = self.carriage_motors.ReadCurrents(0x80)
        for i in range(8):
            if self.currents[i] > self.current_threshold:
                self.arm_stop()
                self.current_exceeded = True
                return
        self.current_exceeded = False

    def update_current(self):
        if self.shoulder_elbow_actuators is not None:
            i, self.currents[0], self.currents[1] = self.shoulder_elbow_actuators.ReadCurrents(0x80)
        if self.base_finger_motors is not None:
            i, self.currents[2], self.currents[3] = self.base_finger_motors.ReadCurrents(0x80)
        if self.wrist_rotation_motors is not None:
            i, self.currents[4], self.currents[5] = self.wrist_rotation_motors.ReadCurrents(0x80)
        # No current measurement in arduino
        # if self.carriage_motors is not None:
        #     i, self.currents[6], self.currents[7] = self.carriage_motors.ReadCurrents(0x80)
