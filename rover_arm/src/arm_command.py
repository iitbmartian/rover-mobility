#!/usr/bin/env python
import rospy


class Arm:

    def __init__(self, claw1, claw2, claw3):
        self.shoulder_elbow_actuators = claw1
        self.wrist_finger_actuators = claw2
        self.base_gripper = claw3
        self.current_exceeded = False
        self.currents = [0, 0, 0, 0, 0, 0]
        self.current_threshold = 400
        self.shoulder_actuator = {'name': "Shoulder Actuator", 'speed': 0,
                                  'direction': 0}  # Claw1M1; Stop: 0, Up: 1, Down: -1
        self.elbow_actuator = {'name': "Elbow Actuator", 'speed': 0,
                               'direction': 0}  # Claw1M2; Stop: 0, Extend: 1, Contract: -1
        self.wrist_actuator = {'name': "Wrist Actuator", 'speed': 0,
                               'direction': 0}  # Claw2M1; Stop: 0, Clockwise: 1, Anticlockwise: -1
        self.finger_actuator = {'name': "Finger Actuator", 'speed': 0, 'direction': 0}  # Claw2M2; Stop: 0, .: 1, .: -1
        self.base_rotation = {'name': "Base Rotation", 'speed': 0,
                              'direction': 0}  # Claw3M1; Stop: 0, Clockwise: 1, Anticlockwise: -1
        self.gripper_motor = {'name': "Gripper Motor", 'speed': 0,
                              'direction': 0}  # Claw3M2; Stop: 0, close: 1, open: -1

    def update_arm_steer(self):
        if self.shoulder_elbow_actuators is not None:
            self.runclawM1(self.shoulder_elbow_actuators, self.shoulder_actuator)
            self.runclawM2(self.shoulder_elbow_actuators, self.elbow_actuator)
        if self.wrist_finger_actuators is not None:
            self.runclawM1(self.wrist_finger_actuators, self.wrist_actuator)
            self.runclawM2(self.wrist_finger_actuators, self.finger_actuator)
        if self.base_gripper is not None:
            self.runclawM1(self.base_gripper, self.base_rotation)
            self.runclawM2(self.base_gripper, self.gripper_motor)

    def arm_callback(self,inp):
        data = inp.data
        self.shoulder_actuator['speed'], self.shoulder_actuator['direction'] = int(data[1]), int(data[0])
        self.elbow_actuator['speed'], self.elbow_actuator['direction'] = int(data[3]), int(data[2])
        self.wrist_actuator['speed'], self.wrist_actuator['direction'] = int(data[5]), int(data[4])
        self.finger_actuator['speed'], self.finger_actuator['direction'] = int(data[7]), int(data[6])
        self.base_rotation['speed'], self.base_rotation['direction'] = int(data[9]), int(data[8])
        self.gripper_motor['speed'], self.gripper_motor['direction'] = int(data[11]), int(data[10])

    def arm_stop(self):
        rospy.loginfo('Arm: ' + "Arm commanded to stop")
        if self.shoulder_elbow_actuators is not None:
            self.shoulder_elbow_actuators.ForwardM1(0x80, 0)
            self.shoulder_elbow_actuators.ForwardM2(0x80, 0)
        if self.wrist_finger_actuators is not None:
            self.wrist_finger_actuators.ForwardM1(0x80, 0)
            self.wrist_finger_actuators.ForwardM2(0x80, 0)
        if self.base_gripper is not None:
            self.base_gripper.ForwardM1(0x80, 0)
            self.base_gripper.ForwardM2(0x80, 0)

    def runclawM1(self, claw, cmd_dict):
        if cmd_dict['direction'] == 0:
            claw.ForwardM1(0x80, 0)
        if cmd_dict['direction'] == 1:
            claw.ForwardM1(0x80, cmd_dict['speed'])
            rospy.loginfo('Arm: ' + cmd_dict['name'] + ' commanded to move in Direction=1')
        if cmd_dict['direction'] == -1:
            claw.BackwardM1(0x80, cmd_dict['speed'])
            rospy.loginfo('Arm: ' + cmd_dict['name'] + ' commanded to move in Direction=-1')

    def runclawM2(self, claw, cmd_dict):
        if cmd_dict['direction'] == 0:
            claw.ForwardM2(0x80, 0)
        if cmd_dict['direction'] == 1:
            claw.ForwardM2(0x80, cmd_dict['speed'])
            rospy.loginfo('Arm: ' + cmd_dict['name'] + ' commanded to move in Direction=1')
        if cmd_dict['direction'] == -1:
            claw.BackwardM2(0x80, cmd_dict['speed'])
            rospy.loginfo('Arm: ' + cmd_dict['name'] + ' commanded to move in Direction=-1')

    def current_limiter(self):
        if self.shoulder_elbow_actuators is not None:
            i, self.currents[0], self.currents[1] = self.shoulder_elbow_actuators.ReadCurrents(0x80)
        if self.wrist_finger_actuators is not None:
            i, self.currents[2], self.currents[3] = self.wrist_finger_actuators.ReadCurrents(0x80)
        if self.base_gripper is not None:
            i, self.currents[4], self.currents[5] = self.base_gripper.ReadCurrents(0x80)
        for i in range(6):
            if self.currents[i] > self.current_threshold:
                self.arm_stop()
                self.current_exceeded = True
                return
        self.current_exceeded = False

    def update_current(self):
        if self.shoulder_elbow_actuators is not None:
            i, self.currents[0], self.currents[1] = self.shoulder_elbow_actuators.ReadCurrents(0x80)
        if self.wrist_finger_actuators is not None:
            i, self.currents[2], self.currents[3] = self.wrist_finger_actuators.ReadCurrents(0x80)
        if self.base_gripper is not None:
            i, self.currents[4], self.currents[5] = self.base_gripper.ReadCurrents(0x80)
