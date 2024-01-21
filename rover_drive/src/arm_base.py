#!/usr/bin/env python3



from roboclaw_3 import Roboclaw

import rospy

from std_msgs.msg import Int32

import signal

import sys

import os

from serial.serialutil import SerialException as SerialException

global stop

stop=0

class Arm:

    def __init__(self, claw2):

        self.base_elbow_motors = claw2

        self.base_motor = {'name': "Base Rotation", 'speed': 0,

                           'direction': "stop"} # Claw3M2; Stop: 0, Clockwise: 1, Anticlockwise: -1

        self.goal_angle=0
        self.enc_data=0
        self.encoder_value=0
        self.config = 0
        self.speed=0

        self.isconfig = False

        self.configComplete = False

        self.runtimer = rospy.Rate(10)



        #PID control

        self.prev_error = 0

        self.sum_error = 0

        self.K_p = 0.01

        self.K_d = 0.0001
        
        self.K_i = 0

    

    def init_enc_vals(self):

        if self.base_elbow_motors is not None:

            self.base_elbow_motors.ForwardM2(0x84, 0)

            self.base_elbow_motors.ResetEncoders(0x84)

            self.base_elbow_motors.SetEncM2(0x84, 0)



    def PID(self):

        error = self.goal_angle - self.enc_data

        diff_error = error - self.prev_error

        self.sum_error = self.sum_error + error

        self.speed = self.K_p*error + self.K_d*diff_error + self.K_i*self.sum_error

        self.prev_error = error



    def update_arm_steer(self):



        if self.base_elbow_motors is not None:
            debug_new=self.base_elbow_motors.ReadEncM2(0x84)
            if len(debug_new)==3:

                _, self.enc_data,_ = debug_new

            if (self.encoder_value)>=(self.enc_data+20):

                self.base_motor['direction'] = 'forward'

            elif (self.encoder_value)<=(self.enc_data-20):

                self.base_motor['direction'] = 'backward'

            else:

                self.base_motor['direction'] = 'stop'



            if self.enc_data < -3200:

                if self.base_motor['direction'] == 'forward':

                    self.runclawM2(self.base_elbow_motors, self.base_motor)

                else:

                    self.base_elbow_motors.ForwardM2(0x84, 0)

            elif -3200 <= self.enc_data < 3200:

                self.runclawM2(self.base_elbow_motors, self.base_motor)

            elif self.enc_data>=3200:

                if self.base_motor['direction'] == 'backward':

                    self.runclawM2(self.base_elbow_motors, self.base_motor)

                else:

                    self.base_elbow_motors.ForwardM2(0x84, 0)



    def arm_callback(self, inp):

        self.config = 0

        self.isconfig = False
        print(inp.data)

        self.goal_angle=inp.data
        print(self.goal_angle)

        self.encoder_value=self.goal_angle*36
        if self.goal_angle == 100:
            Arm_1.arm_stop()
            # rospy.spin()
            sys.exit()



        # self.base_motor['speed'], self.base_motor['direction'] = int(inp.base_motor.speed), inp.base_motor.direction

    def arm_stop(self):
        global stop
        if self.base_elbow_motors is not None:

            while True:

               

                if (self.enc_data)>=20:

                    self.base_motor['direction'] = 'backward'

                elif (self.enc_data)<=(-20):

                    self.base_motor['direction'] = 'forward'

                else:
                    stop=1
                    self.base_motor['direction'] = 'stop'

                    for i in range (0,2):
                        self.base_elbow_motors.ForwardM2(0x84, 0)
                        
                    sys.exit()
                    break

                

    def arm_break(self):

        rospy.loginfo('Arm: ' + "Arm commanded to break")



        if self.base_elbow_motors is not None:

            self.base_elbow_motors.ForwardM2(0x84, 0)

            self.base_elbow_motors.ForwardM2(0x84, 0)



    def runclawM2(self, claw, cmd_dict):
        debug_neww=self.base_elbow_motors.ReadEncM2(0x84)
        if len(debug_neww)==3:

            _, self.enc_data,_ = debug_neww
            print(debug_neww)
        if cmd_dict['direction'] == "stop":

            claw.ForwardM2(0x84, 0)

            # rospy.loginfo('Arm: '+ cmd_dict['name'] + ' commanded to stop')

        if cmd_dict['direction'] == "forward":

            claw.ForwardM2(0x84, 30) # rospy.loginfo('Arm: ' + cmd_dict['name'] + ' commanded to move in Direction = 1')

        if cmd_dict['direction'] == "backward":

            claw.BackwardM2(0x84, 30)

            # rospy.loginfo('Arm: ' + cmd_dict['name'] + ' commanded to move in Direction = -1')



def sigint_handler_arm(signal, frame):

    Arm_1.arm_stop()

    sys.exit(0)





def enable_actuators_motors():

    print()

    enb_base = "y"

    #enb_all = input("Enable all Actuators/Motors? ")

    if enb_base == "y" or enb_base == "Y" or enb_base == "yes" or enb_base == "Yes":

        enable_base_elbow_rot_motors = True



    else:

        enable_base_elbow_rot_motors = False



    return enable_base_elbow_rot_motors



if __name__ == "__main__":

    signal.signal(signal.SIGINT, sigint_handler_arm)



    rospy.init_node("Arm_Node")

    rospy.loginfo("Starting Arm_Node")

    iteration_time = rospy.Rate(1)

    enable_base_elbow_rot_motors = enable_actuators_motors()

    if enable_base_elbow_rot_motors:

        while True:

            try:

                base_elbow_rot_motors = Roboclaw("/dev/ttyUSB6", 115200)

                break

            except SerialException:

                rospy.logwarn("Could not connect to Base and Elbow Motors' RoboClaw, retrying...")

                iteration_time.sleep()

        rospy.loginfo("Connected to Base and Elbow RoboClaw")

    else:

        base_elbow_rot_motors = None

    Arm_1 = Arm(base_elbow_rot_motors)

    Arm_1.init_enc_vals()



    rospy.loginfo("Subscribing to /rover/auto_cam_angle...")

    rospy.Subscriber("/rover/auto_cam_angle", Int32, Arm_1.arm_callback)

    rospy.loginfo("Subscribed to /rover/auto_cam_angle")

    run_time = rospy.Rate(10)

    while (not rospy.is_shutdown() and stop==0):

        # Arm_1.PID()

        Arm_1.update_arm_steer()

        # print("Enc = "+str(base_elbow_rot_motors.ReadEncM2(0x84)))
        # print("Speed = "+str(base_elbow_rot_motors.ReadSpeedM2(0x84)))
        

        run_time.sleep()

