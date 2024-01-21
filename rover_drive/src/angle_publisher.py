#!/usr/bin/env python3
# license removed for brevity
import sys
import os
import rospy
from std_msgs.msg import Int32



class angle_publisher():
    
    def __init__(self):

        self.pub = rospy.Publisher('/rover/auto_cam_angle',Int32, queue_size=10)
        
    def angle_generator(self):
        angle=Int32()
        while not rospy.is_shutdown():
            angle.data=int(input("Enter an angle:"))

            self.pub.publish(angle)
        
           
            



            



    
if __name__ == '__main__':
    rospy.init_node('camera',anonymous=True)
    device=angle_publisher()
    device.angle_generator()
    rospy.spin()