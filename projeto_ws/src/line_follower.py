#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist


bridge = CvBridge()

class Sensores:

    def __init__(self):

        self.sensores = np.array([-1, -1, -1, -1, -1])
        self.publish_time = 0.05

        self.img_sensor_full_left = rospy.Subscriber("/camera/rgb/image_raw", Image, self.imgFullLeftCallback, queue_size = 10)
        self.sensor_pub = rospy.Publisher("/dados", Int32MultiArray, queue_size=10)
        self.timer_pub = rospy.Timer(rospy.Duration(self.publish_time), self.timerCallback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        rospy.logwarn("Rodando") 
        self.cmd_vel_msg = Twist()


    def imgFullLeftCallback (self, ros_img):

        img = bridge.imgmsg_to_cv2(ros_img)       
        self.img_full_left = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        self.cor_full_left = np.mean(self.img_full_left)
        if (self.cor_full_left > 115):
            print(self.cor_full_left)
            rospy.logwarn("A") 
            self.sensores[0] = 0
            self.cmd_vel_msg.linear.x = -0.1
        else:
            rospy.logwarn("C") 
            self.cmd_vel_msg.angular.z = 1
            rospy.logwarn("Cor identificada diferente de branco e preto!\n") 


        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def timerCallback(self, event):
        msg = Int32MultiArray()
        msg.data = self.sensores
        self.sensor_pub.publish(msg)

if __name__ == '__main__':

    try:
        rospy.init_node("sensores")
        Sensores()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass