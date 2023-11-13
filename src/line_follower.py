#!/usr/bin/env python3
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class Controle:

    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.control_sub = rospy.Subscriber('/set_point', Float64, self.controller) #NÃO RECEBE IMAGE
        self.twist = Twist()
        self.controle1 = self.PID(kp=.00001, ki=0, kd=.00001, target=320)
        self.cx = 0
    def controller(self, msg):
        self.cx = msg.data
        self.twist.angular.z = self.controle1.compute(self.cx)
        print(self.twist.angular.z)
        self.twist.linear.x = 1
        self.cmd_vel_pub.publish(self.twist)


    class PID(object):
        def __init__(self, kp, ki, kd, target):
            self.kp = kp
            self.kd = kd
            self.ki = ki
            self.pos= target  #posição atual do robo
            self.error = 0
            self.integral_error = 0 
            self.error_last = 0
            self.derivate_error = 0
            self.output = 0
        def compute(self, erro_msg):
            self.error = erro_msg
            self.integral_error += self.error
            self.derivate_error = (self.error - self.error_last)
            self.error_last = self.error
            self.output = self.kp * self.error + self.kd * self.derivate_error + self.ki * self.integral_error

            return self.output

#rospy.init_node('Controle')
#control = Controle()
#rospy.spin()