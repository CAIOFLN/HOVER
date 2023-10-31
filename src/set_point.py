#!/usr/bin/env python3
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

TIME_STEP = 0.001
output_topic = rospy.Publisher('/output', Float64, queue_size=1)
setpoint_topic = rospy.Publisher('/setpoint', Float64, queue_size=1)
class Hover:

    def __init__(self):

        self.bridge = cv_bridge.CvBridge()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.recebe_bloco_azul = rospy.Subscriber('image_data', Float64, self.processa_curva)
        self.twist = Twist()
        self.controle1 = self.PID(kp=0.1, ki=0, kd=20, target=320)
        self.cx = 0
    def processa_curva(self, msg):
        self.cx = msg.data
        self.twist.angular.z = self.controle1.compute(self.cx)
        print(self.twist.angular.z)
        self.twist.linear.x = -0.2
        self.cmd_vel_pub.publish(self.twist)
        """if len(msg.data) >= 4:
            if self.trade == 0:
                print(f'area{msg.data[0]}')
                if (msg.data[0] > 45000):
                    self.trade = 1
                else:
                    self.cx_blue = msg.data[1]  # Access the second element of the data array
                    print(self.cx_blue)
                    self.velang_z = self.controle1.compute(self.cx_blue)
                    self.twist.angular.z = self.velang_z
                    self.twist.linear.x = -0.2
                    self.cmd_vel_pub.publish(self.twist)
            elif self.trade == 1:
                if (msg.data[2] < 500):
                    print('oiiiiii')
                    self.twist.angular.z = -0.5
                    self.twist.linear.x = -0.2
                    self.cmd_vel_pub.publish(self.twist)
                elif(msg.data[2] > 120000):
                    self.twist.angular.z = 0
                    self.twist.linear.x = 0
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    print('ahb\dichbdivn\dvns')
                    self.cx_red = msg.data[3]
                    print(self.cx_red)
                    '''if (315<self.cx_red<325):
                        print("oi")
                        self.velang_z = 0'''
                    self.twist.angular.z = self.controle2.compute(self.cx_red)
                    self.twist.linear.x = -0.2
                    self.cmd_vel_pub.publish(self.twist)
            else:
                print("fudeu o sel.trade")"""
                

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
        def compute(self, setpoint):
            self.error = (setpoint - self.pos)/20
            self.integral_error += self.error * TIME_STEP
            self.derivate_error = (self.error - self.error_last)/TIME_STEP
            self.error_last = self.error
            self.output = self.kp * self.error + self.kd * self.derivate_error + self.ki * self.integral_error
            output_topic.publish(Float64(data=self.output))
            setpoint_topic.publish(Float64(data=(self.kp*setpoint/20)))
            #rospy.sleep(1)
            return self.output

rospy.init_node('hover')
hover = Hover()
rospy.spin()
