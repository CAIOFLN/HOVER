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
        self.controle1 = self.PID(kp=50, ki=0, kd=0, target=320)
        self.cx = 0
    def processa_curva(self, msg):
        self.cx = msg.data
        self.twist.angular.z = self.controle1.compute(self.cx)
        print(self.twist.angular.z)
        self.twist.linear.x = -0.4
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
        def compute(self, setpoint):
            self.error = -float(1/(setpoint - self.pos))
            self.integral_error += self.error * TIME_STEP
            self.derivate_error = (self.error - self.error_last)
            self.error_last = self.error
            self.output = self.kp * self.error + self.kd * self.derivate_error + self.ki * self.integral_error
            output_topic.publish(Float64(data=self.output))
            setpoint_topic.publish(Float64(data=(self.kp*setpoint/20)))
            #rospy.sleep(1)
            return self.output

rospy.init_node('hover')
hover = Hover()
rospy.spin()
