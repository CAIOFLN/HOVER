#!/usr/bin/env python3
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist

TIME_STEP = 0.001

class Hover:

    def __init__(self):

        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.twist = Twist()

        self.lower_black = np.array([0, 0, 0])  # HSV
        self.upper_black = np.array([179, 255, 30])  # HSV
        self.controle = self.PID(kp = 0.1, ki = 0, kd = 0, target = 320)
        self.velang_z = 0
    def image_callback(self, msg):

        self.twist.linear.x = -0.2
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.lower_black, self.upper_black)

        height, width, depth = image.shape
        mask[0:height, int(2*width/5):int(3*width/5)] = 0 
        M = cv2.moments(mask)
        if M['m00'] > 0:
            # M['m10'] é o momento horizontal dos pixels
            cx = int(M['m10']/M['m00'])
            self.velang_z = self.controle.compute(cx)
            """if cx > width/2:
                self.twist.angular.z = 0.8
            elif cx < width/2:"""
            
            self.twist.angular.z = self.velang_z
            print(self.twist.angular.z)
            self.cmd_vel_pub.publish(self.twist)
            image = cv2.putText(image, str(cx), (0,50), cv2.FONT_HERSHEY_SIMPLEX, 2, (50,150,255), 3, cv2.LINE_AA)
            image = cv2.circle(image, (width//2, height//2), 5, (0,255,0), -1)  # -1 preenche o ponto
        # Mostra a imagem vista pelo hover
        cv2.imshow("Vision", image)
        cv2.imshow("MASK", mask)
        cv2.waitKey(3)
    
    class PID(object):
        def __init__(self, kp, ki, kd, target):
            self.kp = kp
            self.kd = kd
            self.ki = ki
            self.setpoint = target
            self.error = 0
            self.integral_error = 0
            self.error_last = 0
            self.derivate_error = 0
            self.output = 0
        def compute(self, pos):
            self.error = -(self.setpoint - pos)/20
            print()
            self.integral_error += self.error * TIME_STEP
            self.derivate_error = (self.error - self.error_last)/TIME_STEP
            self.error_last = self.error
            self.output = self.kp * self.error + self.kd * self.derivate_error + self.ki * self.integral_error
            return self.output

rospy.init_node('hover')
hover = Hover()
rospy.spin()
