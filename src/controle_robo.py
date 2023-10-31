#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError

class SET_POINT:
    def __init__(self):
        # Inicialização do nó ROS
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.image_data_pub = rospy.Publisher('image_data', Float64, queue_size=1)
        self.image_publisher = rospy.Publisher('image_with_masks', Image, queue_size=1)  # Tópico para publicar a imagem com as máscaras

        # Definição das faixas de cores em formato HSV
        self.lower_green = np.array([40, 100, 100])   # HSV (verde)
        self.upper_green = np.array([80, 255, 255])  # HSV (verde)
        self.lower_red = np.array([0, 100, 100])    # HSV (vermelho)
        self.upper_red = np.array([20, 255, 255])   # HSV (vermelho)
        
        self.Mgreen = 0
        self.Mred = 0



        self.combined_mask_x_history = []  # Lista para armazenar os últimos valores
        self.average_window_size = 25  # Tamanho da janela para calcular a média

    def image_callback(self, msg):
        # Conversão da mensagem da imagem para o formato OpenCV
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        height, width, depth = image.shape
        image[0:height, int(2*width/5):int(3*width/5)] = 0
        # Detecção da cor verde
        green_mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        green_mask_largest = np.zeros_like(green_mask)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(green_contours) >= 2:
        # Classifique os contornos pelo tamanho em ordem decrescente
            self.fudeu_green = 0
            green_contours = sorted(green_contours, key=cv2.contourArea, reverse=True)

            # Pegue os dois maiores contornos
            green_largest_contours = green_contours[:2]

            # Desenhe os dois maiores contornos na máscara
            for contour in green_largest_contours:
                cv2.drawContours(green_mask_largest, [contour], 0, 255, -1)
        else:
            self.fudeu_green = 1
            # Trate o caso em que não há dois contornos

        # Detecção da cor vermelha
        red_mask = cv2.inRange(hsv, self.lower_red, self.upper_red)
        red_mask_largest = np.zeros_like(green_mask)
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(red_contours) >= 2:
        # Classifique os contornos pelo tamanho em ordem decrescente
            self.fudeu_red = 0
            red_contours = sorted(red_contours, key=cv2.contourArea, reverse=True)

            # Pegue os dois maiores contornos
            red_largest_contours = red_contours[:2]

            # Desenhe os dois maiores contornos na máscara
            for contour in red_largest_contours:
                cv2.drawContours(red_mask_largest, [contour], 0, 255, -1)
        else:
            self.fudeu_red = 1
            # Trate o caso em que não há dois contornos


        combined_mask = cv2.bitwise_or(green_mask_largest, red_mask_largest)

        self.Mgreen = cv2.moments(green_mask_largest)
        self.Mred = cv2.moments(red_mask_largest)
        MR = self.Mred
        MG = self.Mgreen

        XcmR = 0
        XcmG = 0

        if (MG['m00']>0 or MR['m00']>0):

            if (MG['m00']>0):
                XcmG = int(MG['m10']/MG['m00'])
            else:
                XcmG = 0
            if (MR['m00']>0):
                XcmR = int(MR['m10']/MR['m00'])
            else:
                XcmR = 0

            Xcm = (MR['m00']*XcmR+MG['m00']*XcmG)/(MR['m00']+MG['m00'])
            self.combined_mask_x_history.append(Xcm)
            if len(self.combined_mask_x_history) > self.average_window_size:
                self.combined_mask_x_history.pop(0)
            combined_mask_x_avg = sum(self.combined_mask_x_history) / len(self.combined_mask_x_history)  


        # Exibição das máscaras e da máscara combinada
        #cv2.imshow("green_mask", green_mask)
        #cv2.imshow("red_mask", red_mask)
        cv2.imshow("green_mask_largest", green_mask_largest)
        cv2.imshow("red_mask_largest", red_mask_largest)
        #cv2.imshow("Result Image", combined_mask)
        cv2.waitKey(3)
        image_with_masks_msg = self.bridge.cv2_to_imgmsg(combined_mask, "mono8")
        self.image_publisher.publish(image_with_masks_msg)

        # Publicação da posição horizontal da máscara combinada
        image_data_msg = Float64(data=combined_mask_x_avg)
        self.image_data_pub.publish(image_data_msg)


rospy.init_node('Set_Point')
Processa = SET_POINT()
rospy.spin()
