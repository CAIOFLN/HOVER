#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError

def _map(value:float, from_low:float, from_high:float, to_low:float, to_high:float)-> float:
    # Mapeia o valor de from_low/from_high para to_low/to_high
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low

class SET_POINT:
    def __init__(self):
        # Inicialização do nó ROS
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.set_point_pub = rospy.Publisher('set_point', Float64, queue_size=1)
        self.image_publisher = rospy.Publisher('image_with_masks', Image, queue_size=1)  # Tópico para publicar a imagem com as máscaras

        # Definição das faixas de cores em formato HSV
        self.lower_green = np.array([40, 100, 100])   # HSV (verde)
        self.upper_green = np.array([80, 255, 255])  # HSV (verde)
        self.lower_red = np.array([0, 100, 100])    # HSV (vermelho)
        self.upper_red = np.array([20, 255, 255])   # HSV (vermelho)

        self.combined_mask_x_history = []  # Lista para armazenar os últimos valores
        self.average_window_size = 20  # Tamanho da janela para calcular a média

    def image_callback(self, msg):
        # Conversão da mensagem da imagem para o formato OpenCV
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        #height, width, depth = image.shape
        #image[0:height, int(2*width/5):int(3*width/5)] = 0
        # Detecção da cor verde
        green_mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        green_mask_largest = np.zeros_like(green_mask)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(green_contours) >= 2:
        # Classifique os contornos pelo tamanho em ordem decrescente
            green_contours = sorted(green_contours, key=cv2.contourArea, reverse=True)

            # Pegue os dois maiores contornos
            green_largest_contours = green_contours[:2]

            # Desenhe os dois maiores contornos na máscara
            for contour in green_largest_contours:
                cv2.drawContours(green_mask_largest, [contour], 0, 255, -1)

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



        gM = cv2.moments(green_mask_largest)
        rM = cv2.moments(red_mask_largest)

        k = 0.1  #constante para ajustar a escala do erro
        
        error = (gM['m00'] - rM['m00']) * k
        
        self.combined_mask_x_history.append(error)
        if len(self.combined_mask_x_history) > self.average_window_size:
            self.combined_mask_x_history.pop(0)
        combined_mask_x_avg = sum(self.combined_mask_x_history) / len(self.combined_mask_x_history)  

        set_point_msg = Float64(data=combined_mask_x_avg)
        self.set_point_pub.publish(set_point_msg)

        height, width, depth = image.shape
        
        try:
            # Desenhe o círculo na imagem
            point = width/2 + _map(combined_mask_x_avg, -40000, 40000, -320, 320)
            cv2.circle(image, (int(np.ceil(point)), int(np.ceil(height/2))), 10, (0, 0, 255), -1)  # O valor -1 preenche o círculo
        except Exception as e:
            print("no mass center found\n", e)
    
        cv2.imshow("Hover\'s Vision", image)
        cv2.waitKey(3)

rospy.init_node('Set_Point')
Processa = SET_POINT()
rospy.spin()