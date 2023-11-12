import cv2
import numpy as np
from pubcam import camera_calibration
# Função para capturar e exibir o vídeo da câmera

def nothing(x):
    pass
cv2.namedWindow('Image')
cv2.createTrackbar('Hue1', 'Image', 0, 179, nothing)
cv2.createTrackbar('Saturation1', 'Image', 0, 255, nothing)
cv2.createTrackbar('Value1', 'Image', 0, 255, nothing)
cv2.createTrackbar('Hue2', 'Image', 0, 179, nothing)
cv2.createTrackbar('Saturation2', 'Image', 0, 255, nothing)
cv2.createTrackbar('Value2', 'Image', 0, 255, nothing)
def mostrar_video_camera():
    # Abre a câmera
    cap = cv2.VideoCapture(0)

    # Verifica se a câmera foi aberta com sucesso
    if not cap.isOpened():
        print("Erro ao abrir a câmera")
        return

    # Loop para capturar e exibir o vídeo
    while True:
        # Captura o frame da câmera
        ret, frame = cap.read()

        # Verifica se a captura foi bem-sucedida
        if not ret:
            print("Erro ao capturar o frame")
            break
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv = camera_calibration(hsv)
        h1 = cv2.getTrackbarPos('Hue1', 'Image')
        s1 = cv2.getTrackbarPos('Saturation1', 'Image')
        v1 = cv2.getTrackbarPos('Value1', 'Image')

        h2 = cv2.getTrackbarPos('Hue2', 'Image')
        s2 = cv2.getTrackbarPos('Saturation2', 'Image')
        v2 = cv2.getTrackbarPos('Value2', 'Image')

        lower_green = np.array([h1, s1, v1])
        upper_green = np.array([h2, s2, v2])
        
        mask = cv2.inRange(hsv, lower_green, upper_green)
        not_mask = cv2.bitwise_not(mask)
        cv2.imshow('Video da Cadsamera', not_mask)
        result = cv2.bitwise_and(hsv, hsv, mask=not_mask)
        #result = cv2.GaussianBlur(result, (5, 5), 0)

        # Exibe o frame
        cv2.imshow('Video da Camera', frame)
        cv2.imshow('Resultado do filtro', result)
        # Verifica se a tecla 'q' foi pressionada para encerrar o loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Libera os recursos ao encerrar
    cap.release()
    cv2.destroyAllWindows()

# Chama a função para mostrar o vídeo da câmera
mostrar_video_camera()
