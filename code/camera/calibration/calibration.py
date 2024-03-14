import numpy as np 
import cv2, time

# Crea una ventana 
cv2.namedWindow("Image Feed")
# Mueve la ventana a una posición en concreto de la pantalla
cv2.moveWindow("Image Feed", 159, -25)
# Inicializa la cámara 
cap = cv2.VideoCapture(0)

# Setup camera: para agilizar el cómputo
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
cap.set(cv2.CAP_PROP_FPS,40)

# Creo una marca de tiempo
prev_frame_time = time.time()
# Inicializan los contadores de las imágenes y los frames 
cal_image_count = 0
frame_count = 0

while True:

    # Lee un frame de la cámara 
    ret,frame = cap.read() 
    
    # Gira la cámara 180º porque la cámara está físiscamente dada la vuelta 
    flipped_frame = cv2.flip(frame,0)

    # Al haber detectado un frame, se aumenta su contador 
    frame_count += 1

    # Cada 30 frames, se guarda una imagen y se inicializa
    # el contador de frames de nuevo a 0 y se incrementa el contador de imágenes
    if frame_count == 30:
        cv2.imwrite("cal_image_" + str(cal_image_count) + ".jpg", flipped_frame)
        cal_image_count += 1
        frame_count = 0

    # Se calculan los FPS y se muestran en la pantalla 
    new_frame_time = time.time()
    fps = 1/(new_frame_time - prev_frame_time)
    prev_frame_time = new_frame_time
    cv2.putText(flipped_frame, "FPS" + str(int(fps)), (10,40), cv2.FONT_HERSHEY_PLAIN, 3, (100, 255, 0), 2, cv2.LINE_AA)

    cv2.imshow("Image Feed", flipped_frame)

    # Se usa la tecla 'q'para finalizar el programa
    #key = cv2.waitKey(1) & 0xFF
    #if key == ord("q"): break

# Cuando todo acabe: libera los recursos de la  cámara 
cap.release()



