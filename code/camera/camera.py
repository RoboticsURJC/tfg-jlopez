import cv2
import numpy as np

def detect_color(frame, lower_color, upper_color):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M["m00"] != 0:
            centroid_x = int(M["m10"] / M["m00"])
            centroid_y = int(M["m01"] / M["m00"])
            return centroid_x, centroid_y
    return None, None

#x e y del mundo real es en milímertros
def getlinearregresion(xrw, yrw):
    # pixel
    xcamera = 208.136 + 1.109*xrw
    ycamera = 479.752 - 1.284*yrw
    
    return (xcamera,ycamera)
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


#camera = getlinearregresion(99,111)# sus valores coinciden mejor  
camera = getlinearregresion(111,99) # no muy preciso

# para acceder a sus posiciones hay que usar camera[0]
print(camera)

center_coordinates = (int(camera[0]), int(camera[1]))

# Definir el radio del punto (en este caso, 5 píxeles)
radius = 5

# Definir el color del punto (en este caso, rojo)
color = (0, 0, 255)

while True:

    # Lee un frame de la cámara 
    ret,frame = cap.read() 
    
    # Gira la cámara 180º porque la cámara está físiscamente dada la vuelta 
    flipped_frame = cv2.flip(frame,0)
    
    flipped_frame = cv2.circle(flipped_frame, center_coordinates, radius, color, -1)

    

    
    cv2.imshow('Frame', flipped_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()

#     # Definir el rango de color HSV del objeto que deseas detectar
#     lower_color = np.array([30, 100, 100])
#     upper_color = np.array([70, 255, 255])

#     # Iniciar la captura de la cámara web
#     cap = cv2.VideoCapture(0)

#     while True:
        
#         ret, frame = cap.read()
#         if not ret:
#             break
        
#         centroid_x, centroid_y = detect_color(frame, lower_color, upper_color)
        
#         if centroid_x is not None and centroid_y is not None:
#             cv2.circle(frame, (centroid_x, centroid_y), 5, (0, 255, 0), -1)
#             cv2.putText(frame, f"Centroide: ({centroid_x}, {centroid_y})", (centroid_x - 100, centroid_y - 20),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
#             # Convertir coordenadas de píxeles a ópticas
#             optic_x, optic_y = pixel2optical(centroid_x, centroid_y)
#             print(f"Coordenadas ópticas: X={optic_x}, Y={optic_y}")
            
#             # Proyectar coordenadas 2D a 3D
#             focal_length = 100  # Longitud focal en mm (debe ser ajustada según las especificaciones de la cámara)
#             x, y, z = project(optic_x, optic_y)
#             print(f"Coordenadas 3D: X={x}, Y={y}, Z={z}")

#         cv2.imshow('Frame', frame)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     cap.release()
#     cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()