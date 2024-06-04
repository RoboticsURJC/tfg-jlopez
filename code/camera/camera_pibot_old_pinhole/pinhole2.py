import numpy as np
import cv2
import math 

FX = 333.76
FY = 335.08
CX = 310.99
CY = 230.93

def getradian(degrees):

    return ((degrees*math.pi)/180)

# de 2D a 3D
def backpropagation(x2d, y2d):
    
    x3d = (x2d - CX) / FX
    y3d = (y2d - CY) / FY
    
    z3d = 1.0
    
    point3d = np.array([x3d,y3d,z3d])
    

    return point3d 

# Función para convertir de píxeles a unidades ópticas
def pixel2optical(pixel_x, pixel_y):

    ancho = 640
    largo = 480
    
    optic_x = largo -1 - pixel_y  
    optic_y = pixel_x
    
    return optic_x, optic_y
    
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

# de 3d a 2d 
def propagation(point3d, degreevalue):

    K = np.array([[FX, 0.0, CX],
                  [0.0, FY, CY],
                  [0.0, 0.0, 1.0]])   
    
    rad = getradian(degreevalue)
    
    RT = np.array([[np.cos(rad), 0.0, np.sin(rad),     0.0],
                [    0.0 , 1.0,       0.0,     0.0],
                [-np.sin(rad), 0, np.cos(rad), 110.0]])
    
    aux = RT.dot(point3d)

    return K.dot(aux)
    

if __name__=="__main__":
    # Crea una ventana 
    cv2.namedWindow("Image Feed")
    # Mueve la ventana a una posición en concreto de la pantalla
    cv2.moveWindow("Image Feed", 159, -25)

    # Definir el rango de color HSV del objeto que deseas detectar
    lower_color = np.array([150, 50, 50])
    upper_color = np.array([170, 255, 255])

    # Inicializa la cámara 
    cap = cv2.VideoCapture(0)

    # Setup camera: para agilizar el cómputo
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
    
    while True:

        # Lee un frame de la cámara 
        ret,frame = cap.read() 
    
        # Gira la cámara 180º porque la cámara está físiscamente dada la vuelta 
        flipped_frame = cv2.flip(frame,0)
        flipped_frame = cv2.flip(flipped_frame,1)
        
        centroid_x, centroid_y = detect_color(flipped_frame, lower_color, upper_color)
    
        if centroid_x is not None and centroid_y is not None:
            cv2.circle(flipped_frame, (centroid_x, centroid_y), 5, (0, 255, 0), -1)
            cv2.putText(flipped_frame, f"Centroide: ({centroid_x}, {centroid_y})", (centroid_x - 100, centroid_y - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # Convertir coordenadas de píxeles a ópticas
            optic_x, optic_y = pixel2optical(centroid_x, centroid_y)
            cv2.circle(flipped_frame, (optic_x, optic_y), 5, (255, 0, 0), -1)

            print(f"Coordenadas pixeles: X={centroid_x}, Y={centroid_y}")
            print(f"Coordenadas ópticas: X={optic_x}, Y={optic_y}")
            
            point3d = backpropagation(optic_x, optic_y)
            print(point3d)

        cv2.imshow('Frame', flipped_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

