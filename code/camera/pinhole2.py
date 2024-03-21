import numpy as np
import cv2
import math 

#x e y del mundo real es en milímertros
#def getlinearregresion(xrw, yrw):
    # pixel
#    xcamera = 208.136 + 1.109*xrw
#    ycamera = 479.752 - 1.284*yrw
    
#return (xcamera,ycamera)
# desplazar la matriz hacia la izquierda y hacia abajo
# para obtener el valor verdadero


#def movepoint(x,y):
    
#    cx = 310.99
#    cy = 230.93
#    truex = x - cx
#    truey = y + cy
    
#    return(truex, truey)
# Función para convertir de píxeles a unidades ópticas
def pixel2optical(pixel_x, pixel_y):
    # Valores de ejemplo, se deben ajustar según las especificaciones de la cámara
    #sensor_width = 6  # Ancho del sensor en mm (ejemplo para un sensor full-frame)
    ancho = 640
    largo = 480
    # Convertir píxeles a unidades ópticas
    optic_x = largo -1 - pixel_y  #(pixel_x - 316.068) * (sensor_width / 640)  # Suponiendo una resolución de 640x480
    optic_y = pixel_x
    
    #(pixel_y - 236.933) * (sensor_width / 480)  # y que la cámara está centrada

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

def getradian(degrees):

    return ((degrees*math.pi)/180)

# de 3d a 2d 
def get2Dpoint(K, point3d, degreevalue):

       
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
    #cap.set(cv2.CAP_PROP_FPS,40)


    # matriz obtenida de los parámetros intrínsecos
    K = np.array([[333.76, 0.0, 310.99],
                  [0.0, 335.08, 230.93],
                  [0.0, 0.0, 1.0]])

    #camera = getlinearregresion(99,111)
    # la z siempre va a ser 0
    #point3d = np.array([90.0,-50.0,0.0,1.0])

    # 40 es el valor obtenido tras medir la inclinación cámara
    #point2d = get2Dpoint(K, point3d, 50)

    #print(point2d)
    # el valor del eje z no es necesario y nos ayuda a poder escalar al
    # valor que estamos buscando del eje x e y
    #finalpoint2D = np.array([point2d[0]/point2d[2], point2d[1]/point2d[2]])
    #print(finalpoint2D)

    #truefinalpoint2D = movepoint(finalpoint2D[0], finalpoint2D[1])
    #print(truefinalpoint2D)

    #center_coordinates = (int(truefinalpoint2D[0]), int(truefinalpoint2D[1]))

    # Definir el radio del punto (en este caso, 5 píxeles)
    #radius = 5

    # Definir el color del punto (en este caso, rojo)
    #color = (0, 0, 255)


    while True:

        # Lee un frame de la cámara 
        ret,frame = cap.read() 
    
        # Gira la cámara 180º porque la cámara está físiscamente dada la vuelta 
        flipped_frame = cv2.flip(frame,0)
        centroid_x, centroid_y = detect_color(flipped_frame, lower_color, upper_color)
    
        #flipped_frame = cv2.circle(flipped_frame, center_coordinates, radius, color, -1)
        #flipped_frame = cv2.circle(flipped_frame, (0, 480) , radius, (0, 255, 0), -1)
        #flipped_frame = cv2.circle(flipped_frame, (0, 0) , radius, (255, 0, 0), -1)
        if centroid_x is not None and centroid_y is not None:
            cv2.circle(frame, (centroid_x, centroid_y), 5, (0, 255, 0), -1)
            cv2.putText(frame, f"Centroide: ({centroid_x}, {centroid_y})", (centroid_x - 100, centroid_y - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # Convertir coordenadas de píxeles a ópticas
            optic_x, optic_y = pixel2optical(centroid_x, centroid_y)
            print(f"Coordenadas ópticas: X={optic_x}, Y={optic_y}")
            
            # Proyectar coordenadas 2D a 3D
            #focal_length = 100  # Longitud focal en mm (debe ser ajustada según las especificaciones de la cámara)
            #x, y, z = project(optic_x, optic_y)
            #print(f"Coordenadas 3D: X={x}, Y={y}, Z={z}")

        cv2.imshow('Frame', flipped_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

