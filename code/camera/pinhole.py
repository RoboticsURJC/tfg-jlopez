
import numpy as np
import cv2
import math 

# de 2d a 3d
# def get3Dpoint(point2d, K):

#     point2d_x = point2d[0]
#     point2d_y = point2d[1]

#     cx = K[][]
#     cy = K[][]

#     fx = K[][]
#     fy = K[][]


#     x = ((point2d_x-cx)*d)/fx
#     y  = ((point2d_y -cy)*d)/ fy

#     z = depth

    

#     return

#x e y del mundo real es en milímertros
def getlinearregresion(xrw, yrw):
    # pixel
    xcamera = 208.136 + 1.109*xrw
    ycamera = 479.752 - 1.284*yrw
    
    return (xcamera,ycamera)

def getradian(degrees):

    return ((degrees*math.pi)/180)

# de 3d a 2d 
def get2Dpoint(K, point3d, degreevalue):

       
    rad = getradian(degreevalue)

    #K = np.array([[333.76, 0.0, 310.99],
    #              [0.0, 335.08, 230.93],
    #              [0.0, 0.0, 1.0]])
    RT = np.array([[np.cos(rad), 0.0, np.sin(rad),     0.0],
                [    0.0 , 1.0,       0.0,     0.0],
                [-np.sin(rad), 0, np.cos(rad), 110.0]])
    
    aux = RT.dot(point3d)

    return K.dot(aux)
    

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


# matriz obtenida de los parámetros intrínsecos
K = np.array([[333.76, 0.0, 310.99],
                  [0.0, 335.08, 230.93],
                  [0.0, 0.0, 1.0]])

#camera = getlinearregresion(99,111)
# la z siempre va a ser 0
point3d = np.array([0.0,0.0,0.0,1.0])

# 40 es el valor obtenido tras medir la inclinación cámara
point2d = get2Dpoint(K, point3d, 40)

print(point2d)
# el valor del eje z no es necesario y nos ayuda a poder escalar al
# valor que estamos buscando del eje x e y
finalpoint2D = np.array([ point2d[0]/point2d[2], point2d[1]/point2d[2]])
print(finalpoint2D)

center_coordinates = (int(finalpoint2D[0]), int(finalpoint2D[1]))

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
    flipped_frame = cv2.circle(flipped_frame, (0, 480) , radius, (0, 255, 0), -1)
    flipped_frame = cv2.circle(flipped_frame, (0, 0) , radius, (255, 0, 0), -1)

    

    
    cv2.imshow('Frame', flipped_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()

