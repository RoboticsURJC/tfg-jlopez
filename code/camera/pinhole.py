
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
    
# matriz obtenida de los parámetros intrínsecos
K = np.array([[333.76, 0.0, 310.99],
                  [0.0, 335.08, 230.93],
                  [0.0, 0.0, 1.0]])

# la z siempre va a ser 0
point3d = np.array([1.0,0.0,0.0,1.0])

# 40 es el valor obtenido tras medir la inclinación cámara
point2d = get2Dpoint(K, point3d, 40)

print(point2d)
# el valor del eje z no es necesario y nos ayuda a poder escalar al
# valor que estamos buscando del eje x e y
finalpoint2D = np.array([ point2d[0]/point2d[2], point2d[1]/point2d[2]])
print(finalpoint2D)
