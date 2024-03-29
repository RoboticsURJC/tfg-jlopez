# -*- coding: utf-8 -*-
#from progeo import *
import cv2
import numpy as np
import math

# defino variables globales:
ANCHO_IMAGEN = 640
LARGO_IMAGEN = 480
FX = 333.76
FY = 335.08
CX = 310.99
CY = 230.93

class Camera:
    def __init__(self, k11, k12, k22, k23, k13, rt11, rt12, rt13, rt21, rt22, rt23, rt31, rt32, rt33, position):
        self.k11 = k11
        self.k12 = k12
        self.k22 = k22
        self.k23 = k23
        self.k13 = k13
        self.rt11 = rt11
        self.rt12 = rt12
        self.rt13 = rt13
        self.rt21 = rt21
        self.rt22 = rt22
        self.rt23 = rt23
        self.rt31 = rt31
        self.rt32 = rt32
        self.rt33 = rt33
        self.position = position

class Punto2D:
    def __init__(self, x=0, y=0, h=1):
        self.x = x
        self.y = y
        self.h = h
  
class Punto3D:
    def __init__(self, x=0, y=0,z=0, h=1):
        self.x = x
        self.y = y
        self.z = z
        self.h = h
    
def getradian(degrees):

    return ((degrees*math.pi)/180)

def backproject (punto2D, camera):
	output = -1
	temp2D = Punto2D()
	punto3D = Punto3D()

	if (punto2D.h>0.):
		temp2D.h=camera.k11
		temp2D.x=punto2D.x*camera.k11/punto2D.h
		temp2D.y=punto2D.y*camera.k11/punto2D.h

		ik11=(1./camera.k11)
		ik12=-camera.k12/(camera.k11*camera.k22)
		ik13=(camera.k12*camera.k23-camera.k13*camera.k22)/(camera.k22*camera.k11)
		ik21=0.
		ik22=(1./camera.k22)
		ik23=-(camera.k23/camera.k22)
		ik31=0.
		ik32=0.
		ik33=1.

		a1=ik11*temp2D.x+ik12*temp2D.y+ik13*temp2D.h
		a2=ik21*temp2D.x+ik22*temp2D.y+ik23*temp2D.h
		a3=ik31*temp2D.x+ik32*temp2D.y+ik33*temp2D.h
		a4=1.

		ir11=camera.rt11
		ir12=camera.rt21
		ir13=camera.rt31
		ir14=0.
		ir21=camera.rt12
		ir22=camera.rt22
		ir23=camera.rt32
		ir24=0.
		ir31=camera.rt13
		ir32=camera.rt23
		ir33=camera.rt33
		ir34=0.
		ir41=0.
		ir42=0.
		ir43=0.
		ir44=1.

		b1=ir11*a1+ir12*a2+ir13*a3+ir14*a4
		b2=ir21*a1+ir22*a2+ir23*a3+ir24*a4
		b3=ir31*a1+ir32*a2+ir33*a3+ir34*a4
		b4=ir41*a1+ir42*a2+ir43*a3+ir44*a4 

		it11=1.
		it12=0.
		it13=0.
		it14=camera.position[0]
		it21=0.
		it22=1.
		it23=0.
		it24=camera.position[1]
		it31=0.
		it32=0.
		it33=1.
		it34=camera.position[2]
		it41=0.
		it42=0.
		it43=0.
		it44=1.

		punto3D.x=it11*b1+it12*b2+it13*b3+it14*b4
		punto3D.y=it21*b1+it22*b2+it23*b3+it24*b4
		punto3D.z=it31*b1+it32*b2+it33*b3+it34*b4
		punto3D.h=it41*b1+it42*b2+it43*b3+it44*b4

		if (punto3D.h!=0.):
			punto3D.x=punto3D.x/punto3D.h
			punto3D.y=punto3D.y/punto3D.h
			punto3D.z=punto3D.z/punto3D.h
			punto3D.h=1.
			output=1
		else:
			output=0

	return(output, punto3D)


def pixel2optical(p2d):
    aux = p2d.x
    p2d.x = LARGO_IMAGEN - 1 - p2d.y
    p2d.y = aux
    p2d.h = 1
    return p2d

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
    
    
    #K = np.array([[FX, 0.0, CX],
    #              [0.0, FY, CY],
    #              [0.0, 0.0, 1.0]])

    rad = getradian(50)
    
    #RT = np.array([[np.cos(rad), 0.0, np.sin(rad),     0.0],
    #            [    0.0 , 1.0,       0.0,     0.0],
    #            [-np.sin(rad), 0, np.cos(rad), 110.0]])
    
    k11 = FX
    k12 = 0.0
    k22 = FY
    k23 = CY 
    k13 = CX
    
    rt11 = np.cos(rad)
    rt12 = 0.0
    rt13 = -np.sin(rad)
    
    rt21 = 0.0
    rt22 = 1.0
    rt23 = 0.0
    
    rt31 = np.sin(rad)
    rt32 = 0.0
    rt33 = np.cos(rad)
    
    posicion_camara = np.array([0.0, 0.0, 110.0])

    # Crear una instancia de la cámara con los parámetros adecuados
    mi_camara = Camera(k11, k12, k22, k23, k13, rt11, rt12, rt13, rt21, rt22, rt23, rt31, rt32, rt33, posicion_camara)
    
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
            
            # Crear un punto en 2D
            punto_2d = Punto2D(centroid_x, centroid_y, 1)
    
            # Convertir el punto en 2D a coordenadas ópticas
            punto_2d_optico = pixel2optical(punto_2d)

            # Llamar a la función backproject
            output, punto_3d = backproject(punto_2d_optico, mi_camara)

            # Verificar si la proyección fue exitosa
            if output == 1:
                # Acceder a las coordenadas en 3D desde el objeto Punto3D
                x_3d = punto_3d.x
                y_3d = punto_3d.y
                z_3d = punto_3d.z
                
                            #cv2.circle(flipped_frame, (optic_x, optic_y), 5, (255, 0, 0), -1)
                print(f"Coordenadas 3D: X={x_3d}, Y={y_3d}, Z={z_3d}")

                
            #else:
                #print("La proyección no fue exitosa.")
            
            # Convertir coordenadas de píxeles a ópticas
            #optic_x, optic_y = pixel2optical(centroid_x, centroid_y)
            #cv2.circle(flipped_frame, (optic_x, optic_y), 5, (255, 0, 0), -1)

            #print(f"Coordenadas pixeles: X={centroid_x}, Y={centroid_y}")
            #print(f"Coordenadas ópticas: X={optic_x}, Y={optic_y}")
            
            #point3d = backpropagation(optic_x, optic_y)
            #print(point3d)

        cv2.imshow('Frame', flipped_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


