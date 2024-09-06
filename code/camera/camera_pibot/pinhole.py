###############################################################################
# (C) 2019 - Julio Vega
###############################################################################
# Algoritmo de navegación de PiBot basado en sonar visual percibido por PiCam.#
# En este programa, los motores están parados, para así poder hacer pruebas.  #
###############################################################################

# -*- coding: utf-8 -*-
from projGeom import *
import cv2
import numpy 


ANCHO_IMAGEN = 640
LARGO_IMAGEN = 480
FX = 497.66
FY = 502.16
CX = 325.3
CY = 240.18

DEGTORAD = 3.1415926535897932 / 180
myCamera = None


def loadCamera ():
	global myCamera
	myCamera = PinholeCamera()
	thetaY = 50*DEGTORAD # considerando que la camara (en vertical) está rotada 90º sobre eje Y
	thetaZ = 0*DEGTORAD # considerando que la camara (en vertical) está rotada 90º sobre eje Y
	thetaX = 0*DEGTORAD # considerando que la camara (en vertical) está rotada 90º sobre eje Y

	R_y = numpy.array ([(numpy.cos(thetaY),0,-numpy.sin(thetaY)),(0,1,0),(numpy.sin(thetaY),0,numpy.cos(thetaY))]) # R is a 3x3 rotation matrix
	R_z = numpy.array ([(numpy.cos(thetaZ),-numpy.sin(thetaZ),0),(numpy.sin(thetaZ),numpy.cos(thetaZ),0),(0,0,1)]) # R is a 3x3 rotation matrix
	R_x = numpy.array ([(1,0,0),(0,numpy.cos(thetaX),numpy.sin(thetaX)),(0, -numpy.sin(thetaX),numpy.cos(thetaX))]) # R is a 3x3 rotation matrix

	R_subt = numpy.dot (R_y, R_z)
	R_tot = numpy.dot (R_subt, R_x)

	T = numpy.array ([(1,0,0,0),(0,1,0,0),(0,0,1,-110)]) # T is a 3x4 traslation matrix
	Res = numpy.dot (R_tot,T)
	RT = numpy.append(Res, [[0,0,0,1]], axis=0) # RT is a 4x4 matrix
	K = numpy.array ([(FX,0,CX,0),(0,FY,CY,0),(0,0,1,0)]) # K is a 3x4 matrix
	# -------------------------------------------------------------

	# -------------------------------------------------------------
	# LOADING BOTH CAMERA MODELS JUST TO TEST THEM
	# -------------------------------------------------------------
	# A) PROGEO CAMERA
	# -------------------------------------------------------------
	myCamera.position.x = 0
	myCamera.position.y = 0
	myCamera.position.z = -88
	myCamera.position.h = 1

	# K intrinsec parameters matrix (values got from the PiCamCalibration.py)
	myCamera.k11 = K[0,0]
	myCamera.k12 = K[0,1]
	myCamera.k13 = K[0,2]
	myCamera.k14 = K[0,3]

	myCamera.k21 = K[1,0]
	myCamera.k22 = K[1,1]
	myCamera.k23 = K[1,2]
	myCamera.k24 = K[1,3]

	myCamera.k31 = K[2,0]
	myCamera.k32 = K[2,1]
	myCamera.k33 = K[2,2]
	myCamera.k34 = K[2,3]

	# RT rotation-traslation matrix
	myCamera.rt11 = RT[0,0]
	myCamera.rt12 = RT[0,1]
	myCamera.rt13 = RT[0,2]
	myCamera.rt14 = RT[0,3]

	myCamera.rt21 = RT[1,0]
	myCamera.rt22 = RT[1,1]
	myCamera.rt23 = RT[1,2]
	myCamera.rt24 = RT[1,3]

	myCamera.rt31 = RT[2,0]
	myCamera.rt32 = RT[2,1]
	myCamera.rt33 = RT[2,2]
	myCamera.rt34 = RT[2,3]

	myCamera.rt41 = RT[3,0]
	myCamera.rt42 = RT[3,1]
	myCamera.rt43 = RT[3,2]
	myCamera.rt44 = RT[3,3]

	myCamera.fdistx = K[0,0] #myCamera.k11
	myCamera.fdisty = K[1,1] #myCamera.k22
	myCamera.u0 = K[0,2] #myCamera.k13
	myCamera.v0 = K[1,2] #myCamera.k23
	myCamera.rows = LARGO_IMAGEN
	myCamera.columns = ANCHO_IMAGEN

	#myCamera.printCameraInfo ()
	# -------------------------------------------------------------
	'''
	# -------------------------------------------------------------
	# B) OPENCV CAMERA
	# -------------------------------------------------------------
	cameraCV = PinholeCamera_CV ()
	campositionCV = [(myCamera.position.x, myCamera.position.y, myCamera.position.z)]
	D = [(0.04853663), (-0.19172296), (-0.00088241), (-0.00245449), (-0.2196874)]
	K_CV = numpy.array ([(313.89382026,0,158.04145907),(0,316.64906146,117.5728043),(0,0,1)]) # K_CV is a 3x3 matrix
	cameraCV.setPinHoleCamera(K_CV, D, R, T, ANCHO_IMAGEN, LARGO_IMAGEN, campositionCV)

	cameraCV.printCameraInfo ()
	'''

def pixel2optical (p2d):
	aux = p2d.x
	p2d.x = LARGO_IMAGEN-1-p2d.y
	p2d.y = aux
	p2d.h = 1

	return p2d

def getIntersectionZ (p2d):
	p3d = Punto3D ()
	res = Punto3D ()
	p2d_ = Punto2D ()

	x = myCamera.position.x
	y = myCamera.position.y
	z = myCamera.position.z

	p2d_ = pixel2optical(p2d)
	result, p3d = backproject(p2d_, myCamera)

	# Check division by zero
	if((p3d.z-z) == 0.0):
		res.h = 0.0
		return

	zfinal = 0. # Quiero que intersecte con el Plano Z = 0

	# Linear equation (X-x)/(p3d.X-x) = (Y-y)/(p3d.Y-y) = (Z-z)/(p3d.Z-z)
	xfinal = x + (p3d.x - x)*(zfinal - z)/(p3d.z-z)
	yfinal = y + (p3d.y - y)*(zfinal - z)/(p3d.z-z)	

	res.x = xfinal
	res.y = yfinal
	res.z = zfinal
	res.h = 1.0

	return res

def calcular_distancia_3d(x_cam, y_cam, z_cam, x_punto, y_punto, z_punto):
    distancia = numpy.sqrt((x_punto - x_cam)**2 + (y_punto - y_cam)**2 + (z_punto - z_cam)**2)
    return distancia


def detect_color(frame, lower_color, upper_color):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    # Esto  hasido modificado 
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M["m00"] != 0:
            centroid_x = int(M["m10"] / M["m00"])
            centroid_y = int(M["m01"] / M["m00"])
            return centroid_x, centroid_y
    return None, None


# obtenemos el pixel de la imagen
def getPoints (frame):

    #global fronteraImg
    #global puntosFrontera

    pixel = Punto2D()
    pixelOnGround3D = Punto3D()

    lower_color = numpy.array([150, 50, 50])
    upper_color = numpy.array([170, 255, 255])
	
    centroid_x, centroid_y = detect_color(frame, lower_color, upper_color)


    if centroid_x is not None and centroid_y is not None:
        cv2.circle(frame, (centroid_x, centroid_y), 5, (0, 255, 0), -1)
        cv2.putText(frame, f"Centroide: ({centroid_x}, {centroid_y})", (centroid_x - 100, centroid_y - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
        pixel.x = centroid_x
        pixel.y = centroid_y
        pixel.h = 1

        pixelOnGround3D = getIntersectionZ (pixel)

        print(f"Coordenadas 3D: X={pixelOnGround3D.x}, Y={pixelOnGround3D.y}, Z={pixelOnGround3D.z}")

if __name__=="__main__":
	
    cv2.namedWindow("Image Feed")
    # Mueve la ventana a una posición en concreto de la pantalla
    cv2.moveWindow("Image Feed", 159, -25)

    # Inicializa la cámara 
    cap = cv2.VideoCapture(0)
	
    
    loadCamera()
	
    while True:
		
        # Lee un frame de la cámara 
        ret,frame = cap.read() 
    
        # Gira la cámara 180º porque la cámara está físicamente dada la vuelta 
        #flipped_frame = cv2.flip(frame,0)
        #flipped_frame = cv2.flip(flipped_frame,1)

	
        getPoints(frame)
        
        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
