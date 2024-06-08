# -*- coding: utf-8 -*-
import cv2
import numpy as np
import math

# Defino variables globales:
FX = 497.66
FY = 502.16
CX = 325.3
CY = 240.18

class Punto2D:
    def __init__(self, x=0, y=0, d=0):
        self.x = x
        self.y = y
        self.d = d

class Punto3D:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

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

# Using bilinear interpolation
def getdistance(xi, yi):
    fx1y1 = 65 #115 # mm
    fx1y2 = 403 #450 # mm
    fx2y1 = 65 #115 # mm
    fx2y2 = 403 #445 # mm
    x1 = 0
    x2 = 640
    y1 = 480
    y2 = 0
    
    part1 = ((yi - y2) / (y1 - y2)) * (((xi - x2) / (x1 - x2)) * fx1y1 + ((xi - x1) / (x2 - x1)) * fx2y1)
    part2 = ((yi - y1) / (y2 - y1)) * (((xi - x2) / (x1 - x2)) * fx1y2 + ((xi - x1) / (x2 - x1)) * fx2y2)
    
    return part1 + part2

# Obtenemos el pixel de la imagen
def from2Dto3D(pixel):
    res = Punto3D()
    res.x = ((pixel.x - CX) * pixel.d) / FX
    res.y = ((pixel.y - CY) * pixel.d) / FY
    res.z = pixel.d
    return res

def getPoints(frame):
    pixel = Punto2D()
    lower_color = np.array([150, 50, 50])
    upper_color = np.array([170, 255, 255])
	
    centroid_x, centroid_y = detect_color(frame, lower_color, upper_color)

    if centroid_x is not None and centroid_y is not None:
        cv2.circle(frame, (centroid_x, centroid_y), 5, (0, 255, 0), -1)
        cv2.putText(frame, f"Centroide: ({centroid_x}, {centroid_y})", (centroid_x - 100, centroid_y - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
        pixel.x = centroid_x
        pixel.y = centroid_y
        pixel.d = getdistance(centroid_x, centroid_y)

        pixel3D = from2Dto3D(pixel)
        return pixel3D
    return None

def camera2world(cameraPoint):
    theta = np.deg2rad(15)
    height = 95.0
    
    R = np.array([
        [math.cos(theta), 0.0, math.sin(theta)],
        [0.0, 1.0, 0.0],
        [-math.sin(theta), 0.0, math.cos(theta)]])
    
    
    t = np.array([0.0, height, 0.0]).reshape(3, 1)

    cameraCoords = np.array([cameraPoint.x, cameraPoint.y, cameraPoint.z]).reshape(3, 1)
    worldCoords = np.dot(R, cameraCoords) + t
    return Punto3D(worldCoords[0, 0], worldCoords[1, 0], worldCoords[2, 0])

def undistortImage(image):
    
      # media de la matriz de distorsión 
    dist = np.array([[0.1898060854, -0.4413072785999999,
                      0.0021029783179999997, -0.0011012700979, 0.28083406670000005]])    
    mtx = np.array([[FX, 0.0, CX],
           [0.0, FY, CY],
           [0.0, 0.0, 1.0]])
           
    undistorted_img = cv2.undistort(image, mtx, dist)
    return undistorted_img
  
if __name__ == "__main__":
    
    cv2.namedWindow("Image Feed")
    # Mueve la ventana a una posición en concreto de la pantalla
    cv2.moveWindow("Image Feed", 159, -25)

    # Inicializa la cámara 
    cap = cv2.VideoCapture(0)
    
    # Setup camera: para agilizar el cómputo
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
    cap.set(cv2.CAP_PROP_FPS,40)
		
    while True:
        # Lee un frame de la cámara 
        ret, frame = cap.read()
        if not ret:
            break
        
        # Quitar la distorsión de la imagen de la cámara (si fuera necesario)
        newframe = undistortImage(frame)

        point3Dcamera = getPoints(newframe)
            
        if point3Dcamera:
            # Conversión de las coordenadas de la cámara al mundo real
            point3Dworld = camera2world(point3Dcamera)
        
            # Imprimir punto en 3D respecto al mundo
            print(f"Coordenadas 3D: X={point3Dworld.x}, Y={point3Dworld.y}, Z={point3Dworld.z}")
        
        cv2.imshow('Frame', newframe)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
