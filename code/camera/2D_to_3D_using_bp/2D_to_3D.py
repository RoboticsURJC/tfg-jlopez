# -*- coding: utf-8 -*-
import cv2
import numpy 

# defino variables globales:
FX = 497.66
FY = 502.16
CX = 325.3
CY = 240.18

class Punto2D:
	x = 0
	y = 0
	d = 0

class Punto3D:
	x = 0
	y = 0
	z = 0
	
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

# using bilinear interpolation  
def getdistance(xi, yi):
    
    fx1y1 = 11.5 # cm
    fx1y2 = 45 # cm
    fx2y1 = 11.5 # cm
    fx2y2 = 44.5 # cm
    x1 = 0
    x2 = 640
    y1 = 480
    y2 = 0
    
    part1 = ((yi-y2)/(y1-y2))*(((xi-x2)/(x1-x2))*fx1y1 + ((xi-x1)/(x2-x1))*fx2y1)
    part2 = ((yi-y1)/(y2-y1))*(((xi-x2)/(x1-x2))*fx1y2 + ((xi-x1)/(x2-x1))*fx2y2)
    
    return part1 + part2

# obtenemos el pixel de la imagen
def from2Dto3D(pixel):
    
    res = Punto3D()
    
    res.x = (((pixel.x - CX)*pixel.d)/FX)
    res.y = (((pixel.y - CY)*pixel.d)/FY)
    res.z = pixel.d
    
    return res
    
def getPoints (frame):

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
        pixel.d = getdistance(centroid_x, centroid_y)

        pixelOnGround3D = from2Dto3D(pixel)

        print(f"Coordenadas 3D: X={pixelOnGround3D.x}, Y={pixelOnGround3D.y}, Z={pixelOnGround3D.z}")

if __name__=="__main__":
	
    cv2.namedWindow("Image Feed")
    # Mueve la ventana a una posición en concreto de la pantalla
    cv2.moveWindow("Image Feed", 159, -25)

    # Inicializa la cámara 
    cap = cv2.VideoCapture(0)
		
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

