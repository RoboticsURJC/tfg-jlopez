# 2019 - Julio Vega
# IMPORTANTE: antes de lanzarlo hay que ejecutar el demonio de pigpio con "sudo pigpiod"
# Este programa sigue una persona que se encuentre dentro de la imagen percibida por PiCamera
#import imutils
import cv2
import argparse
import sys, traceback, Ice
import easyiceconfig as EasyIce
import jderobot
import numpy as np
import threading
import sys, tty, termios, time, pigpio

servos = [4,18] # Usamos los servos conectados a los pines 4 y 24
dit = pigpio.pi()

def motor1_forward():
    dit.set_servo_pulsewidth(servos[0], 1600)

def motor1_reverse():
    dit.set_servo_pulsewidth(servos[0], 1300)

def motor1_stop():
    dit.set_servo_pulsewidth(servos[0], 1525)

def motor2_forward():
    dit.set_servo_pulsewidth(servos[1], 1400)

def motor2_reverse():
    dit.set_servo_pulsewidth(servos[1], 1700)

def motor2_stop():
    dit.set_servo_pulsewidth(servos[1], 1510)

def forward (tiempo):
	print "Avanzando"
        motor1_forward()
        motor2_forward()
        time.sleep(tiempo)

def reverse (tiempo):
	print "Retrocediendo"
        motor1_reverse()
        motor2_reverse()
        time.sleep(tiempo)

def turn_left (tiempo):
	print "Avanzando izquierda"
	dit.set_servo_pulsewidth(servos[0], 1540)
	dit.set_servo_pulsewidth(servos[1], 1490)
        time.sleep(tiempo)

def turn_right (tiempo):
	print "Avanzando derecha"
	dit.set_servo_pulsewidth(servos[0], 1550)
	dit.set_servo_pulsewidth(servos[1], 1500)
        time.sleep(tiempo)

def stop (tiempo):
        print("DETENIENDO")
        motor1_stop()
        motor2_stop()
        time.sleep(tiempo)
        #for s in servos: # stop servo pulses
	#	dit.set_servo_pulsewidth(s, 0)

        #dit.stop()

def seguirPersona (image, color):
    # resize the frame
    image = imutils.resize(image, width=600)

    # Convert to grayscale
    gray = cv2.cvtColor(imgage, cv2.COLOR_BGR2GRAY)

    # Función Haar para detectar caras
    faces = face_cascade.detectMultiScale(gray, 1.1, 4)

    # Dibujamos un rectángulo alrededor de la cara detectada
    origen = (x, y)
    extremo = (x+w, y+h)
    area = w*h
    cv2.rectangle(image, origen, extremo, (255, 0, 0), 2)

    center = (int((x+w)/2), int((y+h)/2))
    print 'x: ' + str(int(center.x)) + ' y: ' + str(int(center.y)) + ' area: ' + str(area)
    moverPiBot (center.x,center.y,round(area, -5))
    time.sleep(0.15)

    # show the image to our screen
    cv2.imshow("Imagen en curso", image)
    # key = cv2.waitKey(1) & 0xFF
    if cv2.waitKey(1) & 0xFF == ord('q'):
        stop (1); # paramos los motores
        for s in servos: # stop servo pulses
	      	dit.set_servo_pulsewidth(s, 0)

        dit.stop()
        clean_up()
        sys.exit()

    return center

def moverPiBot (x,y,area):
    if x > 330:
       turn_right((x-320)/(320*12))
    elif x < 310:
       turn_left((320-x)/(320*12))
    elif ((310 <= x <= 330) and area<1500000):
       forward(0.05)
    elif ((310 <= x <= 330) and area>4000000):
       reverse(0.05)

if __name__ == "__main__":
    
    #ic = EasyIce.initialize(sys.argv)
    #properties = ic.getProperties()
    #basecameraL = ic.propertyToProxy("Camera.Proxy")
    #cameraProxy = jderobot.CameraPrx.checkedCast(basecameraL)
    #key=-1

    while key != 1048689:
        forward(1000)
        # grab the current image
        #imageData = cameraProxy.getImageData("RGB8")
        #imageData_h = imageData.description.height
        #imageData_w = imageData.description.width
        #image = np.zeros((imageData_h, imageData_w, 3), np.uint8)
        #image = np.frombuffer(imageData.pixelData, dtype=np.uint8)
        #image.shape = imageData_h, imageData_w, 3
        #image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
	    #rotated = imutils.rotate_bound(image, 180) # Because we have inverted piCamera on PiBot

        #center = seguirPersona (rotated, color)

        #if center != None:
        #    print center
        #else:
        #    print "NO HAY PERSONA"

    # cleanup the camera and close any open windows
    #camera.release()
    cv2.destroyAllWindows()
