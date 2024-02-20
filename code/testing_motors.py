# 2019 - Julio Vega
# 2024 - Modificado por Julia López Augusto
# IMPORTANTE: antes de lanzarlo hay que ejecutar el demonio de pigpio con "sudo pigpiod"
import argparse
import sys, traceback
import numpy as np
import threading
import sys, tty, termios, time, pigpio

servos = [4,18] # Usamos los servos conectados a los pines GPIO 4(der) y 18(izq)
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
    print ("Avanzando")
    motor1_forward()
    motor2_forward()
    time.sleep(tiempo)

def reverse (tiempo):
	print("Retrocediendo")
    motor1_reverse()
    motor2_reverse()
    time.sleep(tiempo)

def turn_left (tiempo):
	print("Avanzando izquierda")
	dit.set_servo_pulsewidth(servos[0], 1540) # derecha
	dit.set_servo_pulsewidth(servos[1], 1490) # izquierda
    time.sleep(tiempo)

def turn_right (tiempo):
	print "Avanzando derecha"
	dit.set_servo_pulsewidth(servos[0], 1550) # derecha
	dit.set_servo_pulsewidth(servos[1], 1500) # izquierda
    time.sleep(tiempo)

def stop (tiempo):
    print("DETENIENDO")
    motor1_stop()
    motor2_stop()
    time.sleep(tiempo)
    for s in servos: # stop servo pulses
	    dit.set_servo_pulsewidth(s, 0)
        dit.stop()

if __name__ == "__main__":
    
    forward(10)

