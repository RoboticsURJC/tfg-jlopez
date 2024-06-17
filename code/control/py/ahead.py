# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time

## ESQUEMA PARA LA RELACIÓN ÁNGULO/MOVIMIENTO DE LOS SERVOS:
### ADELANTE ->      M_izq = 180º, M_Der = 0º
### GIRO DERECHA->   M_izq = 180º, M_Der = 90º
### GIRO IZQUIERDA-> M_izq = 90º, M_Der = 180º
### ATRÁS->          M_izq = 0º, M_Der = 180º

## 12% -> 180º
##  2% ->   0º

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False) 

#Usa pin 4 
pwm_gpio_left = 7
#Usa pin 18 
pwm_gpio_right = 12
frequence = 50

GPIO.setup(pwm_gpio_left, GPIO.OUT)
pwm_left = GPIO.PWM(pwm_gpio_left, frequence)
GPIO.setup(pwm_gpio_right, GPIO.OUT)
pwm_right = GPIO.PWM(pwm_gpio_right, frequence)

#Va hacia adelante (180º, 0º)
pwm_left.start(12)
pwm_right.start(2)
time.sleep(0.15)
