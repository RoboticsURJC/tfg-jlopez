# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

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
pwm_left.stop()
pwm_right.stop()

