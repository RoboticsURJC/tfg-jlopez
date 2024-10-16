#!/usr/bin/env python3
#-- coding: utf-8 --
import RPi.GPIO as GPIO
import time

#Set function to calculate percent from angle
def angle_to_percent (angle) :
    if angle > 180 or angle < 0 :
        return False

    start = 4
    end = 12.5
    ratio = (end - start)/180 #Calcul ratio from angle to percent

    angle_as_percent = angle * ratio

    return start + angle_as_percent

GPIO.setmode(GPIO.BOARD) #Use Board numerotation mode
GPIO.setwarnings(False) #Disable warnings

#Use pin 4 for PWM signal
pwm_gpio_left = 7
#Use pin 18 for PWM signal
pwm_gpio_right = 12
frequence = 50

GPIO.setup(pwm_gpio_left, GPIO.OUT)
pwm_left = GPIO.PWM(pwm_gpio_left, frequence)
GPIO.setup(pwm_gpio_right, GPIO.OUT)
pwm_right = GPIO.PWM(pwm_gpio_right, frequence)

#Go ahead
pwm_left.start(angle_to_percent(180))
pwm_right.start(angle_to_percent(0))
time.sleep(1)

#Go back
pwm_left.start(angle_to_percent(0))
pwm_right.start(angle_to_percent(180))
time.sleep(1)

#Go ahead to the left
pwm_left.start(angle_to_percent(180))
pwm_right.start(angle_to_percent(45))
time.sleep(1)

#Go ahead to the right
pwm_left.start(angle_to_percent(135))
pwm_right.start(angle_to_percent(0))
time.sleep(1)

#Close GPIO & cleanup
pwm_left.stop()
pwm_right.stop()
GPIO.cleanup()