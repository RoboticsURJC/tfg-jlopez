import RPi.GPIO as GPIO
import time


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
pwm_left.start(7)
pwm_right.start(2)
time.sleep(0.25)