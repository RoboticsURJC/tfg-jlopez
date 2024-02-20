
#  gira 180 grados hacia delante y detrás 
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(7,GPIO.OUT)

servo = GPIO.PWM(7,10)

servo.start(0)
print("Waiting for 1 second")
time.sleep(1)

print("Rotating at intervals of 10 degrees")
duty = 2

while duty <= 12:
    servo.ChangeDutyCycle(duty)
    time.sleep(1)
    duty = duty + 1

print("Turning back to 0 degrees")
servo.ChangeDutyCycle(2)
time.sleep(1)
servo.ChangeDutyCycle(0)

servo.stop()
GPIO.cleanup()
print("Everything's cleaned up")
