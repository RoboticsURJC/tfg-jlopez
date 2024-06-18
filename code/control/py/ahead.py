import RPi.GPIO as GPIO
import time

def angle2dutycycle(angle):
    x1 = 180
    y1 = 12
    x2 = 0
    y2 = 2
    c = y2
    dutycycle = ((y1 - y2) / (x1 - x2)) * angle + c
    return dutycycle

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

pwm_gpio_left = 7
pwm_gpio_right = 12
frequence = 50

GPIO.setup(pwm_gpio_left, GPIO.OUT)
pwm_left = GPIO.PWM(pwm_gpio_left, frequence)
GPIO.setup(pwm_gpio_right, GPIO.OUT)
pwm_right = GPIO.PWM(pwm_gpio_right, frequence)

duty_cycle_left = angle2dutycycle(180)
duty_cycle_right = angle2dutycycle(0)

try:
    pwm_left.start(duty_cycle_left)
    pwm_right.start(duty_cycle_right)
    time.sleep(0.15)
    pwm_left.stop()
    pwm_right.stop()
    GPIO.cleanup()
    with open('/var/www/html/control/log/python_debug.log', 'a') as f:
        f.write('Adelante: Ejecutado correctamente\n')
except Exception as e:
    with open('/var/www/html/control/log/python_debug.log', 'a') as f:
        f.write(f'Adelante: Error - {str(e)}\n')
