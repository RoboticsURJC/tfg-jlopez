import pigpio
import time

## ESQUEMA PARA LA RELACIÓN ÁNGULO/MOVIMIENTO DE LOS SERVOS:
### ADELANTE ->      M_izq = 180º, M_Der = 0º
### GIRO DERECHA->   M_izq = 180º, M_Der = 90º
### GIRO IZQUIERDA-> M_izq = 90º, M_Der = 180º
### ATRÁS->          M_izq = 0º, M_Der = 180º

# Usa pin 7 (GPIO 4) para el motor izquiero
pwm_gpio_left = 4
# Usa pin 12 (GPIO 18) para el motor derecho
pwm_gpio_right = 18

def set_servo_angle(pi, gpio, angle):
    # Calcula el ciclo de trabajo dado el ángulo
    duty_cycle = int(12.346 * angle ** 2 + 7777.8 * angle + 700000)
    # Convierte el ciclo de trabajo al rango usado por pigpiod (500 to 2500 microsegundos)
    pulse_width = duty_cycle / 1000
    pi.set_servo_pulsewidth(gpio, pulse_width)

def main():
    # Inicializo pigpiod
    angle = 0
    pi = pigpio.pi()

    if not pi.connected:
        exit(0)

    try:
        # Ángulo para el motor izquierdo
        set_servo_angle(pi, pwm_gpio_left, angle)
        # Ángulo para el motor derecho
        set_servo_angle(pi, pwm_gpio_right, 180 - angle)
        time.sleep(0.15)
    finally:
        # Para los servos
        pi.set_servo_pulsewidth(pwm_gpio_left, 0)
        pi.set_servo_pulsewidth(pwm_gpio_right, 0)
        pi.stop()

if __name__ == '__main__':
    main()

