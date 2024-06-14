import pigpio
import time

# Use pin 7 (GPIO 4) for PWM signal for the left servo
pwm_gpio_left = 4
# Use pin 12 (GPIO 18) for PWM signal for the right servo
pwm_gpio_right = 18

def set_servo_angle(pi, gpio, angle):
    # Calculate the duty cycle for the given angle
    duty_cycle = int(12.346 * angle ** 2 + 7777.8 * angle + 700000)
    # Convert the duty cycle to the range used by pigpio (500 to 2500 microseconds)
    pulse_width = duty_cycle / 1000
    pi.set_servo_pulsewidth(gpio, pulse_width)

def main():
    # Initialize pigpio
    angle = 0
    pi = pigpio.pi()

    if not pi.connected:
        exit(0)

    try:
        #while True:
                # Set angle for left servo
        set_servo_angle(pi, pwm_gpio_left, angle)
                # Set angle for right servo (mirrored movement)
        set_servo_angle(pi, pwm_gpio_right, 180 - angle)
        time.sleep(0.5)
    finally:
        # Stop the servos and clean up
        pi.set_servo_pulsewidth(pwm_gpio_left, 0)
        pi.set_servo_pulsewidth(pwm_gpio_right, 0)
        pi.stop()

if __name__ == '__main__':
    main()