#Python Robotic DC motors

import RPi.GPIO as GPIO
import time

#Pin setup
IN1 = x
IN2 = x
ENA = x

IN3 = x
IN4 = x
ENB = x

#Setup GPIO mode
GPIO.setmode(GPIO.BCM)

#Setup pins as output
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Create PWM objects for ENA and ENB for speed control
pwm_ena = GPIO.PWM(ENA, 1000) # frequency is set to 1kHz
pwm_enb = GPIO.PWM(ENB, 1000)

pwm_ena.start(0) # Start PWM with 0% duty cycle
pwm_enb.start(0)

def motor1_forward(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm_ena.ChangeDutyCycle(speed) #Control motor speed with PWM

def motor1_backward(speed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm_ena.ChangeDutyCycle(speed)

def motor1_brake():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

#Voor 2de motor, kan ik de bovenste 3 defs kopiëren

#Main loop
try:
    while True:
        motor1_brake()
        #motor2
        time.sleep(0.1)

        motor1_forward(50)
        #motor2
        time.sleep(1)
        
        motor1_brake()
        #motor2
        time.sleep(0.1)
        
        motor1_backward(50)
        #motor2
        time.sleep(1)

except KeyboardInterrupt:
    print("Program interrupted.")

finally:
    pwm_ena.stop()
    #motor 2 (pwm_enb) stop
    GPIO.cleanup() # Clean up GPIO settings
        
