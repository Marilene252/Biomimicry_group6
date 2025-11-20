#DC motor with Breadboard

import time
import numpy as np
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)
motorSpeed = GPIO.PWM(27, 1000)
motorSpeed.start(0)

def forward(speed):
    GPIO.output(17, True)
    GPIO.output(18, False)
motorSpeed.ChangeDutyCycle(speed)

def backward(speed):
    GPIO.output(17, False)
    GPIO.output(18, True)
    motorSpeed.ChangeDutyCycle(0)

def stop(speed):
    GPIO.output(17, False)
    GPIO.output(18, False)
    motorSpeed.ChangeDutyCycle(0)

try:
    forward(50)
    stop()
    backward(75)
    time.sleep(3)

finally:
    motorSpeed.stop()
    GPIO.cleanup()