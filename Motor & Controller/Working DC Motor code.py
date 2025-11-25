import RPi.GPIO as GPIO
import time

#pin mapping
IN1 = 20 #Direction forward
IN2 = 21 #Direction backward
ENA = 26 #PWM (speed control)

GPIO.setmode(GPIO.BCM)

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

#Set up PWM for speed control
pwm = GPIO.PWM(ENA, 1000)
pwm.start(0)

def motor1_forward(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(speed)

def motor1_backward(speed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm.ChangeDutyCycle(speed)

def motor1_brake():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(0)

#Test
try:
    while True:
        motor1_brake()
        time.sleep(0.1)

        motor1_forward(50)
        time.sleep(1)

        motor1_brake()
        time.sleep(0.1)

        motor1_backward(50)
        time.sleep(1)

except KeyboardInterrupt:
    print ("Program stopped")

finally:
    pwm.stop()
    GPIO.cleanup()
    print("GPIO opgeruimd")


