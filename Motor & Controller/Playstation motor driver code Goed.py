from pyPS4Controller.controller import Controller
import RPi.GPIO as GPIO
import time

IN1 = 20
IN2 = 21
ENA = 26   # PWM pin for motor 1

IN3 = 6
IN4 = 13
ENB = 12   # PWM pin for motor 2

GPIO.setmode(GPIO.BCM)

# Motor 1 pins
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

# Motor 2 pins
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# PWM setup
pwm_ena = GPIO.PWM(ENA, 1000)
pwm_enb = GPIO.PWM(ENB, 1000)

pwm_ena.start(0)
pwm_enb.start(0)

def set_motor_left(speed):
    """
    Stuurt linker motoren aan
    speed: waarde tussen -100 en 100
    """
    if speed > 0: 
        #Vooruit
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        pwm_ena.ChangeDutyCycle(speed)
    elif speed < 0:
        #Achteruit
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        pwm_ena.ChangeDutyCycle (abs(speed))
    else:
        #Stop
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        pwm_ena.ChangeDutyCycle(0)

def set_motor_right(speed):
    """
    Stuurt rechter motoren aan
    speed: waarde tussen -100 en 100
    """
    if speed > 0: 
        #Vooruit
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ena.ChangeDutyCycle(speed)
    elif speed < 0:
        #Achteruit
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        pwm_ena.ChangeDutyCycle (abs(speed))
    else:
        #Stop
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ena.ChangeDutyCycle(0)

class MyController(Controller):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        #We houden de stand van gas en sturen bij
        self.throttle = 0
        self.steering = 0

    def update_motors(self):
        """
        Berekent de snelheid voor links en rechts op basis van throttle en steering.
        """
        #Links = Gas + Stuur
        #Rechts = Gas - Stuur
        left_speed = self.throttle + self.steering
        right_speed = self.throttle - self.steering

        #Waarden niet boven 100 of onder -100
        left_speed = max(min(left_speed, 100), -100)
        right_speed = max(min(right_speed, 100), -100)

        #Stuur de motoren aan
        set_motor_left(left_speed)
        set_motor_right(right_speed)
        
    def on_L3_up(self, value):
        #Value is -32767 (vol omhoog) tot 0. Omgezet tot 0-100
        self.throttle = int(abs(value) / 32767 * 100)
        self.update_motors()

    def on_L3_down(self, value):
        #Voor deze value geldt hetzelfde als bij on_L3_up
        self.throttle = -int(value / 32767 * 100)
        self.update_motors()

    def on_L3_y_at_rest(self):
        #Joystick lostaten (geen gas)
        self.throttle = 0
        self.update_motors()

    #L3 Links / Rechts (Sturen)

    def on_L3_left(self, value):
        #Naar links sturen: Links langzamer, rechts sneller
        #Negatieve waarde geconverteerd naar -100 tot 0
        steering_val = int(abs(value) / 32767 * 100)
        self.steering = -steering_val #Links is negatieve waarde
        self.update_motors()

    def on_L3_right(self, value):
        #Naar rechts sturen
        steering_val = int(value / 32767 * 100)
        self.steering = steering_val
        self.update_motors()

    def on_L3_x_at_rest(self):
        #Joystick losgelaten (Niet sturen)
        self.steering = 0
        self.update_motors()

    def on_L3_press(self):
        #Noodstop als de stick indrukt
        self.throttle = 0
        self.steering = 0
        self.update_motors()

try:
    controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    print("Controller verbonden. Gebruik L3 om te rijden.")
    controller.listen(timeout=60)

except KeyboardInterrupt:
    print("Onderbroken door gebruiker.")

finally:
    pwm_ena.stop()
    pwm_enb.stop()
    GPIO.cleanup()

