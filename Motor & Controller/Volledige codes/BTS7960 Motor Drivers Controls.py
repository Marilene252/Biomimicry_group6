from pyPS4Controller.controller import Controller
import RPi.GPIO as GPIO
import time

L_RPWM = 17
L_LPWM = 18   # PWM pin for motor 1

R_RPWM = 27
R_LPWM = 22   # PWM pin for motor 2

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(L_RPWM, GPIO.OUT)
GPIO.setup(L_LPWM, GPIO.OUT)
GPIO.setup(R_RPWM, GPIO.OUT)
GPIO.setup(R_LPWM, GPIO.OUT)

# PWM setup
pwm_left_fwd = GPIO.PWM(L_RPWM, 1000)
pwm_left_bwd = GPIO.PWM(L_LPWM, 1000)

pwm_right_fwd = GPIO.PWM(R_RPWM, 1000)
pwm_right_bwd = GPIO.PWM(R_LPWM, 1000)

#Start alles op 0
pwm_left_fwd.start(0)
pwm_left_bwd.start(0)
pwm_right_fwd.start(0)
pwm_right_bwd .start(0)

def set_motor_left(speed):
    """
    Stuurt linker BTS7960 aan
    speed: waarde tussen -100 en 100
    """
    if speed > 0: 
        #Vooruit
        pwm_left_fwd.ChangeDutyCycle(speed)
        pwm_left_bwd.ChangeDutyCycle(0)
    elif speed < 0:
        #Achteruit
        pwm_left_fwd.ChangeDutyCycle(0)
        pwm_left_bwd.ChangeDutyCycle(abs(speed))
    else:
        #Stop
        pwm_left_fwd.ChangeDutyCycle(0)
        pwm_left_bwd.ChangeDutyCycle(0)

def set_motor_right(speed):
    """
    Stuurt rechter BTS7960 driver aan
    speed: waarde tussen -100 en 100
    """
    if speed > 0: 
        #Vooruit
        pwm_right_fwd.ChangeDutyCycle(speed)
        pwm_right_bwd.ChangeDutyCycle(0)
    elif speed < 0:
        #Achteruit
        pwm_right_fwd.ChangeDutyCycle(0)
        pwm_right_bwd.ChangeDutyCycle(abs(speed))
    else:
        #Stop
        pwm_right_fwd.ChangeDutyCycle(0)
        pwm_right_bwd.ChangeDutyCycle(0)

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
        #Noodstop als de stick wordt ingedrukt
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
    pwm_left_fwd.stop()
    pwm_left_bwd.stop()
    pwm_right_fwd.stop()
    pwm_right_bwd.stop()
    GPIO.cleanup()

