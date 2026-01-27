#Stepper Motor

from pyPS4Controller.controller import Controller
import RPi.GPIO as GPIO
import time
import threading

#Pin definities voor de stepper driver (Voorbeeld
STEP_PIN = 20
DIR_PIN = 21

GPIO.setmode(GPIO.BCM)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

class MyController(Controller):
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.running = False
        self.speed = 0.01
        self.direction = GPIO.HIGH
        self.motor_thread = None

    def motor_loop(self):
        # Loops on the background
        while self.running:
            if self.speed < 0.1:
                GPIO.output(DIR_PIN, self.direction)
                GPIO.output(STEP_PIN, GPIO.HIGH)
                time.sleep(self.speed)
                GPIO.output(STEP_PIN, GPIO.LOW)
                time.sleep(self.speed)
            else:
                time.sleep(0.1)

    def on_L3_up(self, value):
        #Determine speed
        val = abs(value) / 32767
        if val > 0.1:
            self.speed = 0.004 / val
            self.direction = GPIO.HIGH
            self.running = True
            if self.motor_thread is None or not self.motor_thread.is_alive():
                self.motor_thread = threading.Thread (target=self.motor_loop, daemon=True)
                self.motor_thread.start()

    def on_L3_y_at_rest(self):
        #Stop motor
        self.running = False 

    def on_L3_down(self, value): 
        val = abs(value) / 32767
        if val > 0.1:
            self.speed = 0.005 / val
            self.direction = GPIO.LOW
            self.running = True


