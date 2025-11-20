#Joystick code

#Import modules and setup GPIO pins
import RPi.GPIO as GPIO
from time import sleep
from smbus import SMBus

#Functions for ADS7830
ads7830_commands = (x, x, x)

#Functions to read from ADS7830
def read_ads7830(input):
    bus.write_byte(x, ads7830_commands[input])
    return bus.read_byte(x)

#Use BCM pin numbers since extender board is labeled in BCM
GPIO.setmode(GPIO.BCM)
delayTime = 0.1

#Start an SMBus object
bus = SMBus(1)

#Setup the GPIO pin for the button on the joystick
buttonPin = x
GPIO.setup(buttonPin, GPIO.IN, pull_up_down = GPIO.PUD_UP)

#Variable to keep track of the button value from the previous loop
buttonPrevious = 1

#Variable to keep track of whether the middleLED is on or off
middleLEDState = True

#Setup Motor
x1Motor = x
x2Motor = x
y1Motor = y
y2Motor = y

GPIO.setup(x1Motor, GPIO.OUT)
GPIO.setup(y1Motor, GPIO.OUT)

try:
    while True:
        #Read x signal from channel 0
        xAV = read_ads7830(0)
        #Read y signal from channel 0
        yAV = read_ads7830(1)

        #x joystick direction
        if xAV < 25:
            GPIO.output(x1Motor, True)
            GPIO.output(x2Motor, False)

        elif xAV > 230:
            GPIO.output(x1Motor, False)
            GPIO.output(x2Motor, True)

        else:
            GPIO.output(x1Motor, False)
            GPIO.output(x2Motor, False)

        #y joystick direction
        if yAV < 25:
            GPIO.output(y1Motor, True)
            GPIO.output(y2Motor, False)

        elif yAV > 230:
            GPIO.output(y1Motor, False)
            GPIO.output(y2Motor, True)

        else:
            GPIO.output(y1Motor, False)
            GPIO.output(y2Motor, False)

        #button toggle
        #read the button
        buttonCurrent = GPIO.input(buttonPin)
        #below is the state of the two variables that indicates the
        #button has been pressed AND let go of
        if (buttonPrevious==0 and buttonCurrent==1):
            #determine if the Motor is currently on or off
            #switch LEDState to True or False for next button push
            if middleLEDState == False:
                GPIO.output(middleLED, True)
                middleLEDState == True
            else:
                GPIO.output(middleLED, False)
                middleLEDState == False

        buttonPrevious = buttonCurrent

        sleep(delayTime)

except KeyboardInterrupt:
    GPIO.cleanup()





