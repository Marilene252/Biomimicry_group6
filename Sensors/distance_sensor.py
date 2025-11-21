import lgpio as GPIO
import time

TRIG = 23
ECHO = 24

h = GPIO.gpiochip_open(0)
GPIO.gpio_claim_output(h, TRIG)
GPIO.gpio_claim_input(h, ECHO)

def get_distance():
    """
    Measures distance in cm using ultrasonic sensor.
    """
    GPIO.gpio_write(h, TRIG, 0)
    time.sleep(0.05) 

    # Send 10us pulse
    GPIO.gpio_write(h, TRIG, 1)
    time.sleep(0.00001)
    GPIO.gpio_write(h, TRIG, 0)

    # Wait for pulse start
    while GPIO.gpio_read(h, ECHO) == 0:
        pulse_start = time.time()

    # Wait for pulse end
    while GPIO.gpio_read(h, ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance_cm = pulse_duration * 17150  
    distance_cm = round(distance_cm, 2)

    return distance_cm

def get_distance_mm():
    """
    Measures distance in millimeters.
    """
    return get_distance() * 10  

def cleanup():
    """
    Close GPIO when done.
    """
    GPIO.gpiochip_close(h)
