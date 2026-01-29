"""
Measures distance using an ultrasonic sensor and
returns the average value over a sampling period.
"""

from gpiozero import DistanceSensor
import time

Trigger = 23
Echo = 24

def get_distance_mm(duration=3.0):
    
    # Create ultrasonic sensor interface
    sensor = DistanceSensor(echo=Echo, trigger=Trigger, max_distance=4)

    try:
        readings = []
        start = time.time()

        # Collect samples for the given duration
        while time.time() - start < duration:
            readings.append(sensor.distance * 1000)
            time.sleep(0.01)

        return sum(readings) / len(readings)

    finally:
        # Release GPIO resources
        sensor.close()

if __name__ == "__main__":
    distance = get_distance_mm(duration=3.0)
    print(f"Average distance: {distance:.1f} mm")
