from gpiozero import DistanceSensor
import time

Trigger = 23
Echo = 24

def get_distance_mm(duration=3.0):
    sensor = DistanceSensor(echo=Echo, trigger=Trigger, max_distance=4)
    try:
        readings = []
        start = time.time()
        while time.time() - start < duration:
            readings.append(sensor.distance * 1000)  # meters → mm
            time.sleep(0.01)  # small delay to stabilize readings
        return sum(readings) / len(readings)
    finally:
        sensor.close()  # release GPIO pins

if __name__ == "__main__":
    distance = get_distance_mm(duration=3.0)
    print(f"Average distance: {distance:.1f} mm")