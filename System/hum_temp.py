import time
import board
import busio
import adafruit_sht31d
import os
from datetime import datetime

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_sht31d.SHT31D(i2c)

SYSTEM_DIR = '/home/rapi6/Biomimicry_group6/System'
RESULTS_DIR = os.path.join(SYSTEM_DIR, 'Results')
os.makedirs(RESULTS_DIR, exist_ok=True)

timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
folder = os.path.join(RESULTS_DIR, f"TempHum_{timestamp}")
os.makedirs(folder, exist_ok=True)

file_path = os.path.join(folder, 'temperature_humidity.txt')

temperature = sensor.temperature
humidity = sensor.relative_humidity

with open(file_path, 'w') as f:
    f.write(f"Date & Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
    f.write(f"Temperature: {temperature:.2f} °C\n")
    f.write(f"Humidity: {humidity:.2f} %\n")

print(f"Temperature & Humidity saved in {file_path}")
print(f"Temperature: {temperature:.2f} °C  Humidity: {humidity:.2f} %")
