"""
Reads temperature and relative humidity from an SHT31-D sensor
over I2C and prints the values once per second.
"""

import time
import board
import busio
import adafruit_sht31d

# Initialize I2C bus and sensor
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_sht31d.SHT31D(i2c)

print("SHT31-D Temperature & Humidity Sensor detected!")

while True:
    # Read temperature and humidity values
    temperature = sensor.temperature
    humidity = sensor.relative_humidity

    print(f"Temperature: {temperature:.2f} °C  Humidity: {humidity:.2f} %")
    
    time.sleep(1)
