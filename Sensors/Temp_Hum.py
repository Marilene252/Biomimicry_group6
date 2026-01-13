import time
import board
import busio
import adafruit_sht31d

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_sht31d.SHT31D(i2c)

print("SHT31-D Temperature & Humidity Sensor detected!")

while True:
    temperature = sensor.temperature
    humidity = sensor.relative_humidity

    print(f"Temperature: {temperature:.2f} °C  Humidity: {humidity:.2f} %")
    
    time.sleep(1)
