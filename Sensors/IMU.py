import board
import busio
import adafruit_lsm9ds1

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

print("Acceleration:", tuple(sensor.acceleration))
print("Gyro:", tuple(sensor.gyro))
print("Magnetometer:", tuple(sensor.magnetic))
