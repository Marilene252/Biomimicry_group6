"""
This test script reads orientation, angular velocity, linear acceleration,
and calibration status from a BNO055 IMU over I2C and
prints the values once per second.
"""

import time
import board
import busio
import adafruit_bno055

# Initialize I2C bus and IMU sensor
i2c = busio.I2C(board.SCL, board.SDA)
imu = adafruit_bno055.BNO055_I2C(i2c)

while True:
    # Read sensor outputs
    euler = imu.euler
    gyro = imu.gyro
    linear_accel = imu.linear_acceleration

    # Read calibration status
    sys_cal, gyro_cal, accel_cal, mag_cal = imu.calibration_status

    print("IMU Readings")

    if euler is not None:
        heading, roll, pitch = euler
        print(f"Heading={heading:.1f}° Roll={roll:.1f}° Pitch={pitch:.1f}°")
    else:
        print("Euler angles not yet available")

    if gyro is not None:
        gx, gy, gz = gyro
        print(f"Angular Velocity: X={gx:.2f} rad/s Y={gy:.2f} rad/s Z={gz:.2f} rad/s")
    else:
        print("Angular velocity not yet available")

    if linear_accel is not None:
        ax, ay, az = linear_accel
        print(f"Linear Acceleration: X={ax:.2f} m/s² Y={ay:.2f} m/s² Z={az:.2f} m/s²")
    else:
        print("Linear acceleration not yet available")

    print(f"Calibration: SYS={sys_cal} GYR={gyro_cal} ACC={accel_cal} MAG={mag_cal}")
    print("-" * 40)

    time.sleep(1)
