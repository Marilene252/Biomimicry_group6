import time
import csv
import os
import board
import busio
import adafruit_bno055

# ---------------- IMU SETUP ----------------
i2c = busio.I2C(board.SCL, board.SDA)
imu = adafruit_bno055.BNO055_I2C(i2c)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(BASE_DIR, "results_IMU")
os.makedirs(RESULTS_DIR, exist_ok=True)


class IMULogger:

    def __init__(self):
        self.logging = False
        self.file = None
        self.writer = None
        self.start_time = None

    # ----- Start logging -----
    def start_logging(self):
        if self.logging:
            print("⚠ Already logging")
            return

        filename = os.path.join(
            RESULTS_DIR,
            f"imu_run_{int(time.time())}.csv"
        )

        self.file = open(filename, "w", newline="")
        self.writer = csv.writer(self.file)

        self.writer.writerow([
            "time",
            "heading",
            "roll",
            "pitch",
            "gx", "gy", "gz",
            "ax", "ay", "az",
            "cal_sys", "cal_gyro", "cal_acc", "cal_mag"
        ])

        self.start_time = time.time()
        self.logging = True

        print(f"🟢 IMU logging started → {filename}")
        print("⚠ Make sure the IMU is calibrated (move in figure-8 if needed)")

    # ----- Stop logging -----
    def stop_logging(self):
        if not self.logging:
            print("⚠ Not logging")
            return

        self.logging = False
        if self.file:
            self.file.close()

        print("IMU logging stopped & file saved")

    # ----- Correct axes for upside-down mounting -----
    def correct_axes(self, euler, gyro, accel):
        # Flip axes because sensor is mounted upside-down
        if euler:
            heading, roll, pitch = euler
            # Flip roll/pitch
            roll = -roll
            pitch = -pitch
            # Make “flat” pitch = 0° instead of 180°
            if pitch > 90:
                pitch = pitch - 180
            elif pitch < -90:
                pitch = pitch + 180
            # Flip heading
            if heading is not None:
                heading = (heading + 180) % 360
        else:
            heading = roll = pitch = None

        if gyro:
            gx, gy, gz = gyro
            gx = -gx
            gy = -gy
            gz = -gz
        else:
            gx = gy = gz = None

        if accel:
            ax, ay, az = accel
            ax = -ax
            ay = -ay
            az = -az
        else:
            ax = ay = az = None

        return (heading, roll, pitch), (gx, gy, gz), (ax, ay, az)

    # ----- Update logger -----
    def update(self):
        if not self.logging:
            return

        # Read raw IMU data
        euler = imu.euler
        gyro = imu.gyro
        accel = imu.linear_acceleration
        sys_cal, gyro_cal, acc_cal, mag_cal = imu.calibration_status

        # Apply correction
        (heading, roll, pitch), (gx, gy, gz), (ax, ay, az) = self.correct_axes(euler, gyro, accel)

        # Warn if IMU is not fully calibrated
        if sys_cal < 3 or gyro_cal < 3 or acc_cal < 3 or mag_cal < 3:
            print(f"⚠ IMU not fully calibrated: SYS={sys_cal} GYR={gyro_cal} ACC={acc_cal} MAG={mag_cal}")

        # Write to CSV
        self.writer.writerow([
            time.time() - self.start_time,
            heading, roll, pitch,
            gx, gy, gz,
            ax, ay, az,
            sys_cal, gyro_cal, acc_cal, mag_cal
        ])
