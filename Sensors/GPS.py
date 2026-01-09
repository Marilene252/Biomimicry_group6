import time
import serial
import adafruit_gps

uart = serial.Serial("/dev/serial0", baudrate=9600, timeout=10)

gps = adafruit_gps.GPS(uart, debug=False)
gps.send_command(b'PMTK220,1000') 

print("Waiting for GPS fix")

while True:
    gps.update()  

    if not gps.has_fix:
        print("Waiting for fix")
        time.sleep(1)
        continue

    print(f"Latitude: {gps.latitude:.6f} degrees")
    print(f"Longitude: {gps.longitude:.6f} degrees")
    print(f"Fix quality: {gps.fix_quality}")
    print(f"Satellites: {gps.satellites}")
    print(f"Altitude: {gps.altitude_m} m")
    print(f"Speed: {gps.speed_knots} knots")
    print("-" * 40)

    time.sleep(1)
