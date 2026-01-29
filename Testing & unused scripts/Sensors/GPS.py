"""
This test script reads data from a GPS module
over UART and prints latitude, longitude,
and other fix information every second. 
This script was mainly used to test the GPS but we ultimately did not use it.
"""

import time
import serial
import adafruit_gps

# Open UART connection to the GPS module
uart = serial.Serial("/dev/serial0", baudrate=9600, timeout=10)

# Create GPS object and set update rate to 1 Hz
gps = adafruit_gps.GPS(uart, debug=False)
gps.send_command(b'PMTK220,1000') 

print("Waiting for GPS fix")

# Read new data from the GPS
while True:
    gps.update()  

# Wait until GPS has satelitte fix
    if not gps.has_fix:
        print("Waiting for fix")
        time.sleep(1)
        continue

# Display results
    print(f"Latitude: {gps.latitude:.6f} degrees")
    print(f"Longitude: {gps.longitude:.6f} degrees")
    print(f"Fix quality: {gps.fix_quality}")
    print(f"Satellites: {gps.satellites}")
    print(f"Altitude: {gps.altitude_m} m")
    print(f"Speed: {gps.speed_knots} knots")
    time.sleep(1)
