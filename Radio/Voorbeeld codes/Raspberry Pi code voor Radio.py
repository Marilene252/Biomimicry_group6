# Basic script for testing radios. This script is also easily to implement in other scripts

import serial
import time
import random

# Configuration Raspberry Pi
SERIAL_PORT = '/dev/ttyS0' 
BAUDRATE = 57600

def setup_serial():
    # Trying to make a connection
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        print(f"Radio active on {SERIAL_PORT}")
        return ser
    except Exception as e:
        print(f"Error: Unable to open port {e}")
        exit()

def main():
    ser = setup_serial()
    print("Waiting for start command from laptop.")
    
    measuring_active = False

    while True:
        # Searching for connection with the laptop
        if ser.in_waiting > 0:
            command = ser.readline().decode('utf-8').strip()
            print(f"Command received: {command}")
            
            if command == "Start_Measuring":
                print("START signal received! I am now going to transmit.")
                measuring_active = True
        
        # If measuring is active, than measure
        if measuring_active:
            # Make data
            sensor_time = int(time.time())
            value_1 = random.randint(100, 200)
            value_2 = round(random.uniform(20.0, 30.0)) 
            
            # Make the message
            message = f"{sensor_time},{value_1},{value_2}\n"
            
            # Send by radio
            ser.write(message.encode('utf-8'))
            print(f"Send: {message.strip()}")
            
            time.sleep(1)

        else:
            time.sleep(0.1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStop.")