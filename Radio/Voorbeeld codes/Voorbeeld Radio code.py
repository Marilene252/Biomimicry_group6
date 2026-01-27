# The script shows how the radios work. It has many commands to explain what each line does. Because of these commands this script is also easily to implenent in other scripts.

import serial
import time
import random
import sys

# Configuration Variables
SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 57600             
MEASUREMENT_INTERVAL = 1     

# Global flag to regulate the measuring device
is_measuring = False 
# Global counter for the test
measurement_counter = 0

def setup_serial():
    """Sets the serial connection."""
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUDRATE,
            timeout=1
        )
        print(f"Serial port {SERIAL_PORT} opened at {BAUDRATE} Baud.")
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        print("Check the ort name and whether the radio is connected.")
        sys.exit(1)

def start_measurement_loop(ser):
    global is_measuring, measurement_counter
    is_measuring = True
    
    print("Command received: Start Measuring")
    
    try:
        while is_measuring:
            # Simple measurement code
            
            # 1. Counter +1
            measurement_counter += 1
            
            # 2. Simulate a random temperature
            temp_value = 20.0 + random.uniform(-0.5, 0.5)
            
            # 3. Create a dataformat 
            data_string = f"MEASUREMENT {measurement_counter}: Temp={temp_value:.2f}C\n"
            
            # 4. Send by radio
            ser.write(data_string.encode('utf-8'))
            print(f"Sent: {data_string.strip()}")
            
            # 5. Prevent CPU from overloading
            time.sleep(MEASUREMENT_INTERVAL)
            
    except Exception as e:
        print(f"Error while measuring: {e}")
    finally:
        is_measuring = False
        print("Stop Measuring")

def main():
    ser = setup_serial()
    print("\nListen to radio. Send 'Start_Measuring' or 'Stop_Measuring' via PuTTY.")

    global is_measuring
    
    while True:
        try:
            # 1. Read a line from the searial port (incomming command)
            response = ser.readline()
            
            if response:
                # Decode the received bytes into a usable string
                command = response.decode('utf-8').strip()
                print(f"\nCommand received: {command}")
                
                # 2. Respond to the command
                if command == "Start_Measuring" and not is_measuring:
                    start_measurement_loop(ser) 
                    
                elif command == "Stop_Measuring":
                    if is_measuring:
                        is_measuring = False
                
                elif command == "Status":
                    status_msg = f"STATUS: Measurements active: {is_measuring}, Laatste teller: {measurement_counter}"
                    ser.write((status_msg + "\n").encode('utf-8'))
                    print(f"➡️ Status sent: {status_msg}")

            # Main loop can not be stuck
            time.sleep(0.1) 

        except KeyboardInterrupt:
            print("\nProgram stopped by user.")
            break
        except Exception as e:
            print(f"\nAn unexpected error has occured in the main loop: {e}")
            break
    
    if ser.is_open:
        ser.close()
        print("Serial port closed.")

if __name__ == "__main__":
    main()