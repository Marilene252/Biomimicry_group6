"""
This script is what the team would use for the Radios. It now makes fake data,
but it's easily to impelement the sensors measurements in that part of the code.
This script implements the two scripts from the Example codes folder.
"""

import serial
import time
import threading 
import csv
import sys
import os
from datetime import datetime

#Configuration Laptop
LAPTOP_SERIAL_PORT = 'COM5' 
BAUDRATE = 57600
# Save in the folder Documents
CSV_BESTANDSNAAM = r'C:\Users\Eric\Documents\meetgegevens.csv'

# Global variables to stop
running = True

def setup_serial():
    try:
        ser = serial.Serial(LAPTOP_SERIAL_PORT, BAUDRATE, timeout=1)
        print(f"Connected with radio on {LAPTOP_SERIAL_PORT}")
        return ser
    except serial.SerialException as e:
        print(f"Can not open port: {e}")
        sys.exit(1)

def listen_to_radio(ser, csv_writer, csv_file_handle):
    print("Listen to radio")
    while running:
        try:
            if ser.in_waiting > 0:
                raw_line = ser.readline()
                try:
                    line = raw_line.decode('utf-8').strip()
                    print(f"Received: {line}") 

                    # To test, we save everything that is not empty
                    if len(line) > 0:
                        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        
                        # Splitsing incomming data
                        data_sharing = line.split(',') 
                        
                        rij = [timestamp] + data_sharing
                        csv_writer.writerow(rij)
                        
                        # Save it to the hard drive
                        csv_file_handle.flush() 
                        
                except UnicodeDecodeError:
                    print(f"Unreadable: {raw_line}")
            time.sleep(0.01)
        except Exception as e:
            print(f"Error: {e}")
            break

if __name__ == "__main__":

    #Where the file will be saved
    print(f"The file is saved to: {os.path.abspath(CSV_BESTANDSNAAM)}")

    ser = setup_serial()
    
    # add delimiter
    mode = 'a' if os.path.exists(CSV_BESTANDSNAAM) else 'w'
    csv_file = open(CSV_BESTANDSNAAM, mode, newline='')
    writer = csv.writer(csv_file, delimiter=';')
    
    if mode == 'w':
        writer.writerow(['Receipt_Time', 'Sensor_Time', 'Counter', 'Temperature'])

    thread = threading.Thread(target=listen_to_radio, args=(ser, writer, csv_file))
    thread.daemon = True
    thread.start()

    # Code menu

    print("\n--- GROUND STATION CONTROLLER ---")
    print("1: Start Measuring")
    print("2: Stop Measuring")
    print("3: Check Status")
    print("4: Close")
    print("---------------------------------")

    try:
        while running:
            # Commands
            choice = input("Choose command: ")
            
            command = ""
            if choice == "1":
                command = "Start_Measuring"
            elif choice == "2":
                command = "Stop_Measuring"
            elif choice == "3":
                command = "Status"
            elif choice.lower() == "4":
                print("Close...")
                running = False
                break
            else:
                print("Invalid choice.")
                continue

            if commando:
                print(f"Send: {command}")
                # Send command followed by enter
                ser.write((command + "\n").encode('utf-8'))
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        csv_file.close()
        ser.close()
        print("Connection lost.")