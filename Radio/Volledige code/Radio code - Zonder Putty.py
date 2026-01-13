import serial
import time
import threading 
import csv
import sys
import os
from datetime import datetime

#Configuratie Laptop
LAPTOP_SERIAL_PORT = 'COM5' 
BAUDRATE = 57600
# Sla op in de map Documents
CSV_BESTANDSNAAM = r'C:\Users\Eric\Documents\meetgegevens.csv'

# Globale variabele om te stoppen
running = True

def setup_serial():
    try:
        ser = serial.Serial(LAPTOP_SERIAL_PORT, BAUDRATE, timeout=1)
        print(f"Verbonden met radio op {LAPTOP_SERIAL_PORT}")
        return ser
    except serial.SerialException as e:
        print(f"Kan poort niet openen: {e}")
        sys.exit(1)

def listen_to_radio(ser, csv_writer, csv_file_handle):
    print("Luisteren naar radio")
    while running:
        try:
            if ser.in_waiting > 0:
                raw_line = ser.readline()
                try:
                    line = raw_line.decode('utf-8').strip()
                    print(f"ONTVANGEN: {line}") 

                    # Om te testen slaan we alles op wat niet leeg is
                    if len(line) > 0:
                        tijdstempel = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        
                        # Probeer de inkomende data te splitsen
                        data_delen = line.split(',') 
                        
                        rij = [tijdstempel] + data_delen
                        csv_writer.writerow(rij)
                        
                        # Sla het op in de harde schijf
                        csv_file_handle.flush() 
                        
                except UnicodeDecodeError:
                    print(f"Onleesbaar: {raw_line}")
            time.sleep(0.01)
        except Exception as e:
            print(f"Fout: {e}")
            break

if __name__ == "__main__":

    #Geef aan waar het bestand opgeslagen wordt
    print(f"Het bestand wordt opgeslagen op: {os.path.abspath(CSV_BESTANDSNAAM)}")

    ser = setup_serial()
    
    # Delimiter ; toevoergen
    mode = 'a' if os.path.exists(CSV_BESTANDSNAAM) else 'w'
    csv_file = open(CSV_BESTANDSNAAM, mode, newline='')
    writer = csv.writer(csv_file, delimiter=';')
    
    if mode == 'w':
        writer.writerow(['Ontvangst_Tijd', 'Sensor_Tijd', 'Teller', 'Temperatuur'])

    thread = threading.Thread(target=listen_to_radio, args=(ser, writer, csv_file))
    thread.daemon = True
    thread.start()

    # Code menu

    print("\n--- GROUND STATION CONTROLLER ---")
    print("1: Start Meting")
    print("2: Stop Meting")
    print("3: Status Opvragen")
    print("4: Afsluiten")
    print("---------------------------------")

    try:
        while running:
            # Commando's
            keuze = input("Kies commando: ")
            
            commando = ""
            if keuze == "1":
                commando = "Start_Meten"
            elif keuze == "2":
                commando = "Stop_Meten"
            elif keuze == "3":
                commando = "Status"
            elif keuze.lower() == "4":
                print("Afsluiten...")
                running = False
                break
            else:
                print("Ongeldige keuze.")
                continue

            if commando:
                print(f"➡️ Verzenden: {commando}")
                # Stuur commando met een enter erachter
                ser.write((commando + "\n").encode('utf-8'))
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nGestopt door gebruiker.")
    finally:
        csv_file.close()
        ser.close()
        print("Verbinding verbroken.")