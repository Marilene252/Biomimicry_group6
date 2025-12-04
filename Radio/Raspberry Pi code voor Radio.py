import serial
import time
import random

# Configuratie Raspberry Pi
SERIAL_PORT = '/dev/ttyS0' 
BAUDRATE = 57600

def setup_serial():
    # Probeer verbinding te maken
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        print(f"Radio actief op {SERIAL_PORT}")
        return ser
    except Exception as e:
        print(f"Fout: Kan poort niet openen: {e}")
        exit()

def main():
    ser = setup_serial()
    print("Wachten op start-commando van laptop.")
    
    meten_actief = False

    while True:
        # Zoek verbinding met de laptop
        if ser.in_waiting > 0:
            commando = ser.readline().decode('utf-8').strip()
            print(f"Commando ontvangen: {commando}")
            
            if commando == "Start_Meten":
                print("START signaal ontvangen! Ik ga nu zenden.")
                meten_actief = True
        
        # Als meten actief is, dan meten
        if meten_actief:
            # Maak nep-data aan (dit wordt later vervangen door echte data)
            sensor_tijd = int(time.time())
            waarde_1 = random.randint(100, 200) # Bijv. teller
            waarde_2 = round(random.uniform(20.0, 30.0), 2) # Bijv. temperatuur
            
            # Maak de regel
            bericht = f"{sensor_tijd},{waarde_1},{waarde_2}\n"
            
            # Verstuur via radio
            ser.write(bericht.encode('utf-8'))
            print(f"Verzonden: {bericht.strip()}")
            
            time.sleep(1)

        else:
            time.sleep(0.1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nGestopt.")