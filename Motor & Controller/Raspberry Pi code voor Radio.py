import serial
import time
import random

# --- CONFIGURATIE RASPBERRY PI ---
# Op oudere Pi's is dit vaak '/dev/ttyAMA0', op nieuwere '/dev/ttyS0'
# Als je een USB-kabel gebruikt, is het '/dev/ttyUSB0'
SERIAL_PORT = '/dev/ttyUSB0' 
BAUDRATE = 57600

def setup_serial():
    # Probeer verbinding te maken
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        print(f"✅ Radio actief op {SERIAL_PORT}")
        return ser
    except Exception as e:
        print(f"❌ Fout: Kan poort niet openen: {e}")
        print("Tip: Heb je 'Serial Console' uitgezet in raspi-config?")
        exit()

def main():
    ser = setup_serial()
    print("⏳ Wachten op start-commando van laptop...")
    
    meten_actief = False

    while True:
        # 1. Luister of de laptop iets zegt
        if ser.in_waiting > 0:
            commando = ser.readline().decode('utf-8').strip()
            print(f"📩 Commando ontvangen: {commando}")
            
            if commando == "Start_Meten":
                print("🚀 START signaal ontvangen! Ik ga nu zenden.")
                meten_actief = True
                
            elif commando == "Stop_Meten":
                print("🚀 STOP signaal ontvangen. Zenden gepauzeerd.")
                meten_actief = False
                
            elif commando == "Status":
                print("Status opgevraagd.")
                status_msg = f"STATUS_REPORT: Meten={'AAN' if meten_actief else 'UIT'}, Systeem=Ok/n"
                ser.write(status_msg.encode('utf-8'))
                print("Status verzonden.")
        
        # 2. Als we mogen meten, stuur data
        if meten_actief:
            # Maak wat nep-data (later vervang je dit door echte sensor-data)
            sensor_tijd = int(time.time())
            waarde_1 = random.randint(100, 200) # Bijv. teller
            waarde_2 = round(random.uniform(20.0, 30.0), 2) # Bijv. temperatuur
            
            # Maak de regel (met komma's ertussen!)
            bericht = f"{sensor_tijd},{waarde_1},{waarde_2}\n"
            
            # Verstuur via radio
            ser.write(bericht.encode('utf-8'))
            print(f"📤 Verzonden: {bericht.strip()}")
            
            time.sleep(1) # Wacht 1 seconde

        else:
            # We zijn nog niet gestart, wacht kort om CPU te sparen
            time.sleep(0.1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nGestopt.")
