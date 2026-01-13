import serial
import time
import random
import sys

# --- Configuratie Variabelen ---
SERIAL_PORT = '/dev/ttyUSB0' # Controleer of dit overeenkomt met uw USB-radio
BAUDRATE = 57600             # Moet overeenkomen met de baudrate van de radio's
MEASUREMENT_INTERVAL = 1     # Tijd tussen metingen (in seconden)

# Globale vlag om de meetlus te regelen
is_measuring = False 
# Globale teller voor de simpele test
measurement_counter = 0

def setup_serial():
    """Stelt de seriële verbinding in."""
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUDRATE,
            timeout=1  # Lees-timeout: bepaalt hoe lang .readline() wacht
        )
        print(f"✅ Seriële poort {SERIAL_PORT} geopend op {BAUDRATE} Baud.")
        return ser
    except serial.SerialException as e:
        print(f"❌ Fout bij het openen van de seriële poort: {e}")
        print("Controleer de poortnaam en of de radio is aangesloten.")
        sys.exit(1)

# ======================================================================
# DEZE FUNCTIE BEVAT DE SPECIFIEKE METINGSCODE
# ======================================================================
def start_measurement_loop(ser):
    global is_measuring, measurement_counter
    is_measuring = True
    
    print("--- COMMANDO ONTVANGEN: BEGIN METING ---")
    
    try:
        while is_measuring:
            # Vanaf hier de meest simpele metingscode om te testen
            
            # 1. Increment de teller
            measurement_counter += 1
            
            # 2. Simuleer een willekeurige temperatuur
            temp_value = 20.0 + random.uniform(-0.5, 0.5)
            
            # 3. Creëer het dataformaat (simpele tekst)
            data_string = f"MEASUREMENT {measurement_counter}: Temp={temp_value:.2f}C\n"
            
            # 4. Verzenden via de radio
            ser.write(data_string.encode('utf-8'))
            print(f"➡️ Verzonden: {data_string.strip()}")
            
            # 5. Voorkom dat de CPU overbelast raakt
            time.sleep(MEASUREMENT_INTERVAL)
            
    except Exception as e:
        print(f"Fout tijdens meting: {e}")
    finally:
        is_measuring = False
        print("--- EINDE METING ---")
# ======================================================================

def main():
    ser = setup_serial()
    print("\nLuistert naar de radio. Stuur 'Start_Meten' of 'Stop_Meten' via PuTTY.")

    global is_measuring
    
    while True:
        try:
            # 1. Lees een regel van de seriële poort (inkomend commando)
            response = ser.readline()
            
            if response:
                # Decodeer de ontvangen bytes naar een bruikbare string
                command = response.decode('utf-8').strip()
                print(f"\n⬅️ Commando ontvangen: {command}")
                
                # 2. Reageer op het commando
                if command == "Start_Meten" and not is_measuring:
                    # Start de metingen
                    start_measurement_loop(ser) 
                    
                elif command == "Stop_Meten":
                    # Zet de vlag om de meetlus te beëindigen
                    if is_measuring:
                        is_measuring = False
                
                elif command == "Status":
                    # Terugkoppeling sturen naar de laptop
                    status_msg = f"STATUS: Metingen actief: {is_measuring}, Laatste teller: {measurement_counter}"
                    ser.write((status_msg + "\n").encode('utf-8'))
                    print(f"➡️ Status verzonden: {status_msg}")
            
            # Zorg ervoor dat de hoofdloop niet vastloopt, zelfs als er geen data binnenkomt
            time.sleep(0.1) 

        except KeyboardInterrupt:
            print("\nProgramma gestopt door gebruiker.")
            break
        except Exception as e:
            print(f"\nEr is een onverwachte fout opgetreden in de hoofdloop: {e}")
            break
    
    if ser.is_open:
        ser.close()
        print("Seriële poort gesloten.")

if __name__ == "__main__":
    main()