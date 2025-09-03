import serial
import pynmea2
from datetime import datetime, timedelta
import time

SERIAL_PORT = "/dev/gps"

try:
    ser = serial.Serial(SERIAL_PORT, baudrate=9600, timeout=1)
    print(f"Successfully connected to port {SERIAL_PORT}. Waiting for GPS signal...")
except serial.SerialException:
    print(f"Error: Could not find or access port {SERIAL_PORT}.")
    exit()

latest_date = None
latest_time = None
gps_fix_status = False

try:
    while True:
        line = ser.readline().decode('utf-8', errors='ignore')

        if not line:
            continue

        try:
            msg = pynmea2.parse(line)

            if isinstance(msg, pynmea2.RMC):
                if msg.datestamp:
                    latest_date = msg.datestamp

            if isinstance(msg, pynmea2.GGA):
                if msg.gps_qual > 0:
                    gps_fix_status = True
                    if msg.timestamp:
                        latest_time = msg.timestamp
                else:
                    gps_fix_status = False

            if gps_fix_status and latest_date and latest_time:
                utc_datetime = datetime.combine(latest_date, latest_time)
                
                kst_datetime = utc_datetime + timedelta(hours=9)
                
                print(f"Date(KST): {kst_datetime.strftime('%Y-%m-%d')} | Time(KST): {kst_datetime.strftime('%H:%M:%S')} | Latitude: {msg.latitude:.5f} | Longitude: {msg.longitude:.5f}")
                
                latest_time = None
            else:
                print("Searching for GPS signal...", end='\r')
        
        except pynmea2.ParseError:
            continue

except KeyboardInterrupt:
    print("\nProgram terminated by user.")
except Exception as e:
    print(f"\nAn unexpected error occurred: {e}")
finally:
    ser.close()
    print("Serial port closed.")
