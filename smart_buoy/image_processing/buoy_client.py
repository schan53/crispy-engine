# buoy_client.py

import serial
import pynmea2
from datetime import datetime, timedelta
import time
import socket

# --- GPS CONFIGURATION ---
SERIAL_PORT = "/dev/gps"

SERVER_HOST = "192.168.50.122" # <--  님들 핫스팟에 랩탑연결하고 아이피로 변경ㄱㄱ
SERVER_PORT = 65432          
try:
    ser = serial.Serial(SERIAL_PORT, baudrate=9600, timeout=1)
    print(f"Successfully connected to GPS module on port {SERIAL_PORT}.")
except serial.SerialException:
    print(f"Error: Could not find or access port {SERIAL_PORT}.")
    exit()

def connect_to_server():
    """
    Attempts to connect to the server. If the connection fails,
    it waits for 5 seconds and retries indefinitely.
    """
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((SERVER_HOST, SERVER_PORT))
            print(f"Successfully connected to Rescue HQ at {SERVER_HOST}:{SERVER_PORT}")
            return s
        except (socket.error, ConnectionRefusedError) as e:
            print(f"Connection failed: {e}. Retrying in 5 seconds...")
            time.sleep(5)

client_socket = connect_to_server()

latest_date = None
latest_time = None
gps_fix_status = False

# 메인뤂
try:
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore')

            if not line:
                continue
            msg = pynmea2.parse(line)
            if isinstance(msg, pynmea2.RMC) and msg.datestamp:
                latest_date = msg.datestamp
            if isinstance(msg, pynmea2.GGA):
                if msg.gps_qual > 0:
                    gps_fix_status = True
                    if msg.timestamp:
                        latest_time = msg.timestamp
                else:
                    gps_fix_status = False
            if gps_fix_status and latest_date and latest_time and hasattr(msg, 'latitude'):
                utc_datetime = datetime.combine(latest_date, latest_time)
                
                kst_datetime = utc_datetime + timedelta(hours=9)
                
                data_to_send = (
                    f"{kst_datetime.strftime('%Y-%m-%d')},"
                    f"{kst_datetime.strftime('%H:%M:%S')},"
                    f"{msg.latitude:.5f},"
                    f"{msg.longitude:.5f}"
                )

                try:
                    client_socket.sendall(data_to_send.encode('utf-8'))
                    print(f"Successfully sent data: {data_to_send}")
                except (socket.error, BrokenPipeError) as e:
                    print(f"Connection lost: {e}. Attempting to reconnect...")
                    client_socket.close()
                    client_socket = connect_to_server() # Reconnect
                latest_time = None
            
            else:
                print("Searching for GPS signal...", end='\r')

        except pynmea2.ParseError:
            continue        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nProgram terminated by user.")
except Exception as e:
    print(f"\nAn unexpected error occurred: {e}")
finally:
    ser.close()
    if client_socket:
        client_socket.close()
    print("Serial port and connection closed.")
