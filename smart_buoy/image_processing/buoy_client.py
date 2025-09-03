# buoy_client.py
# Run this script on the Raspberry Pi 5 (the "Buoy").

import serial
import pynmea2
from datetime import datetime, timedelta
import time
import socket

# --- GPS CONFIGURATION ---
# Enter the serial port name of your GPS module.
SERIAL_PORT = "/dev/gps"

# --- NETWORK CONFIGURATION ---
# IMPORTANT: Replace this with the IP address of the laptop running the server script.
SERVER_HOST = "192.168.50.122" # <-- CHANGE THIS
SERVER_PORT = 65432          # The port to connect to. Must match the server.

# --- GPS INITIALIZATION ---
try:
    # Initialize the serial connection.
    ser = serial.Serial(SERIAL_PORT, baudrate=9600, timeout=1)
    print(f"Successfully connected to GPS module on port {SERIAL_PORT}.")
except serial.SerialException:
    print(f"Error: Could not find or access port {SERIAL_PORT}.")
    exit()

# Function to handle connection and reconnection logic.
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

# --- INITIAL CONNECTION ---
client_socket = connect_to_server()

# Variables to store the latest date and time received from the GPS.
latest_date = None
latest_time = None
gps_fix_status = False

# --- MAIN LOOP ---
try:
    while True:
        try:
            # Read a line of data from the serial port.
            line = ser.readline().decode('utf-8', errors='ignore')

            if not line:
                continue

            # Parse the NMEA sentence.
            msg = pynmea2.parse(line)

            # Extract date from RMC sentences.
            if isinstance(msg, pynmea2.RMC) and msg.datestamp:
                latest_date = msg.datestamp

            # Extract time and fix status from GGA sentences.
            if isinstance(msg, pynmea2.GGA):
                if msg.gps_qual > 0:
                    gps_fix_status = True
                    if msg.timestamp:
                        latest_time = msg.timestamp
                else:
                    gps_fix_status = False
            # Check if we have all the necessary data to send.
            if gps_fix_status and latest_date and latest_time and hasattr(msg, 'latitude'):
                # Combine date and time into a single datetime object.
                utc_datetime = datetime.combine(latest_date, latest_time)
                
                # Convert from UTC to KST (UTC+9).
                kst_datetime = utc_datetime + timedelta(hours=9)
                
                # Format the data into a single string, separated by commas.
                data_to_send = (
                    f"{kst_datetime.strftime('%Y-%m-%d')},"
                    f"{kst_datetime.strftime('%H:%M:%S')},"
                    f"{msg.latitude:.5f},"
                    f"{msg.longitude:.5f}"
                )

                # --- SEND DATA TO SERVER using the persistent connection ---
                try:
                    # Encode the string to bytes and send it.
                    client_socket.sendall(data_to_send.encode('utf-8'))
                    print(f"Successfully sent data: {data_to_send}")
                except (socket.error, BrokenPipeError) as e:
                    print(f"Connection lost: {e}. Attempting to reconnect...")
                    client_socket.close()
                    client_socket = connect_to_server() # Reconnect
                
                # Reset to avoid sending the same data point multiple times.
                latest_time = None
            
            else:
                # If we are still waiting for a fix or data, print a status message.
                print("Searching for GPS signal...", end='\r')

        except pynmea2.ParseError:
            # Ignore sentences that cannot be parsed.
            continue
        
        # A small delay to prevent overwhelming the CPU.
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nProgram terminated by user.")
except Exception as e:
    print(f"\nAn unexpected error occurred: {e}")
finally:
    # Ensure the serial port and socket are closed when the program exits.
    ser.close()
    if client_socket:
        client_socket.close()
    print("Serial port and connection closed.")
