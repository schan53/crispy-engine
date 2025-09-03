# gps_center_확인용.py
import serial
import pynmea2
import time
import os

GPS_PORT = "/dev/gps" 
BAUDRATE = 9600

def clear_screen():#화면밀기
    os.system('cls' if os.name == 'nt' else 'clear')

def main():
    try:
        ser = serial.Serial(GPS_PORT, baudrate=BAUDRATE, timeout=1)
        print(f"{GPS_PORT}. Reading data...")
        time.sleep(1)
    except Exception as e:
        print(f"Error: Could not connect to GPS on {GPS_PORT}. {e}")
        return

    status = {
        "fix_quality": "No Fix (0)",
        "sats_in_view": 0,
        "sats_used": 0,
        "latitude": 0.0,
        "longitude": 0.0,
        "last_update": "N/A"
    }

    try:
        while True:
            try:
                line = ser.readline().decode('utf-8', errors='ignore')
                if not line:
                    continue

                msg = pynmea2.parse(line)

                if isinstance(msg, pynmea2.GGA):
                    quality = int(msg.gps_qual)#고정떠야 지피에스 사용가능 - 안될 시 노트북 유센터로 콜드스타트 요망
                    if quality == 0: status["fix_quality"] = "No Fix (0)"
                    elif quality == 1: status["fix_quality"] = "GPS Fix (1)"
                    elif quality == 2: status["fix_quality"] = "DGPS Fix (2)"
                    else: status["fix_quality"] = f"Other ({quality})"
                    
                    status["sats_used"] = int(msg.num_sats)
                    status["latitude"] = msg.latitude
                    status["longitude"] = msg.longitude
                    status["last_update"] = time.strftime("%H:%M:%S")

                if isinstance(msg, pynmea2.GSV):
                    status["sats_in_view"] = int(msg.num_sv_in_view)

                clear_screen()
                print("--- GPS Status Monitor --- (Press Ctrl+C to exit)")
                print(f"Last Update:   {status['last_update']}")
                print("-" * 28)
                print(f"Fix Quality:     {status['fix_quality']}")
                print(f"Satellites Used: {status['sats_used']}")
                print(f"Satellites View: {status['sats_in_view']}")
                print("-" * 28)
                print(f"Latitude:        {status['latitude']:.6f}")
                print(f"Longitude:       {status['longitude']:.6f}")
                print("\nSearching...")

            except pynmea2.ParseError:
                continue
            except Exception as e:
                print(f"An error occurred: {e}")
                time.sleep(1)

    except KeyboardInterrupt:
        print("\nProgram terminated by user.")
    finally:
        ser.close()
        print("Serial port closed.")

if __name__ == "__main__":
    main()
