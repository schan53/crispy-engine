# dual_camera_autodrive_system.py
import cv2
import time
import math
import serial
import numpy as np
from ultralytics import YOLO
import sys 

# --- Camera Libraries ---
from picamera2 import Picamera2
from pylepton.Lepton3 import Lepton3

# --- [AUTO-CONFIG] System Configuration ---
# This script will now automatically detect the light level to choose the camera.
LUX_THRESHOLD = 25.0 # If lux is below this value, use Thermal Camera (night mode)


# --- System Configuration and Constants ---

# --- Motor and Control Constants ---
MIN_APPROACH_DISTANCE_M = 0.7
CENTER_THRESHOLD_RGB = 30
CENTER_THRESHOLD_THERMAL = 15
M1_PWM_ON = 1200
M2_PWM_ON = 1080
MOTOR_OFF = 1000

# --- Rescue State Definitions ---
STATE_SEARCHING = "SEARCHING"
STATE_APPROACHING = "APPROACHING"
STATE_ARRIVED = "ARRIVED"

# --- Distance Estimation Constants ---
K_CONSTANT_RGB = 178.0
K_CONSTANT_THERMAL = 89.0
#_A4_MODEL
#K_CONSTANT_RGB = 31.8 
#K_CONSTANT_RGB = 15.9

# --- Keypoint connection pairs for drawing skeleton ---
skeleton_pairs = [
    (5, 6), (5, 7), (7, 9), (6, 8), (8, 10), (5, 11), (6, 12), (11, 12),
    (11, 13), (13, 15), (12, 14), (14, 16)
]

# --- Initialization ---

# --- [MODIFIED] Serial Port Setup ---
ARDUINO_PORT = "/dev/arduino"
ser = None
try:
    # Increase timeout to wait for Arduino to be ready
    ser = serial.Serial(ARDUINO_PORT, 9600, timeout=15) 
    print(f"Arduino connected on {ARDUINO_PORT}. Sending START signal...")
    
    # Send the START signal to trigger the buzzer
    ser.write(b'START\n')
    handshake_successful = False
    # Now wait for the "READY" signal back from Arduino
    while True:
        response = ser.readline().decode('utf-8').strip()
        if response == "READY":
            print(f"Arduino is ready.")
            handshake_successful = True
            break
        if not response: # For debugging
            print("Did not receive 'READY' within the timeout. Assuming Arduino is already running.")
            break

except Exception as e:
    print(f"Failed to connect to Arduino on {ARDUINO_PORT}: {e}")


def get_light_level(serial_port):
    """Requests the current light level from the Arduino."""
    if serial_port is None:
        return None
    try:
        serial_port.reset_input_buffer()
        serial_port.write(b'?\n') # Send request character
        lux_str = serial_port.readline().decode('utf-8').strip()
        return float(lux_str)
    except (serial.SerialException, ValueError, TypeError):
        return None

# --- Determine Camera Mode based on Light Sensor ---
USE_THERMAL_CAMERA = False # Default to RGB
current_lux = 0.0
if ser:
    lux = get_light_level(ser)
    if lux is not None:
        current_lux = lux
        print(f"Initial light level: {lux:.2f} lux")
        if lux < LUX_THRESHOLD:
            USE_THERMAL_CAMERA = True
    else:
        print("Could not get light level from Arduino. Defaulting to RGB mode.")

# Camera and YOLO Model Initialization
picam2 = None
model = None 

if USE_THERMAL_CAMERA:
    print("Initializing Thermal Mode (Night)...")
    model = YOLO("yolo11n-pose_ncnn_model")
else: # Use RGB Camera
    print("Initializing RGB Mode (Day)...")
    model = YOLO("yolo11s-pose_ncnn_model")
    try:
        picam2 = Picamera2()
        picam2.preview_configuration.main.size = (640, 480)
        picam2.preview_configuration.main.format = "RGB888"
        picam2.preview_configuration.align()
        picam2.configure("preview")
        picam2.start()
        time.sleep(1)
    except Exception as e:
        print(f"Error initializing RGB Camera: {e}")
        exit()
	

# =================================================================================
# --- Core Function Definitions ---
# =================================================================================

# This function now sends the current state along with motor commands
def update_arduino_state(state, m1_pwm, m2_pwm):
    if ser is not None:
        command = f"{state},{m1_pwm},{m2_pwm}\n"
        ser.write(command.encode())

def is_handsup_pose(keypoints):
    nose_y = keypoints[0][1]
    lshoulder_y = keypoints[5][1]
    rshoulder_y = keypoints[6][1]
    lwrist_y = keypoints[9][1]
    rwrist_y = keypoints[10][1]
    if lwrist_y < nose_y and rwrist_y < nose_y and lwrist_y < lshoulder_y and rwrist_y < rshoulder_y:
        return True
    return False

def get_shoulder_distance(keypoints):
    x1, y1 = keypoints[5]
    x2, y2 = keypoints[6]
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def estimate_distance(shoulder_pixel, k_constant):
    if shoulder_pixel == 0:
        return None
    return round(k_constant / shoulder_pixel, 2)

# =================================================================================
# --- Main Program Execution ---
# =================================================================================

def main_loop():
    global current_lux # Use the global variable
    current_state = STATE_SEARCHING
    prev_time = 0
    last_lux_check_time = time.time()
    last_heartbeat_time = time.time()


    # --- Timers for state control ---
    last_target_seen_time = time.time() # Tracks when a target was last seen
    arrived_timer_start = 0             # Tracks when the buoy arrived at a target

    while True:
        # --- Periodically check light level for mode switch ---
        if time.time() - last_lux_check_time > 5.0:
            lux = get_light_level(ser)
            if lux is not None:
                current_lux = lux
                print(f"--- Sensor Update | Current Lux: {current_lux:.2f} ---")
                is_dark = current_lux < LUX_THRESHOLD
                if not USE_THERMAL_CAMERA and is_dark:
                    print(f"Lux ({current_lux:.2f}) is below threshold. Switching to THERMAL mode.")
                    update_arduino_state(STATE_SEARCHING, MOTOR_OFF, MOTOR_OFF)
                    sys.exit(10)
                elif USE_THERMAL_CAMERA and not is_dark:
                    print(f"Lux ({current_lux:.2f}) is above threshold. Switching to RGB mode.")
                    update_arduino_state(STATE_SEARCHING, MOTOR_OFF, MOTOR_OFF)
                    sys.exit(10)
            last_lux_check_time = time.time()
	    
        if time.time() - last_heartbeat_time > 10.0: 
            if ser is not None:
                ser.write(b'PING\n') 
            last_heartbeat_time = time.time()
	    	    
        # --- Frame Capture and Pre-processing ---
        if USE_THERMAL_CAMERA:
            a, _ = lepton.capture()
            if a is None: continue
            cv2.normalize(a, a, 0, 65535, cv2.NORM_MINMAX)
            np.right_shift(a, 8, a)
            frame = np.uint8(a)
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            k_dist, center_threshold, active_mode = K_CONSTANT_THERMAL, CENTER_THRESHOLD_THERMAL, "THERMAL"
        else: # RGB Mode
            frame_bgr = picam2.capture_array()
            k_dist, center_threshold, active_mode = K_CONSTANT_RGB, CENTER_THRESHOLD_RGB, "RGB"
        
        frame_height, frame_width, _ = frame_bgr.shape
        frame_center_x = frame_width / 2
        
        # --- YOLO Processing ---
        results = model(frame_bgr)
        
        # --- Display Frame Creation ---
        display_frame = frame_bgr.copy()
        if USE_THERMAL_CAMERA:
            display_frame = cv2.resize(display_frame, (640, 480), interpolation=cv2.INTER_NEAREST)

        scale_x = display_frame.shape[1] / frame_width
        scale_y = display_frame.shape[0] / frame_height

        # --- [MODIFIED] Find the closest person with hands up ---
        closest_target_keypoints = None
        min_distance = float('inf')

        if results[0].keypoints is not None:
            for keypoints in results[0].keypoints.xy:
                if is_handsup_pose(keypoints):
                    dist = estimate_distance(get_shoulder_distance(keypoints), k_dist)
                    if dist is not None and dist < min_distance:
                        min_distance = dist
                        closest_target_keypoints = keypoints

        # --- State Update and Motor Control based on the closest target ---
        if closest_target_keypoints is not None:
            target_detected_this_frame = True
            last_target_seen_time = time.time()
            if current_state != STATE_ARRIVED:
                current_state = STATE_APPROACHING
            
            estimated_m = min_distance
            if estimated_m < MIN_APPROACH_DISTANCE_M and current_state != STATE_ARRIVED:
                current_state = STATE_ARRIVED
                arrived_timer_start = time.time()
        else:
            target_detected_this_frame = False
            if current_state != STATE_ARRIVED:
                current_state = STATE_SEARCHING

        # --- Action based on State ---
        if current_state == STATE_SEARCHING:
            if time.time() - last_target_seen_time > 7.0:
                update_arduino_state(STATE_SEARCHING, MOTOR_OFF, M2_PWM_ON) # Rotate right
            else:
                update_arduino_state(STATE_SEARCHING, MOTOR_OFF, MOTOR_OFF)
        elif current_state == STATE_APPROACHING:
            x = int(closest_target_keypoints[0][0])
            center_diff_x = x - frame_center_x
            if center_diff_x < -center_threshold:
                update_arduino_state(STATE_APPROACHING, M1_PWM_ON, MOTOR_OFF)
            elif center_diff_x > center_threshold:
                update_arduino_state(STATE_APPROACHING, MOTOR_OFF, M2_PWM_ON)
            else:
                update_arduino_state(STATE_APPROACHING, M1_PWM_ON, M2_PWM_ON)
        elif current_state == STATE_ARRIVED:
            update_arduino_state(STATE_ARRIVED, MOTOR_OFF, MOTOR_OFF)
            if time.time() - arrived_timer_start > 5.0:
                current_state = STATE_SEARCHING
                last_target_seen_time = time.time()

        # --- [RESTORED] Manually draw all annotations ---
        if results[0].boxes:
            for box in results[0].boxes:
                x1, y1, x2, y2 = [int(i) for i in box.xyxy[0]]
                x1_d, y1_d, x2_d, y2_d = int(x1*scale_x), int(y1*scale_y), int(x2*scale_x), int(y2*scale_y)
                cv2.rectangle(display_frame, (x1_d, y1_d), (x2_d, y2_d), (255,0,0), 2)
                label = f"{model.names[int(box.cls)]} {box.conf[0]:.2f}"
                cv2.putText(display_frame, label, (x1_d, y1_d - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,0,0), 2)

        if results[0].keypoints is not None:
            for keypoints in results[0].keypoints.xy:
                scaled_kpts = []
                for pt in keypoints:
                    x_d, y_d = int(pt[0] * scale_x), int(pt[1] * scale_y)
                    scaled_kpts.append((x_d, y_d))
                    cv2.circle(display_frame, (x_d, y_d), 3, (0, 255, 0), -1)
                for pair in skeleton_pairs:
                    if len(keypoints) > pair[0] and len(keypoints) > pair[1]:
                        cv2.line(display_frame, scaled_kpts[pair[0]], scaled_kpts[pair[1]], (0, 255, 0), 2)
                
                if is_handsup_pose(keypoints):
                    x_d, y_d = scaled_kpts[0]
                    dist = estimate_distance(get_shoulder_distance(keypoints), k_dist)
                    cv2.putText(display_frame, "rescue sign", (x_d, y_d - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                    if dist: cv2.putText(display_frame, f"Dist: {dist:.1f}m", (x_d, y_d - 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)


        # --- Display Information ---
        current_time = time.time()
        fps = 1 / (current_time - prev_time) if prev_time != 0 else 0
        prev_time = current_time
        cv2.putText(display_frame, f"FPS: {int(fps)}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(display_frame, f"STATE: {current_state}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
        cv2.putText(display_frame, f"MODE: {active_mode}", (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.putText(display_frame, f"LUX: {current_lux:.1f}", (20, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        cv2.imshow("Dual Camera Auto-rescue System", display_frame)
	
        if cv2.waitKey(1) == ord("q"):
            update_arduino_state("SHUTDOWN", MOTOR_OFF, MOTOR_OFF)
            break

# --- Main execution block ---
if __name__ == "__main__":
    try:
        if USE_THERMAL_CAMERA:
            with Lepton3() as lepton:
                main_loop()
        else:
            main_loop()

    except Exception as e:
        print(f"An error occurred in the main execution: {e}")

    finally:
        if ser and ser.is_open:
            update_arduino_state("SHUTDOWN", MOTOR_OFF, MOTOR_OFF)
            ser.close()
        if picam2:
            picam2.stop()
        cv2.destroyAllWindows()
