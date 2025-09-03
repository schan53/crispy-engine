import cv2
from picamera2 import Picamera2
from ultralytics import YOLO
import time
import math
import serial

try:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    time.sleep(2)
    print("Arduino connected successfully.")
except Exception as e:
    print(f"Failed to connect to Arduino: {e}")
    ser = None

def set_motors(m1_pwm, m2_pwm):
    if ser is not None:
        command = "{},{}\n".format(m1_pwm, m2_pwm)
        ser.write(command.encode())
	
def send_command(cmd):
    if ser is not None:
        ser.write((cmd + '\n').encode())
        time.sleep(0.05)

M1_PWM_ON = 1150
M2_PWM_ON = 1080
MOTOR_OFF = 1000

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
time.sleep(1)

_, _, frame_width, frame_height = picam2.camera_properties['ScalerCropMaximum']
frame_center_x = frame_width / 2
print(f"Frame dimensions set to: {frame_width} x {frame_height}")

model = YOLO("yolo11n-pose_ncnn_model")
prev_time = 0

def is_handsup_pose(keypoints):
    #keypoint index: 0 = nose, 5 = lshoulder, 6 = rshoulder, 7 = lelbow, 8 = relbow, 9 = lwrist, 10 = rwrist
    nose_y = keypoints[0][1]
    lshoulder_y = keypoints[5][1]
    rshoulder_y = keypoints[6][1]
    lwrist_y = keypoints[9][1]
    rwrist_y = keypoints[10][1]
    if lwrist_y < nose_y and rwrist_y < nose_y and lwrist_y < lshoulder_y and rwrist_y <rshoulder_y:
        return True
    return False
    
def get_shoulder_distance(keypoints):
    x1, y1 = keypoints[5]
    x2, y2 = keypoints[6]
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
def estimate_distance(shoulder_pixel):
    if shoulder_pixel == 0:
        return None
    k = 120 
    return round(k / shoulder_pixel, 2)
    
try:
    while True:
        frame = picam2.capture_array()
        
        frame_height, frame_width, _ = frame.shape
        frame_center_x = frame_width / 2
 
	results = model(frame)
        annotated_frame = results[0].plot()
        
        target_detected = False
        if results[0].keypoints is not None:
            for keypoints in results[0].keypoints.xy:
                if is_handsup_pose(keypoints):
                    target_detected = True
                    x, y = int(keypoints[0][0]), int(keypoints[0][1])
                    cv2.putText(annotated_frame, "rescue sign", (x,y-30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                    
                    target_x = x
                    center_diff_x = target_x - frame_center_x
                    CENTER_THRESHOLD = 30 

                    print(f"Target detected. Center_Diff_X: {center_diff_x:.1f}")
                    if center_diff_x < -CENTER_THRESHOLD:
                        print("Action: Turning LEFT")
                        send_command(f'M1{M1_PWM_ON}')
                        send_command(f'M2{MOTOR_OFF}')
                    elif center_diff_x > CENTER_THRESHOLD:
                        print("Action: Turning RIGHT")
                        send_command(f'M1{MOTOR_OFF}')
                        send_command(f'M2{M2_PWM_ON}')
                    else:
                        print("Action: Moving FORWARD")
                        send_command(f'M1{M1_PWM_ON}')
                        send_command(f'M2{M2_PWM_ON}')
                    shoulder_pixel = get_shoulder_distance(keypoints)
                    estimated_m = estimate_distance(shoulder_pixel)
                    if estimated_m:
                        cv2.putText(annotated_frame, f"Distance: {estimated_m}m", (x, y-60),cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
                    
                    break
        
        if not target_detected:
            print("Action: No target, STOPPING")
            send_command('STOP')

        cv2.imshow("RGB Auto-tracking System", annotated_frame)
        if cv2.waitKey(1) == ord("q"):
            send_command('STOP')
            break

finally:
    if ser and ser.is_open:
        send_command('STOP')
        ser.close()
        print("Serial port closed.")
    cv2.destroyAllWindows()
