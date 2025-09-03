import cv2
import numpy as np
import time
import math
from ultralytics import YOLO
from flirpy.camera.lepton import Lepton

model = YOLO("yolo11n-pose_ncnn_model")

prev_time = 0

def is_handsup_pose(keypoints):
    try:
        nose_y = keypoints[0][1]
        lshoulder_y = keypoints[5][1]
        rshoulder_y = keypoints[6][1]
        lwrist_y = keypoints[9][1]
        rwrist_y = keypoints[10][1]
        if lwrist_y < nose_y and rwrist_y < nose_y and lwrist_y < lshoulder_y and rwrist_y < rshoulder_y:
            return True
    except IndexError:
        return False
    return False

def get_shoulder_distance(keypoints):
    try:
        x1, y1 = keypoints[5]
        x2, y2 = keypoints[6]
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    except IndexError:
        return 0

def estimate_distance(shoulder_pixel):
    if shoulder_pixel == 0:
        return None
    k = 120 # 
    return round(k / shoulder_pixel, 2)

# 4. Main Loop
try:
    with Lepton() as camera:
        while True:

            thermal_image_16bit = camera.grab()
            
            if thermal_image_16bit is None:
                continue

            # 
            cv2.normalize(thermal_image_16bit, thermal_image_16bit, 0, 65535, cv2.NORM_MINMAX)
            thermal_image_8bit = np.right_shift(thermal_image_16bit, 8).astype(np.uint8)
            thermal_bgr = cv2.cvtColor(thermal_image_8bit, cv2.COLOR_GRAY2BGR)

            # YOLO 
            results = model(thermal_bgr, verbose=False)
            
            annotated_frame = cv2.applyColorMap(thermal_bgr, cv2.COLORMAP_JET)

            # 
            if results[0].boxes:
                for box in results[0].boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            if results[0].keypoints and results[0].keypoints.xy.numel() > 0:
                for keypoints in results[0].keypoints.xy:
                    for i, (x, y) in enumerate(keypoints):
                        if x > 0 and y > 0:
                           cv2.circle(annotated_frame, (int(x), int(y)), 2, (0, 255, 255), -1)
                    if is_handsup_pose(keypoints):
                        x, y = int(keypoints[0][0]), int(keypoints[0][1])
                        cv2.putText(annotated_frame, "RESCUE SIGN", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)


            current_time = time.time()
            fps = 1 / (current_time - prev_time) if prev_time != 0 else 0
            prev_time = current_time
            cv2.putText(annotated_frame, f"FPS: {int(fps)}", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # Lepton 3 
            display_frame = cv2.resize(annotated_frame, (640, 480), interpolation=cv2.INTER_NEAREST)
            cv2.imshow("Thermal Pose Detection (Lepton 3)", display_frame)

            if cv2.waitKey(1) == ord("q"):
                break
finally:
    cv2.destroyAllWindows()
