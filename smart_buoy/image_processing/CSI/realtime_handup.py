import cv2
from picamera2 import Picamera2
from ultralytics import YOLO
import time
import math

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

model = YOLO("yolo11s-pose_ncnn_model")

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
    
    
while True:
    frame = picam2.capture_array()
    results = model(frame)
    current_time = time.time()
    annotated_frame = results[0].plot()
    fps = 1 / (current_time - prev_time) if prev_time != 0 else 0
    prev_time = current_time
    cv2.putText(annotated_frame, f"FPS: {int(fps)}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    if results[0].keypoints is not None:
        for keypoints in results[0].keypoints.xy:
            if is_handsup_pose(keypoints):
                x, y = int(keypoints[0][0]), int(keypoints[0][1])
                cv2.putText(annotated_frame, "rescue sign", (x,y-30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                shoulder_pixel = get_shoulder_distance(keypoints)
                estimated_m = estimate_distance(shoulder_pixel)
                if estimated_m:
                    cv2.putText(annotated_frame, f"Distance: {estimated_m}m", (x, y-60),cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
    cv2.imshow("REALTIME_RESCUE", annotated_frame)
    if cv2.waitKey(1) == ord("q"):
        break

cv2.destroyAllWindows()
