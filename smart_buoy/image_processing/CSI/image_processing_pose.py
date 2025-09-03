import cv2
from picamera2 import Picamera2
from ultralytics import YOLO
import time

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

model = YOLO("yolo11n-pose_ncnn_model")

prev_time = 0

while True:
    frame = picam2.capture_array()
    results = model(frame)
    current_time = time.time()
    annotated_frame = results[0].plot()
    fps = 1 / (current_time - prev_time) if prev_time != 0 else 0
    prev_time = current_time
    cv2.putText(annotated_frame, f"FPS: {int(fps)}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("IMAGE_PROCESSING", annotated_frame)
    if cv2.waitKey(1) == ord("q"):
        break

cv2.destroyAllWindows()
