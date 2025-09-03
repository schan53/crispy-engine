import cv2
from ultralytics import YOLO
import time

print("load ncnn")
model = YOLO("yolo11n_ncnn_model", task="detect")
print("complete load")

cap = cv2.VideoCapture("tcp://localhost:8888")
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)

prev_time = 0

while True:
    ret, frame = cap.read()
    print("ret:", ret)
    if not ret:
        print("fail")
        break

    results = model(frame, verbose=False)

    current_time = time.time()
    fps = 1 / (current_time - prev_time) if prev_time != 0 else 0
    prev_time = current_time

    annotated_frame = results[0].plot()

    cv2.putText(annotated_frame, f"FPS: {int(fps)}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("YOLOv11 NCNN Inference", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
