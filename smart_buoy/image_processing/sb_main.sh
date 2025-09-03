#!/bin/bash


cd /home/pi/Desktop/smart-buoy/

# 가상환경활성화
source yolo_env/bin/activate

echo "GPS client will start in 3 minutes."
(sleep 180; python3 /home/pi/Desktop/smart-buoy/image_processing/buoy_client.py) &
# Store the Process ID (PID) of the background task
GPS_CLIENT_PID=$!

cd /home/pi/Desktop/smart-buoy/image_processing

while true
do
    python3 dual_camera_autodrive_lux_system.py
    
    EXIT_CODE=$?
    
    if [ $EXIT_CODE -eq 10 ]
    then
        echo "Mode change detected. Restarting"
        sleep 2
    else
        echo "Vision script exited normally. Shutting down."
        break
    fi
done

echo "Stopping GPS client (PID: $GPS_CLIENT_PID)"
kill $GPS_CLIENT_PID

echo "Shutdown"
