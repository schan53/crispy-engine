#!/bin/bash

# This script starts the main vision system and the GPS client.

# Navigate to the project's root directory
cd /home/pi/Desktop/smart-buoy/

# Activate the Python virtual environment
source yolo_env/bin/activate

# --- Start the GPS client in the background after a 4-minute delay ---
echo "GPS client will start in 4 minutes."
# The parentheses create a subshell, so sleep and python run sequentially in the background
(sleep 180; python3 /home/pi/Desktop/smart-buoy/image_processing/buoy_client.py) &
# Store the Process ID (PID) of the background task
GPS_CLIENT_PID=$!


# --- Start the main camera/motor control loop ---
# Navigate to the directory of the main script
cd /home/pi/Desktop/smart-buoy/image_processing

while true
do
    # Run the main python script
    python3 dual_camera_autodrive_lux_system.py
    
    # Get the exit code from the python script
    EXIT_CODE=$?
    
    # Check if the exit code is our special code for restarting
    if [ $EXIT_CODE -eq 10 ]
    then
        echo "Mode change detected. Restarting script in 2 seconds..."
        sleep 2
    else
        # If it's any other code, break the loop and exit
        echo "Vision script exited normally. Shutting down."
        break
    fi
done

# --- Cleanup ---
# When the main loop is finished, stop the background GPS client as well.
echo "Stopping GPS client (PID: $GPS_CLIENT_PID)..."
kill $GPS_CLIENT_PID

echo "Shutdown complete."
