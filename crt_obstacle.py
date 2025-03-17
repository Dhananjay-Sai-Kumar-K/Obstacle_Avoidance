import sys
import numpy as np
import cv2
import pyzed.sl as sl

# Initialize ZED camera
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.camera_fps = 30
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
init_params.coordinate_units = sl.UNIT.METER

if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("Failed to open ZED Camera")
    sys.exit(1)

# Get camera info
cam_info = zed.get_camera_information()
width, height = cam_info.camera_configuration.resolution.width, cam_info.camera_configuration.resolution.height
center_x, center_y = width // 2, height // 2 + 50  # Lower bounding box

# Define ROI
region_size = 300  # Smaller bounding box
half_region = region_size // 2

image, depth = sl.Mat(), sl.Mat()

# PID Control Variables
prev_error, integral = 0, 0
Kp, Ki, Kd = 200, 1, 50  # PID tuning parameters

def pid_control(target, current):
    """PID speed control for smooth movement."""
    global prev_error, integral
    error = target - current
    integral += error
    derivative = error - prev_error
    prev_error = error
    return max(50, min(255, Kp * error + Ki * integral + Kd * derivative))  # Limit speed range

while True:
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        # Get images
        zed.retrieve_image(image, sl.VIEW.LEFT)
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
        
        depth_cv = np.nan_to_num(depth.get_data(), nan=0.0)
        image_cv = image.get_data()
        
        # Define regions (Center, Left, Right)
        center_region = depth_cv[center_y-half_region:center_y+half_region, center_x-half_region:center_x+half_region]
        left_region = depth_cv[center_y-half_region:center_y+half_region, center_x-2*half_region:center_x-half_region]
        right_region = depth_cv[center_y-half_region:center_y+half_region, center_x+half_region:center_x+2*half_region]
        
        # Compute min depth for each region
        center_distance = np.min(center_region[(center_region > 0.1) & (center_region <= 5.0)], initial=np.inf)
        left_distance = np.min(left_region[(left_region > 0.1) & (left_region <= 5.0)], initial=np.inf)
        right_distance = np.min(right_region[(right_region > 0.1) & (right_region <= 5.0)], initial=np.inf)
        
        # Get minimum of all three
        min_distance = min(center_distance, left_distance, right_distance)
        
        # Draw bounding boxes
        frame = image.get_data()
        cv2.rectangle(frame, (center_x-half_region, center_y-half_region), 
                      (center_x+half_region, center_y+half_region), (0, 255, 0), 2)
        cv2.rectangle(frame, (center_x-2*half_region, center_y-half_region), 
                      (center_x-half_region, center_y+half_region), (255, 0, 0), 2)
        cv2.rectangle(frame, (center_x+half_region, center_y-half_region), 
                      (center_x+2*half_region, center_y+half_region), (255, 0, 0), 2)
        
        # Obstacle Avoidance Logic
        safe_distance, turn_distance, slow_down_distance = 1.0, 0.5, 1.5
        command = "Moving Forward"
        speed = 255

        if min_distance < turn_distance:
            command = "STOP & Turn Left" if left_distance > right_distance else "STOP & Turn Right"
        elif min_distance < safe_distance:
            command = "STOP"
        elif min_distance < slow_down_distance:
            speed = pid_control(1.5, min_distance)
            command = f"Slow Down (Speed: {speed})"

        # Overlay text on camera feed
        cv2.putText(image_cv, f"Center Dist: {center_distance:.2f}m", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(image_cv, f"Left Dist: {left_distance:.2f}m", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.putText(image_cv, f"Right Dist: {right_distance:.2f}m", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(image_cv, f"Command: {command}", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        cv2.imshow("ZED Camera Feed", image_cv)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Cleanup
zed.close()
cv2.destroyAllWindows()

