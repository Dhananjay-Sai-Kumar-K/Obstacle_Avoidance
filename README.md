# 🤖 ZED2i Obstacle Avoidance with PID Control 🚧

This project demonstrates obstacle avoidance using the ZED2i stereo camera and PID control for smooth speed adjustments. It analyzes depth data to detect obstacles and provides commands for navigation.

## 🛠️ Prerequisites

* **ZED SDK:** Install the ZED SDK from [Stereolabs](https://www.stereolabs.com/developers/).
* **Python:** Python 3.x is required.
* **Libraries:** Install the necessary Python libraries:

    ```bash
    pip install opencv-python numpy pyzed-wrapper
    ```

* **ZED2i Camera:** Connect your ZED2i camera to your computer.

## ⚙️ Setup

1.  **Connect ZED2i Camera:** Connect your ZED2i camera to your computer.
2.  **Install ZED SDK:** Follow the installation instructions provided by Stereolabs.
3.  **Install Python Libraries:** Install the required Python libraries using `pip`.

## 🏃‍♂️ Running the Script

1.  **Execute the Python Script:** Run the Python script:

    ```bash
    python your_script_name.py
    ```

2.  **View the Results:** The script will open a window displaying the real-time video feed from the ZED2i camera with bounding boxes around the analyzed regions and navigation commands.
3.  **Exit:** Press the 'q' key to close the window and exit the script.

## 📄 Code Explanation

* **Import Libraries:** The script imports `sys`, `numpy`, `cv2` (OpenCV), and `pyzed.sl` (ZED SDK).
* **Initialize ZED Camera:** The ZED2i camera is initialized with HD720 resolution, 30 FPS, PERFORMANCE depth mode, and depth units set to meters.
* **Define Regions of Interest (ROI):** The script defines three regions: center, left, and right, for depth analysis.
* **PID Control:**
    * A PID controller is implemented to smoothly adjust the speed based on the distance to obstacles.
    * The `pid_control` function calculates the speed based on the error between the target distance and the current distance.
* **Depth Analysis:**
    * The script captures frames and depth maps from the ZED2i camera.
    * It calculates the minimum depth within each ROI.
    * It determines the navigation command based on the minimum depth.
* **Display Results:**
    * Bounding boxes are drawn around the ROIs.
    * The minimum depth values and navigation commands are displayed on the frame.
    * The processed frames are displayed in a window.
* **Exit:** The script exits when the 'q' key is pressed.


## 💡 Future Improvements

* **Advanced Navigation:** Implement more advanced navigation algorithms, such as path planning and obstacle avoidance.
* **Real-time Control:** Integrate the script with a robotic system to control its movement in real-time.
* **Object Tracking:** Implement object tracking to track the movement of detected obstacles.
* **3D Mapping:** Create a 3D map of the environment using the depth data.
* **Dynamic ROI:** Implement dynamic ROI resizing based on the detected obstacle.
* **Add error checking:** add more error checking for depth retrieval and camera issues.
