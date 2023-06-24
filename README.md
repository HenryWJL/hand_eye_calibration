# Hand-Eye Calibration

This package is used for hand-eye calibration. **Intel Realsense D455** camera and **JAKA MiniCobo** robot are required.

## Usage Instruction

### Step 1: Print a calibration target
This package supports a variety of calibration targets, including checkerboard, single ArUco marker, ArUco bundles, single AprilTag marker and AprilTag bundles.
- [Single ArUco marker](https://chev.me/arucogen/) (Dictionary: "Original ArUco")
- [ArUco bundles](https://github.com/HenryWJL/hand_eye_calibration/blob/main/scripts/generate_aruco_bundles.py)
- [Single AprilTag marker](https://github.com/AprilRobotics/apriltag-imgs)

### Step 2: Modify the arguments in the launch files
Please follow the instructions in the launch files and modify the arguments. 

### Step 3: Start the camera node
```bash
roslaunch realsense2_camera rs_camera.launch
```
### Step 4: Start the pose estimation node
- Single ArUco marker
```bash
roslaunch hand_eye_calibration aruco_single.launch
```
- ArUco bundles
```bash
roslaunch hand_eye_calibration aruco_bundle.launch
```
- AprilTag
```bash
roslaunch hand_eye_calibration apriltag_single.launch
```
### Step 5: Start the robot node
```bash
roslaunch hand_eye_calibration robot_start.launch
```
### Step 6: Start the calibration node. 
```bash
rosrun hand_eye_calibration hand_to_eye_calib_single.py
```
When everything is prepared, move the calibration target and type 'r' in the terminal to record calibration data. After recording more than two data, type 'c' in the terminal to calculate the calibration result. After a few seconds, you can view the result on the screen. If you want to save the result, type 's' in the terminal. This will save the result as a **.yaml** file.
