# Hand-Eye Calibration

This package is used for hand-eye calibration. **Intel Realsense D455** camera and **JAKA MiniCobo** robot are required.

## Usage Instruction

### Step 1: Print a calibration target
This package supports a variety of calibration targets, including checkerboard, single ArUco marker, ArUco bundles, single AprilTag marker and AprilTag bundles.
- [Single ArUco marker](https://chev.me/arucogen/) (Dictionary: "Original ArUco")
- [ArUco bundles](https://github.com/HenryWJL/hand_eye_calibration/blob/main/scripts/generate_aruco_bundles.py)
- [Single AprilTag marker](https://github.com/AprilRobotics/apriltag-imgs)

### Step 2: Modify the arguments in `launch` and `config`
Please follow the instructions in the documents while modifying the arguments. 

### Step 3: Start the camera
```bash
roslaunch realsense2_camera rs_camera.launch
```
### Step 4: Start pose estimation
- Checkerboard
```bash
roslaunch hand_eye_calibration checkerboard_start.launch
```  
- Single ArUco marker
```bash
roslaunch hand_eye_calibration aruco_single_start.launch
```
- ArUco bundles
```bash
roslaunch hand_eye_calibration aruco_bundle_start.launch
```
- Single AprilTag marker
```bash
roslaunch hand_eye_calibration apriltag_single_start.launch
```
- AprilTag bundles
```bash
roslaunch hand_eye_calibration apriltag_bundle_start.launch
```
- ChArUco
```bash
roslaunch hand_eye_calibration charuco_start.launch
```
### Step 5: Start the robot
```bash
roslaunch hand_eye_calibration robot_start.launch
```
### Step 6: Start calibration. 
There are two options for hand-eye calibration, one is manual, while the other is automatic (without human assistance). If you choose the manual one, you are required to move the robot with your hands while calibrating. If you select the automatic one, please [preset the poses of the robot](https://github.com/HenryWJL/hand_eye_calibration/blob/main/scripts/automatic_calib_pose_preset.py) before calibrating.
- Manual
```bash
rosrun hand_eye_calibration hand_to_eye_calib.py
```
- Automatic
```bash
rosrun hand_eye_calibration automatic_hand_to_eye_calib.py
```
