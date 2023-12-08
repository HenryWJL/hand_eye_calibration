# Hand-Eye Calibration

This package is used for hand-eye calibration. **Intel Realsense D455** camera and **JAKA MiniCobo** manipulator are required.

## Usage

### Step 1: Print a calibration target
This package supports a variety of calibration targets, including checkerboard, single ArUco marker, ArUco bundles, single AprilTag marker, AprilTag bundles and ChArUco marker. You can find different types of markers through the links below. You can also use our provided Python scripts to generate custom markers.
- [Single ArUco marker](https://chev.me/arucogen/)
- [ArUco bundles](https://github.com/HenryWJL/hand_eye_calibration/blob/main/scripts/generate_aruco_bundles.py)
- [Single AprilTag marker](https://github.com/AprilRobotics/apriltag-imgs)

### Step 2: Modify the documents in `launch` and `config` 

### Step 3: Enable the camera
```bash
roslaunch realsense2_camera rs_camera.launch
```
### Step 4: Marker detection and pose estimation
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
### Step 5: Enable the manipulator
```bash
roslaunch hand_eye_calibration robot_start.launch
```
### Step 6: Hand-eye calibration. 
There are two options for hand-eye calibration: manual and automatic (without human assistance). If you choose the former one, please move the manipulator with your hands during calibration. If you choose the latter one, please [predefine the trajectories of the manipulator](https://github.com/HenryWJL/hand_eye_calibration/blob/main/scripts/automatic_calib_pose_preset.py) before calibration.
- Manual
```bash
rosrun hand_eye_calibration hand_to_eye_calib.py
```
- Automatic
```bash
rosrun hand_eye_calibration automatic_hand_to_eye_calib.py
```
