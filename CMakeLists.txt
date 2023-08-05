cmake_minimum_required(VERSION 3.0.2)
project(hand_eye_calibration)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  tf2_msgs
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    geometry_msgs
    sensor_msgs
    tf2_msgs
    cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS
    scripts/automatic_calib_pose_preset.py
    scripts/aruco_bundles_detection.py
    scripts/checkerboard_detection.py
    scripts/charuco_detection.py
    scripts/automatic_hand_to_eye_calib.py
    scripts/hand_to_eye_calib.py
    scripts/easy_hand_to_eye_calib.py 
    scripts/marker_detection.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)



