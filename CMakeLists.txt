cmake_minimum_required(VERSION 3.0.2)
project(hand_eye_calibration)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  tf2_msgs
)

catkin_package(
  LIBRARIES aruco_utils
  CATKIN_DEPENDS rospy geometry_msgs sensor_msgs tf2_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/hand_to_eye_calib_single.py scripts/automatic_hand_to_eye_calib_single.py
  scripts/aruco_bundles_pose_estimate.py scripts/automatic_hand_to_eye_calib_bundle.py
  scripts/easy_hand_to_eye_calib.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



