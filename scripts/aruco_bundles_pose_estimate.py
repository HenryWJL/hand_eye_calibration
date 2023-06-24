#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This is used for hand-eye calibration with ArUco bundles. Before running this program,
please print out the ArUco board and revise the parameters in the relevant launch files.
"""

import numpy as np
import cv2
import rospy
import time
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge


def info_callback(info):
    # Obtaining the camera matrix and the distortion coefficients
    global cameraMatrix
    global distCoeffs
    cameraMatrix = np.array(info.K)
    np.reshape(cameraMatrix, (3, 3))
    distCoeffs = np.array(info.D)


def image_callback(image):
    # Acquiring and transforming the ROS Image messages into OpenCV BGR images
    global bgrImage
    bgrImage = bridge.imgmsg_to_cv2(image, "bgr8")


def estimate_pose(image, dictionary, camera_matrix, dist_coeffs, rvec=None, tvec=None):
    # Declaring a dictionary object and a board object
    dictionary = cv2.aruco.Dictionary_get(dictionary)
    board = cv2.aruco.GridBoard_create(markersX, markersY, markerLength, markerSeparation, dictionary)
    # Transforming the BGR images into gray-scale images
    gray_scale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Setting detector parameters
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.adaptiveThreshConstant = 10
    # Detecting markers
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray_scale, dictionary, parameters=parameters)
    # If at least one marker is detected, estimating the pose
    if len(corners) > 0 and len(ids) > 0:
        success, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, camera_matrix, dist_coeffs, rvec, tvec)
        if success:
            # Drawing quads and axes on the image
            cv2.aruco.drawDetectedMarkers(image, corners)
            cv2.aruco.drawAxis(image, camera_matrix, dist_coeffs, rvec, tvec, 0.01)
            cv2.imshow('Pose', image)
        else:
            rospy.logwarn("Failed to estimate the pose!")
    else:
        rospy.logwarn("Failed to detect the markers!")


if __name__ == '__main__':
    rospy.init_node("hand_to_eye_calib_bundle", anonymous=True)
    # Parameters and Variables
    markersX = rospy.get_param("/markers_x", default=5)  # The number of markers in each row
    markersY = rospy.get_param("/markers_y", default=7)  # The number of markers in each column
    markerLength = rospy.get_param("/marker_length", default=0.04)  # The length of each marker
    markerSeparation = rospy.get_param("/marker_separation", default=0.01)  # The separation between any two markers
    arucoDictionary = rospy.get_param("/aruco_dictionary", default=cv2.aruco.DICT_4X4_100)  # The marker dictionary
    camera_info = rospy.get_param("/camera_info", default="/camera/color/camera_info")  # The "/camera_info" topic
    image_raw = rospy.get_param("/image_raw", default="/camera/color/image_raw")  # The "/image_raw" topic
    bgrImage = None  # The OpenCV BGR images
    cameraMatrix = None  # The 3X3 camera matrix
    distCoeffs = None  # The distortion coefficients
    bridge = CvBridge()

    rospy.Subscriber(camera_info, CameraInfo, info_callback, queue_size=10)
    rospy.Subscriber(image_raw, Image, image_callback, queue_size=10)

    rospy.sleep(1)

    while not rospy.is_shutdown():
        try:
            if bgrImage is None:
                rospy.logwarn("No image data available!")
                time.sleep(1)
                continue
            if cameraMatrix is None:
                rospy.logwarn("No camera matrix available!")
                time.sleep(1)
                continue
            if distCoeffs is None:
                rospy.logwarn("No distortion coefficients available!")
                time.sleep(1)
                continue
            # Estimating the pose using acquired images
            estimate_pose(bgrImage, arucoDictionary, cameraMatrix, distCoeffs)

        except rospy.ROSInterruptException:
            rospy.logwarn("No available data!")
            continue
