#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This is used for hand-eye calibration with ArUco bundles.
"""

import numpy as np
import cv2
import rospy
import time
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge


def info_callback(info):
    global cameraMatrix
    global distCoeffs
    cameraMatrix = np.array(info.K)
    np.reshape(cameraMatrix, (3, 3))
    distCoeffs = np.array(info.D)


def image_callback(image):
    global rgbImage
    rgbImage = bridge.imgmsg_to_cv2(image, "bgr8")


def estimate_pose(image, dictionary, camera_matrix, dist_coeffs, rvec=None, tvec=None):
    dictionary = cv2.aruco.Dictionary_get(dictionary)
    board = cv2.aruco.GridBoard_create(markersX, markersY, markerLength, markerSeparation, dictionary)
    gray_scale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.adaptiveThreshConstant = 10
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray_scale, dictionary, parameters=parameters)
    if len(corners) > 0 and len(ids) > 0:
        success, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, camera_matrix, dist_coeffs, rvec, tvec)
        if success:
            cv2.aruco.drawDetectedMarkers(image, corners)
            cv2.aruco.drawAxis(image, camera_matrix, dist_coeffs, rvec, tvec, 0.01)
            cv2.imshow('Pose', image)
        else:
            rospy.logwarn("Failed to estimate the pose!")
    else:
        rospy.logwarn("Failed to detect the markers!")


if __name__ == '__main__':
    rospy.init_node("hand_to_eye_calib_bundle", anonymous=True)

    markersX = rospy.get_param("markers_x", default=5)
    markersY = rospy.get_param("markers_y", default=7)
    markerLength = rospy.get_param("marker_length", default=0.04)
    markerSeparation = rospy.get_param("marker_separation", default=0.01)
    arucoDictionary = rospy.get_param("aruco_dictionary", default=cv2.aruco.DICT_4X4_100)
    camera_info = rospy.get_param("/camera_info", default="/camera/color/camera_info")
    image_raw = rospy.get_param("/image_raw", default="/camera/color/image_raw")

    rgbImage = None
    bridge = CvBridge()
    cameraMatrix = None
    distCoeffs = None

    rospy.Subscriber(camera_info, CameraInfo, info_callback, queue_size=10)
    rospy.Subscriber(image_raw, Image, image_callback, queue_size=10)
    rospy.sleep(1)
    while not rospy.is_shutdown():
        if rgbImage is None:
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

        estimate_pose(rgbImage, arucoDictionary, cameraMatrix, distCoeffs)
        time.sleep(1)
