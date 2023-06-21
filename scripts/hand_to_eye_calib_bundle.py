#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This is used for hand-eye calibration.
"""

import numpy as np
import cv2
import rospy
from sensor_msgs import CameraInfo


def callback(info):
    global cameraMatrix
    global distCoeffs
    cameraMatrix = np.array(info.K)
    np.reshape(cameraMatrix, (3, 3))
    distCoeffs = np.array(info.D)


def estimate_pose(frames, dictionary, camera_matrix, dist_coeffs, rvec=None, tvec=None):
    dictionary = cv2.aruco.Dictionary_get(dictionary)
    board = cv2.aruco.GridBoard_create(markersX, markersY, markerLength, markerSeparation, dictionary)
    gray_scale = cv2.cvtColor(frames, cv2.COLOR_BGR2GRAY)
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.adaptiveThreshConstant = 10

    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray_scale, dictionary, parameters)
    if len(corners) > 0 and len(ids) > 0:
        success, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, camera_matrix, dist_coeffs, rvec, tvec)
        if success:
            cv2.aruco.drawDetectedMarkers(frames, corners)
            cv2.aruco.drawAxis(frames, camera_matrix, dist_coeffs, rvec, tvec, 0.01)
            cv2.imshow('Pose', frames)


if __name__ == '__main__':
    rospy.init_node("hand_to_eye_calib_bundle", anonymous=True)

    cameraPortId = rospy.get_param("camera_port_id", default=4)
    markersX = rospy.get_param("markers_x", default=5)
    markersY = rospy.get_param("markers_y", default=7)
    markerLength = rospy.get_param("marker_length", default=0.04)
    markerSeparation = rospy.get_param("marker_separation", default=0.01)
    arucoDictionary = rospy.get_param("aruco_dictionary", default="DICT_4X4_100")
    camera_info = rospy.get_param("/camera_info", default="/camera/color/camera_info")
    cameraMatrix = None
    distCoeffs = None

    rospy.Subscriber(camera_info, CameraInfo, callback, queue_size=10)

    capturer = cv2.VideoCapture(cameraPortId)
    while not rospy.is_shutdown():
        ret, frame = capturer.read()
        if ret is False:
            break
        estimate_pose(frame, arucoDictionary, cameraMatrix, distCoeffs)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    capturer.release()
    cv2.destroyAllWindows()
