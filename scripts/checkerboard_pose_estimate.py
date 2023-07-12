#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This is used for pose estimation with checkerboard.
"""

import numpy as np
import cv2
import rospy
import time
import transforms3d as tfs
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge


def camera_info_callback(info):
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


def estimate_pose(image, size, length, camera_matrix, dist_coeffs):
    # Setting criteria for corner location refinement
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # Setting object points
    points_per_row, points_per_column = size
    objectPoints = np.zeros((points_per_row * points_per_column, 3))
    objectPoints[:, : 2] = np.mgrid[0: points_per_row, 0: points_per_column].T.reshape(-1, 2)
    objectPoints = objectPoints * length
    # Transforming the BGR images into gray-scale images
    gray_scale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Finding the corners of the checkerboard
    ret, corners = cv2.findChessboardCorners(gray_scale, size)
    if ret:
        # Corners refinement
        imagePoints = cv2.cornerSubPix(gray_scale, corners, (11, 11), (-1, -1), criteria)
        # Computing pose
        success, rvec, tvec, inliers = cv2.solvePnPRansac(objectPoints, imagePoints, camera_matrix, dist_coeffs)
        # Obtaining translation and rotation components of the pose
        T_target2cam = tvec
        R_target2cam = cv2.Rodrigues(rvec)[0]
        R_target2cam = tfs.quaternions.mat2quat(R_target2cam)
        # Transforming the pose into tf
        tf = TFMessage()
        header = tf.transforms[0].header
        pose = tf.transforms[0].transform

        header.frame_id = "/camera_link"
        header.child_frame_id = "/checkerboard"

        pose.translation.x = T_target2cam[0]
        pose.translation.y = T_target2cam[1]
        pose.translation.z = T_target2cam[2]
        pose.rotation.w = R_target2cam[0]
        pose.rotation.x = R_target2cam[1]
        pose.rotation.y = R_target2cam[2]
        pose.rotation.z = R_target2cam[3]

        return tf

    else:
        return None


if __name__ == '__main__':
    rospy.init_node("checkerboard_pose_estimate", anonymous=True)
    # Parameters and Variables
    cornersX = rospy.get_param("corners_per_row", default=5)  # The number of corners in each row
    cornersY = rospy.get_param("corners_per_column", default=7)  # The number of corners in each column
    checkerboardSize = (cornersX, cornersY)
    gridLength = rospy.get_param("grid_length", default=0.04)  # The length of each grid
    camera_info = rospy.get_param("camera_info", default="/camera/color/camera_info")  # The camera_info topic
    image_topic = rospy.get_param("image_topic", default="/camera/color/image_raw")  # The image topic
    bgrImage = None  # The OpenCV BGR images
    cameraMatrix = None  # The 3X3 camera matrix
    distCoeffs = None  # The distortion coefficients
    bridge = CvBridge()

    rospy.Subscriber(camera_info, CameraInfo, camera_info_callback, queue_size=10)
    rospy.Subscriber(image_topic, Image, image_callback, queue_size=10)
    pub = rospy.Publisher("/tf", TFMessage, queue_size=10)
    rate = rospy.Rate(5)

    rospy.sleep(1)

    while not rospy.is_shutdown():
        try:
            if not bgrImage:
                rospy.logwarn("No image data available!")
                time.sleep(1)
                continue
            if not cameraMatrix:
                rospy.logwarn("No camera matrix available!")
                time.sleep(1)
                continue
            if not distCoeffs:
                rospy.logwarn("No distortion coefficients available!")
                time.sleep(1)
                continue
            # Estimating the pose
            estimatedPose = estimate_pose(bgrImage, checkerboardSize, gridLength, cameraMatrix, distCoeffs)
            if not estimatedPose:
                rospy.logwarn("Pose estimation failed!")
                time.sleep(1)

            else:
                pub.publish(estimatedPose)
                rate.sleep()

        except rospy.ROSInterruptException:
            rospy.logwarn("No available data!")
            continue
