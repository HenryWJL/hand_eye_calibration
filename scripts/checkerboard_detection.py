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
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge


def camera_info_callback(info):
    # Obtaining the camera matrix and the distortion coefficients
    global cameraMatrix
    global distCoeffs
    cameraMatrix = np.array(info.K)
    cameraMatrix = np.reshape(cameraMatrix, (3, 3))
    distCoeffs = np.array(info.D)


def image_callback(image):
    # Acquiring and transforming the ROS Image messages into OpenCV BGR images
    global bgrImage
    bgrImage = bridge.imgmsg_to_cv2(image, "bgr8")


def draw_axis(img, corners, imgpts):
    # Convert np.float32 to int32
    imgpts = imgpts.astype(int)
    corners = corners.astype(int)
    # Drawing axes
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 3)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 3)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 3)
    return img


def estimate_pose(image, size, length, camera_matrix, dist_coeffs):
    # Setting criteria for corner location refinement
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # Setting object points
    points_per_row, points_per_column = size
    objectPoints = np.zeros((points_per_row * points_per_column, 3))
    objectPoints[:, : 2] = np.mgrid[0: points_per_row, 0: points_per_column].T.reshape(-1, 2)
    objectPoints = objectPoints * length
    # Transforming the BGR images into gray-scale images
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Finding the corners of the checkerboard
    ret, corners = cv2.findChessboardCorners(gray, size)
    if ret:
        # Corners refinement
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        # Computing pose
        ret, rvec, tvec = cv2.solvePnP(objectPoints, corners, camera_matrix, dist_coeffs)
        if ret:
            # Drawing axes on the image
            axis = np.array([[0.05, 0, 0], [0, 0.05, 0], [0, 0, -0.05]], dtype=np.float32).reshape(-1, 3)
            imagePoints, jacobian = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)
            image = draw_axis(image, corners, imagePoints)
            image = bridge.cv2_to_imgmsg(image, "bgr8")
            # Obtaining translation and rotation components of the pose
            T_target2cam = tvec
            R_target2cam = cv2.Rodrigues(rvec)[0]
            R_target2cam = tfs.quaternions.mat2quat(R_target2cam)
            # Transforming the pose into tf message
            tf = TFMessage()
            transforms = TransformStamped()

            transforms.header.frame_id = "/camera_link"
            transforms.child_frame_id = "/checkerboard"
            transforms.transform.translation.x = T_target2cam[0]
            transforms.transform.translation.y = T_target2cam[1]
            transforms.transform.translation.z = T_target2cam[2]
            transforms.transform.rotation.w = R_target2cam[0]
            transforms.transform.rotation.x = R_target2cam[1]
            transforms.transform.rotation.y = R_target2cam[2]
            transforms.transform.rotation.z = R_target2cam[3]

            tf.transforms.append(transforms)

            return image, tf

        else:
            return None, None

    else:
        return None, None


if __name__ == '__main__':
    rospy.init_node("checkerboard_detection", anonymous=True)
    # Parameters and Variables
    cornersX = rospy.get_param("checkerboard_detection/corners_per_row",
                               default=5)  # The number of corners in each row
    cornersY = rospy.get_param("checkerboard_detection/corners_per_column",
                               default=7)  # The number of corners in each column
    checkerboardSize = (cornersX, cornersY)
    gridLength = rospy.get_param("checkerboard_detection/grid_length",
                                 default=0.04)  # The length of each grid
    camera_info = rospy.get_param("checkerboard_detection/camera_info",
                                  default="/camera/color/camera_info")  # The camera_info topic
    image_topic = rospy.get_param("checkerboard_detection/image_topic",
                                  default="/camera/color/image_raw")  # The image topic
    bgrImage = None  # The OpenCV BGR images
    cameraMatrix = None  # The 3X3 camera matrix
    distCoeffs = None  # The distortion coefficients
    bridge = CvBridge()

    rospy.Subscriber(camera_info, CameraInfo, camera_info_callback, queue_size=10)
    rospy.Subscriber(image_topic, Image, image_callback, queue_size=10)
    tf_pub = rospy.Publisher("/tf", TFMessage, queue_size=10)
    image_pub = rospy.Publisher("/checkerboard_detections_image", Image, queue_size=10)
    rate = rospy.Rate(5)

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
            # Estimating the pose
            image_msg, estimatedPose = estimate_pose(bgrImage, checkerboardSize, gridLength, cameraMatrix, distCoeffs)
            if estimatedPose is None and image_msg is None:
                rospy.logwarn("Detection failed!")
                time.sleep(1)

            else:
                image_pub.publish(image_msg)
                tf_pub.publish(estimatedPose)
                rate.sleep()

        except rospy.ROSInterruptException:
            rospy.logwarn("No available data!")
            continue
