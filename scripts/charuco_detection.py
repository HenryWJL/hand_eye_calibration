#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This is used for pose estimation with ChArUco.
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
    cameraMatrix = np.reshape(cameraMatrix, [3, 3])
    distCoeffs = np.array(info.D)


def image_callback(image):
    # Acquiring and transforming the ROS Image messages into OpenCV BGR images
    global bgrImage
    bgrImage = bridge.imgmsg_to_cv2(image, "bgr8")


def estimate_pose(image, aruco_dictionary, charuco_board, camera_matrix, dist_coeffs, rvec=None, tvec=None):
    # Transforming the BGR images into gray-scale images
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Setting detector parameters
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.adaptiveThreshConstant = 10
    # Detecting markers
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dictionary, parameters=parameters)
    # If at least one marker is detected, estimating the pose
    if len(corners) > 0 and len(ids) > 0:
        charucoCorners, charucoIds = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, charuco_board)
        if len(charucoCorners) > 0 and len(charucoIds) > 0:
            ret, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, charuco_board,
                                                                 camera_matrix, dist_coeffs, rvec, tvec)
            if ret:
                # Drawing quads and axes on the image
                cv2.aruco.drawDetectedCornersCharuco(image, charucoCorners, charucoIds, (255, 0, 0))
                cv2.aruco.drawAxis(image, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
                image = bridge.cv2_to_imgmsg(image, "bgr8")
                # Obtaining the translational and the rotational components of the pose
                T_target2cam = tvec
                R_target2cam = cv2.Rodrigues(rvec)[0]
                R_target2cam = tfs.quaternions.mat2quat(R_target2cam)
                # Transforming the pose into tf message
                tf = TFMessage()
                transforms = TransformStamped()

                transforms.header.frame_id = "/camera_link"
                transforms.child_frame_id = "/charuco_board"
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

    else:
        return None, None


if __name__ == '__main__':
    rospy.init_node("charuco_detection", anonymous=True)
    # Parameters and Variables
    squaresX = rospy.get_param("charuco_detection/squares_per_row",
                               default=4)  # The number of squares in each row
    squaresY = rospy.get_param("charuco_detection/squares_per_column",
                               default=4)  # The number of squares in each column
    squareLength = rospy.get_param("charuco_detection/square_length",
                                   default=0.04)  # The length of each square
    markerLength = rospy.get_param("charuco_detection/marker_length",
                                   default=0.03)  # The length of each marker
    dictionary = rospy.get_param("charuco_detection/aruco_dictionary",
                                 default=cv2.aruco.DICT_6X6_250)  # The marker dictionary
    camera_info = rospy.get_param("charuco_detection/camera_info",
                                  default="/camera/color/camera_info")  # The camera_info topic
    image_topic = rospy.get_param("charuco_detection/image_topic",
                                  default="/camera/color/image_raw")  # The image topic
    bgrImage = None  # The OpenCV BGR images
    cameraMatrix = None  # The 3X3 camera matrix
    distCoeffs = None  # The distortion coefficients
    bridge = CvBridge()

    rospy.Subscriber(camera_info, CameraInfo, camera_info_callback, queue_size=10)
    rospy.Subscriber(image_topic, Image, image_callback, queue_size=10)
    tf_pub = rospy.Publisher("/tf", TFMessage, queue_size=10)
    image_pub = rospy.Publisher("/charuco_detections_image", Image, queue_size=10)
    rate = rospy.Rate(5)

    # Creating a dictionary and a board
    arucoDictionary = cv2.aruco.Dictionary_get(dictionary)
    charucoBoard = cv2.aruco.CharucoBoard_create(squaresX, squaresY, squareLength, markerLength, arucoDictionary)

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
            image_msg, estimatedPose = estimate_pose(bgrImage, arucoDictionary, charucoBoard, cameraMatrix, distCoeffs)
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
