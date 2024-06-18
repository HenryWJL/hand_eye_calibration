#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This is used for automatic hand-eye calibration with either an
ArUco marker or an AprilTag marker.
"""

import rospy
import numpy as np
import cv2
import transforms3d as tfs
from tf2_msgs.msg import TFMessage
import time
import yaml
import jkrc


def get_end2base_mat(pose):
    # Calculating the gripper-to-base transformation matrix
    tran = np.array((pose[0], pose[1], pose[2]))
    rot = tfs.euler.euler2mat(pose[3], pose[4], pose[5])
    return rot, tran


def get_target2cam_mat(pose):
    # Calculating the target-to-camera transformation matrix
    pose = pose.transforms[0].transform
    tran = np.array(([pose.translation.x], [pose.translation.y], [pose.translation.z]))
    rot = tfs.quaternions.quat2mat((pose.rotation.w, pose.rotation.x,
                                    pose.rotation.y, pose.rotation.z))
    return rot, tran


def callback(pose):
    # Obtaining the pose of the marker
    global target_pose
    if pose is None:
        rospy.logwarn("No ArUco or AprilTag data!")

    else:
        target_pose = pose


if __name__ == '__main__':
    rospy.init_node('automatic_hand_to_eye_calib', anonymous=True)
    rospy.Subscriber('/tf2', TFMessage, callback, queue_size=10)
    robot_ip = rospy.get_param("/robot_ip", default="192.168.200.100")
    # Starting the robot
    robot = jkrc.RC(robot_ip)
    robot.login()
    robot.enable_robot()
    # Variables
    target_pose = None  # The pose of the marker target
    R_end2base_samples = []  # The rotation components of end-to-base matrices
    T_end2base_samples = []  # The translation components of end-to-base matrices
    R_target2cam_samples = []  # The rotation components of target-to-camera matrices
    T_target2cam_samples = []  # The translation components of target-to-camera matrices
    sample_number = 0  # The number of recorded samples
    cam2base = None  # The camera-to-base matrix
    # Obtaining 25 joint poses
    with open('../yaml/pose_for_automatic_calib.yaml', 'r') as f:
        joint_pose = yaml.load(f.read(), Loader=yaml.FullLoader)
        joint_pose = np.array(joint_pose)

    rospy.sleep(1)

    while not rospy.is_shutdown():
        try:
            ret = robot.joint_move(joint_pose[sample_number, :].tolist(), 0, True, 2)
            time.sleep(1)
            if ret[0] == 0:
                ret = robot.get_tcp_position()
                if ret[0] == 0:
                    end_pose = ret[1]
                    # Recording target-to-camera matrix data
                    (R_target2cam, T_target2cam) = get_target2cam_mat(target_pose)
                    R_target2cam_samples.append(R_target2cam)
                    T_target2cam_samples.append(T_target2cam)
                    # Recording end-to-base matrix data
                    (R_end2base, T_end2base) = get_end2base_mat(end_pose)
                    R_end2base_samples.append(R_end2base)
                    T_end2base_samples.append(T_end2base)

                    sample_number += 1
                    print(f"{sample_number} samples have been recorded")

                else:
                    rospy.logwarn("Failed to get the end pose!")
                    print("Failed to get the end pose!")
                    continue

            else:
                rospy.logerr("Failed to move to the target position!")
                print("Failed to move to the target position!")
                continue

            if sample_number == 25:
                # Calculating the camera-to-base matrix
                R_cam2base, T_cam2base = cv2.calibrateHandEye(R_end2base_samples, T_end2base_samples,
                                                              R_target2cam_samples, T_target2cam_samples,
                                                              cv2.CALIB_HAND_EYE_TSAI)
                cam2base = np.column_stack((R_cam2base, T_cam2base))
                cam2base = np.row_stack((cam2base, np.array([0, 0, 0, 1])))
                print(cam2base)  # Transformation matrix
                R_cam2base_quaternions = tfs.quaternions.mat2quat(R_cam2base)
                cam2base_quaternions = np.concatenate((T_cam2base.reshape(3),
                                                       R_cam2base_quaternions.reshape(4)), axis=0)
                print(cam2base_quaternions.tolist())  # Quaternions
                with open('../yaml/camera_to_base_matrix.yaml', 'w', encoding='utf-8') as f:
                    yaml.dump(data=cam2base.tolist(), stream=f)
                print("The matrix has been successfully saved")
                robot.logout()
                break

        except rospy.ROSInterruptException:
            rospy.logwarn("No available data!")
            continue
