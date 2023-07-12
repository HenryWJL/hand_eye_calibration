#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This is used for hand-eye calibration with either an ArUco marker or
an AprilTag marker.
"""

import rospy
import numpy as np
import cv2
import transforms3d as tfs
from geometry_msgs.msg import TwistStamped
from tf2_msgs.msg import TFMessage
import time
import yaml


def get_end2base_mat(pose):
    # Calculating the gripper-to-base transformation matrix
    tran = np.array((pose.twist.linear.x, pose.twist.linear.y, pose.twist.linear.z))
    rot = tfs.euler.euler2mat(pose.twist.angular.x, pose.twist.angular.y, pose.twist.angular.z)
    return rot, tran


def get_target2cam_mat(pose):
    # Calculating the ArUco-target-to-camera transformation matrix
    pose = pose.transforms[0].transform
    tran = np.array(([pose.translation.x], [pose.translation.y], [pose.translation.z]))
    rot = tfs.quaternions.quat2mat((pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z))
    return rot, tran


def end_pose_callback(pose):
    # Obtaining the pose of the end-effector
    global end_pose
    if pose is None:
        rospy.logwarn("No robot pose data!")

    else:
        end_pose = pose


def target_pose_callback(pose):
    # Acquiring the pose of the marker
    global target_pose
    if pose is None:
        rospy.logwarn("No ArUco or AprilTag data!")

    else:
        target_pose = pose


if __name__ == '__main__':
    rospy.init_node('hand_to_eye_calib', anonymous=True)
    rospy.Subscriber('/robot_driver/tool_point', TwistStamped, end_pose_callback, queue_size=10)
    rospy.Subscriber('/tf', TFMessage, target_pose_callback, queue_size=10)
    # Variables
    end_pose = None  # The pose of robot's end
    target_pose = None  # The pose of ArUco target
    cam2base = None  # The camera-to-base transformation matrix
    R_end2base_samples = []  # The rotation components of end-to-base matrices
    T_end2base_samples = []  # The translation components of end-to-base matrices
    R_target2cam_samples = []  # The rotation components of target-to-camera matrices
    T_target2cam_samples = []  # The translation components of target-to-camera matrices
    sample_number = 0  # The number of samples already recorded

    rospy.sleep(1)

    while not rospy.is_shutdown():
        try:
            if not end_pose:
                rospy.logwarn("Waiting for JAKA data...")
                time.sleep(1)
                continue

            if not target_pose:
                rospy.logwarn("Waiting for ArUco or AprilTag data...")
                time.sleep(1)
                continue

            print("Record: r, Calculate: c, Save: s, Quit: q")
            command = str(input())

            if command == 'r':
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

            elif command == 'c':
                if sample_number < 3:
                    rospy.logwarn("No enough samples!")

                else:
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

            elif command == 's':
                if not cam2base:
                    rospy.logwarn("No result to save!")

                else:
                    with open('../yaml/camera_to_base_matrix.yaml', 'w', encoding='utf-8') as f:
                        yaml.dump(data=cam2base.tolist(), stream=f)
                    print("Successfully save the matrix")
                    break

            elif command == 'q':
                break

            else:
                rospy.logwarn("Invalid option!")

            time.sleep(2)

        except rospy.ROSInterruptException:
            rospy.logwarn("No available data!")
            continue
