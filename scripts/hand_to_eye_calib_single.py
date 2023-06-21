#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This is used for hand-eye calibration. Either ArUco markers or
AprilTag markers are feasible for pose estimation.
"""

import rospy
import numpy as np
import cv2
import transforms3d as tfs
from geometry_msgs.msg import TwistStamped
from tf2_msgs.msg import TFMessage
import time
import yaml

end_pose = None  # The pose of robot's end
target_pose = None  # The pose of ArUco target


def get_end2base_mat(pose):
    # Calculate the gripper-to-base transformation matrix
    tran = np.array((pose.twist.linear.x, pose.twist.linear.y, pose.twist.linear.z))
    rot = tfs.euler.euler2mat(pose.twist.angular.x, pose.twist.angular.y, pose.twist.angular.z)
    return rot, tran


def get_target2cam_mat(pose):
    # Calculate the ArUco-target-to-camera transformation matrix
    pose = pose.transforms[0].transform
    tran = np.array(([pose.translation.x], [pose.translation.y], [pose.translation.z]))
    rot = tfs.quaternions.quat2mat((pose.rotation.w, pose.rotation.x,
                                    pose.rotation.y, pose.rotation.z))
    return rot, tran


def end_pose_callback(pose):
    global end_pose
    if pose is None:
        rospy.logwarn("No robot pose data!")

    else:
        end_pose = pose


def target_pose_callback(pose):
    global target_pose
    if pose is None:
        rospy.logwarn("No ArUco or AprilTag data!")

    else:
        target_pose = pose


if __name__ == '__main__':
    """
    End-pose data and target-pose data are obtained from topics '/robot_driver/tool_point' 
    and '/tf', respectively. 
    """
    rospy.init_node('hand_to_eye_calib', anonymous=True)
    rospy.Subscriber('/robot_driver/tool_point', TwistStamped, end_pose_callback, queue_size=10)
    rospy.Subscriber('/tf', TFMessage, target_pose_callback, queue_size=10)

    R_end2base_samples = []
    T_end2base_samples = []
    R_target2cam_samples = []
    T_target2cam_samples = []
    sample_number = 0
    cam2base = None

    while not rospy.is_shutdown():
        try:
            if end_pose is None:
                rospy.logwarn("Waiting for JAKA data...")
                time.sleep(2)
                continue

            if target_pose is None:
                rospy.logwarn("Waiting for ArUco or AprilTag data...")
                time.sleep(2)
                continue

            print("Record: r, Calculate: c, Save: s, Quit: q")
            command = str(input())

            if command == 'r':
                (R_target2cam, T_target2cam) = get_target2cam_mat(target_pose)
                R_target2cam_samples.append(R_target2cam)
                T_target2cam_samples.append(T_target2cam)

                (R_end2base, T_end2base) = get_end2base_mat(end_pose)
                R_end2base_samples.append(R_end2base)
                T_end2base_samples.append(T_end2base)

                sample_number += 1
                print(f"{sample_number} samples have been recorded")

            elif command == 'c':
                if sample_number < 3:
                    rospy.logwarn("No enough samples!")

                else:
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
                if cam2base is None:
                    rospy.logwarn("No result to save!")

                else:
                    with open('../yaml/camera_to_base_matrix_apriltag.yaml', 'w', encoding='utf-8') as f:
                        yaml.dump(data=cam2base.tolist(), stream=f)
                    break

            elif command == 'q':
                break

            else:
                rospy.logwarn("Invalid option!")

            time.sleep(2)

        except rospy.ROSInterruptException:
            rospy.logwarn("No available data!")
            continue
