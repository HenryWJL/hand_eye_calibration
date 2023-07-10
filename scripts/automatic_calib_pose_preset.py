#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This is used for setting the fixed poses used in automatic
hand-eye calibration.
"""

import rospy
import numpy as np
import yaml
import jkrc

if __name__ == '__main__':
    rospy.init_node('automatic_calib_pose_preset', anonymous=True)
    robot_ip = rospy.get_param("/robot_ip", default="192.168.200.100")
    # Creating an array storing the 25 joint poses
    joint_pose = np.array([25, 6])
    # Counter
    pose_number = 0
    # Starting the robot
    robot = jkrc.RC(robot_ip)
    robot.login()
    robot.power_on()
    robot.enable_robot()

    rospy.sleep(1)

    while not rospy.is_shutdown():
        try:
            print("Record: r")
            command = str(input())
            if command == 'r':
                ret = robot.get_joint_position()
                if ret[0] == 0:
                    pose = np.array(ret[1])
                    joint_pose[pose_number, :] = pose
                    pose_number += 1
                    print(f"{pose_number} poses have been recorded")

                else:
                    print("Failed to acquire the pose data!")

            else:
                rospy.logwarn("Invalid option!")

            if pose_number == 25:
                with open('../yaml/pose_for_automatic_calib.yaml', 'w', encoding='utf-8') as f:
                    yaml.dump(data=joint_pose.tolist(), stream=f)
                robot.logout()
                break

        except rospy.ROSInterruptException:
            rospy.logwarn("No object pose data available!")
