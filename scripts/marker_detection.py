#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import time
import yaml
from tf2_msgs.msg import TFMessage
from sklearn.metrics import mean_squared_error


def callback(data):
    global estimatedPose
    estimatedPose = data.transforms[0].transform


if __name__ == '__main__':
    rospy.init_node("marker_detection", anonymous=True)
    rospy.Subscriber('/tf2', TFMessage, callback, queue_size=10)
    estimatedPose = None
    poses = np.random.random((20, 7))
    cnt = 0

    with open('../yaml/ground_truth_no_occlusion.yaml', 'r') as f:
        gt = yaml.load(f.read(), Loader=yaml.FullLoader)
        gt = np.array(gt)

    while not rospy.is_shutdown():
        if estimatedPose is None:
            time.sleep(1)
            continue
        poses[cnt, 0] = estimatedPose.translation.x
        poses[cnt, 1] = estimatedPose.translation.y
        poses[cnt, 2] = estimatedPose.translation.z
        poses[cnt, 3] = estimatedPose.rotation.w
        poses[cnt, 4] = estimatedPose.rotation.x
        poses[cnt, 5] = estimatedPose.rotation.y
        poses[cnt, 6] = estimatedPose.rotation.z
        cnt += 1
        if cnt == 20:
            pose = np.mean(poses, axis=0)

            # with open('../yaml/ground_truth_no_occlusion.yaml', 'w', encoding='utf-8') as f:
            #     yaml.dump(data=pose.tolist(), stream=f)

            translation_error = np.sqrt(mean_squared_error(gt[0: 3], pose[0: 3]))
            gt_norm = np.linalg.norm(gt[3:])
            pose_norm = np.linalg.norm(pose[3:])
            rotation_error = 1.0 - abs(np.dot(gt[3:] / gt_norm, pose[3:] / pose_norm))
            print(translation_error)
            print(rotation_error)

            break
        time.sleep(0.1)
