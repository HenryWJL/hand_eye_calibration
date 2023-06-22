#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This is used for generating self-customized ArUco boards.
"""

import cv2
from aruco_utils import ARUCO_DICTIONARY


def generate_bundles():
    dictionary = cv2.aruco.Dictionary_get(ARUCO_DICTIONARY[arucoDictionary])
    board = cv2.aruco.GridBoard_create(markersX, markersY, markerLength, markerSeparation, dictionary)
    width = int(markersX * (markerLength + markerSeparation) * 10000)  # The width of the image (in pixels)
    height = int(markersY * (markerLength + markerSeparation) * 10000)  # The height of the image (in pixels)
    board_img = cv2.aruco_GridBoard.draw(board, (width, height), 1)
    cv2.imwrite('../Img/{}X{}_{}_board.jpg'.format(markersX, markersY, aruco_dictionary), board_img)


if __name__ == '__main__':
    # Parameters
    markersX = 5  # The number of markers in each row
    markersY = 7  # The number of markers in each column
    markerLength = 0.04  # The length of each marker (in meters)
    markerSeparation = 0.01  # The separation between any two markers (in meters)
    arucoDictionary = "DICT_4X4_100"  # The dictionary that the markers belong to
    """
    Available Dictionaries:
    "DICT_ARUCO_ORIGINAL"
    "DICT_4X4_50"
    "DICT_4X4_100"
    "DICT_4X4_250"
    "DICT_4X4_1000"
    "DICT_5X5_50"
    "DICT_5X5_100"
    "DICT_5X5_250"
    "DICT_5X5_1000"
    "DICT_6X6_50"
    "DICT_6X6_100"
    "DICT_6X6_250"
    "DICT_6X6_1000"
    "DICT_7X7_50"
    "DICT_7X7_100"
    "DICT_7X7_250"
    "DICT_7X7_1000"
    "DICT_APRILTAG_16h5"
    "DICT_APRILTAG_25h9"
    "DICT_APRILTAG_36h10"
    "DICT_APRILTAG_36h11"
    """

    generate_bundles()
