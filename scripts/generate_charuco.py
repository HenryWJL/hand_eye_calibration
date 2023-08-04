#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This is used for generating self-customized ChArUco boards.
"""

import cv2
from aruco_utils import ARUCO_DICTIONARY


def generate_charuco(squares_x, squares_y, square_length, marker_length, aruco_dictionary):
    arucoDictionary = cv2.aruco.Dictionary_get(ARUCO_DICTIONARY[aruco_dictionary])
    charucoBoard = cv2.aruco.CharucoBoard_create(squares_x, squares_y, square_length, marker_length, arucoDictionary)
    width = int(squares_x * square_length * 10000)  # The width of the image (in pixels)
    height = int(squares_y * square_length * 10000)  # The height of the image (in pixels)
    charucoBoardImg = cv2.aruco_CharucoBoard.draw(charucoBoard, (width, height), 1)
    cv2.imwrite('../Img/charuco/{}X{}_{}.jpg'.format(squares_x, squares_y, aruco_dictionary), charucoBoardImg)


if __name__ == '__main__':
    # Parameters
    squaresX = 4  # The number of squares in each row
    squaresY = 4  # The number of squares in each column
    squareLength = 0.04  # The length of each square
    markerLength = 0.03  # The length of each marker
    dictionary = "DICT_6X6_250"  # The marker dictionary
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
    generate_charuco(squaresX, squaresY, squareLength, markerLength, dictionary)
