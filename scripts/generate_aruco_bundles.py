#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This is used for generating self-customized ArUco bundles.
"""

import cv2
from utils import ARUCO_DICTIONARY

markersX = 5  # The number of markers in each row
markersY = 7  # The number of markers in each column
markerLength = 0.04  # The length of each marker (in meters)
markerSeparation = 0.01  # The separation between any two markers (in meters)
aruco_dictionary = "DICT_4X4_100"  # The dictionary that the markers belong to


def generate_bundles():
    dictionary = cv2.aruco.Dictionary_get(ARUCO_DICTIONARY[aruco_dictionary])
    board = cv2.aruco.GridBoard_create(markersX, markersY, markerLength, markerSeparation, dictionary)
    width = int(markersX * (markerLength + markerSeparation) * 10000)  # The width of the image (in pixels)
    height = int(markersY * (markerLength + markerSeparation) * 10000)  # The height of the image (in pixels)
    board_img = cv2.aruco_GridBoard.draw(board, (width, height), 1)
    cv2.imwrite('../Img/{}X{}_{}_board.jpg'.format(markersX, markersY, aruco_dictionary), board_img)


if __name__ == '__main__':
    generate_bundles()
