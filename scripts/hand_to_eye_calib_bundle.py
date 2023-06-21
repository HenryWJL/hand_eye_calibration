import numpy as np
import cv2

camera_number = 4
X = 5
Y = 7
length = 0.04
separation = 0.01
aruco_dictionary = cv2.aruco.DICT_4X4_100
dictionary = cv2.aruco.Dictionary_get(aruco_dictionary)
board = cv2.aruco.GridBoard_create(X, Y, length, separation, dictionary)
width = int(X * (length + separation) * 10000)
height = int(Y * (length + separation) * 10000)
board_img = cv2.aruco_GridBoard.draw(board, (width, height), 1)
cv2.imwrite('../Img/board.jpg', board_img)
# cv2.imshow('board', board_img)
# capturer = cv2.VideoCapture(camera_number)
# while True:
#     ret, frame = capturer.read()
#     if ret is False:
#         break
#
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
#
# capturer.release()
# cv2.destroyAllWindows()
