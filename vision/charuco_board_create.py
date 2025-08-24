import cv2
import cv2.aruco as aruco
import numpy as np

# Parameters for ChAruCo board
squares_x = 5
squares_y = 7
square_length = 0.04  # meters
marker_length = 0.02  # meters

# For OpenCV >= 4.7.0, use cv2.aruco.CharucoBoard instead of CharucoBoard_create
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
board = aruco.CharucoBoard((squares_x, squares_y), square_length, marker_length, aruco_dict)

# Save ChAruCo board image

# For OpenCV >= 4.7.0, use the draw method from the CharucoBoard class
img = np.zeros((900, 600), dtype=np.uint8)
img = board.generateImage((600, 900))
cv2.imwrite('charuco_board.png', img)
print('ChAruCo board image saved as charuco_board.png')
