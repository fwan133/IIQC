import numpy as np
import cv2

# Example intrinsic matrix (3x3)
intrinsic_matrix = np.array([[1000.0, 0.0, 500.0],
                             [0.0, 1000.0, 300.0],
                             [0.0, 0.0, 1.0]])

# Example extrinsic matrix (3x4)
extrinsic_matrix = np.array([[1.0, 0.0, 0.0, 0.0],
                             [0.0, 0.0, -1.0, -2.0],
                             [0.0, 1.0, 0.0, 0.0]])

# Example 3D world point (x, y, z)
world_point = np.array([0.0, 0.0, -0.0])

# Project the 3D point to the image plane using OpenCV
image_point = cv2.projectPoints(world_point, extrinsic_matrix[:, :3], extrinsic_matrix[:, 3], intrinsic_matrix, None)

# Extract the 2D image coordinates
image_point = image_point[0]

print("Projected image point:", image_point)
