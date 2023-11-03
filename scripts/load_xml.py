import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import open3d as o3d


def avgPositionFromFrames(frames):
    num_frames = len(frames)
    total_x = sum(frame.est_x for frame in frames)
    total_y = sum(frame.est_y for frame in frames)
    total_z = sum(frame.est_z for frame in frames)

    central_x = total_x / num_frames
    central_y = total_y / num_frames
    central_z = total_z / num_frames

    # print(f"Central Point (x, y, z): {central_x}, {central_y}, {central_z}")
    return [central_x, central_y, central_z]


class Point:
    def __init__(self, x_position, y_position, z_position, red_color, green_color, blue_color):
        self.x = x_position
        self.y = y_position
        self.z = z_position
        self.r = red_color
        self.g = green_color
        self.b = blue_color

    def __str__(self):
        return f"Position: x={self.x}, y={self.y}, z={self.z}, Color: r={self.r}, g={self.g}, b={self.b}"

    def applyTranslation(self, translation_vector):
        self.x = self.x - translation_vector[0]
        self.y = self.y - translation_vector[1]
        self.z = self.z - translation_vector[2]


'''
Read the tie points
'''
# Parse the XML file
tree_landmarks = ET.parse('/home/feng/Code/catkin_ros/src/IIQC/data/real_world/pier/Pier - AT - export - TiePoints.xml')

# Get the root element of the XML document
tie_points = tree_landmarks.getroot()

# Access and manipulate the data in the XML document

cloud = []

for tie_point in tie_points.findall('TiePoint'):
    position = tie_point.find('Position')
    color = tie_point.find('Color')

    x = float(position.find('x').text)
    y = float(position.find('y').text)
    z = float(position.find('z').text)
    r = float(color.find('Red').text)
    g = float(color.find('Green').text)
    b = float(color.find('Blue').text)

    point = Point(x, y, z, r, g, b)
    cloud.append(point)

'''
Read the camera frames
'''


class CameraFrame:
    def __init__(self, id, x, y, z, qx, qy, qz, qw):
        self.id = id
        self.est_x = x
        self.est_y = y
        self.est_z = z
        self.est_qx = qx
        self.est_qy = qy
        self.est_qz = qz
        self.est_qw = qw

    def applyTranslation(self, translation_vector):
        self.est_x = self.est_x - translation_vector[0]
        self.est_y = self.est_y - translation_vector[1]
        self.est_z = self.est_z - translation_vector[2]


# Parse the XML file
tree_frames = ET.parse('/home/feng/Code/catkin_ros/src/IIQC/data/real_world/pier/Pier - AT - export.xml')
root_frames = tree_frames.getroot()

# Get the root element of the XML document
camera_frames = []
for photo in root_frames.findall('.//Photo'):
    image_id = int(photo.find('Id').text) + 1

    # Obtain the position vector
    x = float(photo.find('.//Center/x').text)
    y = float(photo.find('.//Center/y').text)
    z = float(photo.find('.//Center/z').text)

    # Obtain the rotation matrix
    M_00 = float(photo.find('.//Rotation/M_00').text)
    M_01 = float(photo.find('.//Rotation/M_01').text)
    M_02 = float(photo.find('.//Rotation/M_02').text)
    M_10 = float(photo.find('.//Rotation/M_10').text)
    M_11 = float(photo.find('.//Rotation/M_11').text)
    M_12 = float(photo.find('.//Rotation/M_12').text)
    M_20 = float(photo.find('.//Rotation/M_20').text)
    M_21 = float(photo.find('.//Rotation/M_21').text)
    M_22 = float(photo.find('.//Rotation/M_22').text)
    rotation_matrix = np.array([
        [M_00, M_01, M_02],
        [M_10, M_11, M_12],
        [M_20, M_21, M_22]
    ])
    inverse_rotation_matrix = np.linalg.inv(rotation_matrix)
    rotation = Rotation.from_matrix(inverse_rotation_matrix)
    quaternion = rotation.as_quat()

    camera_frame = CameraFrame(image_id, x, y, z, quaternion[0], quaternion[1], quaternion[2], quaternion[3])
    camera_frames.append(camera_frame)

'''
Applied the translation and save the data
'''
translation_vector = avgPositionFromFrames(camera_frames)

for point in cloud:
    point.applyTranslation(translation_vector)

# Create an empty Open3D point cloud
point_cloud = o3d.geometry.PointCloud()

# Convert the list of Point objects to a numpy array
points = []
colors = []
for point in cloud:
    points.append([point.x, point.y, point.z])
    colors.append([point.r, point.g, point.b])

point_cloud.points = o3d.utility.Vector3dVector(points)
point_cloud.colors = o3d.utility.Vector3dVector(colors)

# Save the point cloud to a .ply file
o3d.io.write_point_cloud('../data/real_world/pier/keypoint_cloud.ply', point_cloud)

# Save the camera frames
for frame in camera_frames:
    frame.applyTranslation(translation_vector)

# Specify the path to the output .txt file
output_file_path = "../data/real_world/pier/camera_frames.txt"

# Open the file for writing and save the data
with open(output_file_path, "w") as file:
    file.write(f"#ID,est_x,est_y,est_z,est_qx,est_qy,est_qz,est_qw\n")
    for frame in camera_frames:
        # Write the data for each CameraFrame object
        file.write(f"{frame.id},{frame.est_x},{frame.est_y},{frame.est_z},{frame.est_qx},{frame.est_qy},{frame.est_qz},{frame.est_qw}\n")

print(f"Data saved to {output_file_path}")
