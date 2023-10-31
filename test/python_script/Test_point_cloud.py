import open3d as o3d
import numpy as np

# Sample list of 3D points with x, y, z coordinates
points_list = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]

# Sample list of string field values
string_field_values = ['A', 'B', 'C']

# Convert the list of points to a NumPy array
points_array = np.array(points_list, dtype=np.float32)

# Create an Open3D PointCloud object
point_cloud = o3d.geometry.PointCloud()

# Set the points
point_cloud.points = o3d.utility.Vector3dVector(points_array)

# Create a new attribute for the point cloud
point_cloud.global_id = np.array(string_field_values, dtype=object)

# Save the PointCloud object to a .pcd file
o3d.io.write_point_cloud('point_cloud.pcd', point_cloud)