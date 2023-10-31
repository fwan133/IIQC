import ifcopenshell
import ifcopenshell.geom
from numpy.typing import ArrayLike as ndArray
import numpy as np
import open3d as o3d


class BridgeComponent:
    def __init__(self):
        self.global_id = ''
        self.semantic_label = ''
        self.sequence_number = ''
        self.scalar = 0
        self.R = 128
        self.G = 128
        self.B = 128
        self.points_data = np.empty((0, 3))

    def print(self):
        print(
            f"Global_ID: {self.global_id}, Scalar: {self.scalar}, Semantic_Label: {self.semantic_label}, Sequence_Number: {self.sequence_number}, R: {self.R}, G: {self.G}, B: {self.B}")


class Bridge:
    def __init__(self):
        self.bridge_components = []

    def addComponent(self, bridge_component):
        self.bridge_components.append(bridge_component)

    def saveComponent(self, file_path):
        with open(file_path + '/BridgeInfo.txt', 'w') as file:
            for bridge_component in self.bridge_components:
                row = f"Global_ID: {bridge_component.global_id}, Scalar: {bridge_component.scalar}, Semantic_Label: {bridge_component.semantic_label}, Sequence_Number: {bridge_component.sequence_number}, R: {bridge_component.R}, G: {bridge_component.G}, B: {bridge_component.B}"
                file.write(f"{row}\n")
            print('Save Components Successfully!')

    def savePointCloud(self, folder_path):
        point_cloud = o3d.geometry.PointCloud()

        point_clout_tem = np.empty((0, 4), dtype=float)

        size = 0
        for bridge_component in self.bridge_components:
            size = size + bridge_component.points_data.shape[0]

        # Open the .ply file for writing
        with open((folder_path + '/PointCloud.ply'), "w") as ply_file:
            # Write the header
            ply_file.write("ply\n")
            ply_file.write("format ascii 1.0\n")
            ply_file.write(f"element vertex {size}\n")
            ply_file.write("property float x\n")
            ply_file.write("property float y\n")
            ply_file.write("property float z\n")
            ply_file.write("property float scalar\n")
            ply_file.write("end_header\n")

            for bridge_component in self.bridge_components:
                for point in bridge_component.points_data:
                    ply_file.write(f"{point[0]} {point[1]} {point[2]} {bridge_component.scalar}\n")

        print('Save Point Cloud Successfully!')


def local_to_world(origin, transform, verts):
    return origin + np.matmul(transform.T, verts.T).T


def barycentric(N):
    """Generate N random barycentric coordinates.
    Returns three numpy arrays of shape (N, 1).
    """
    u = np.random.rand(N, 1)
    v = np.random.rand(N, 1)

    lp = u + v > 1
    u[lp] = 1 - u[lp]
    v[lp] = 1 - v[lp]
    w = 1 - (u + v)
    return u, v, w


def triangle_areas(v1: ndArray, v2: ndArray, v3: ndArray):
    """Calculate the area of multiple triangle given its vertices.
    Parameters are numpy arrays of shape (n, 3) where n is the number of triangles.
    """
    return 0.5 * np.linalg.norm(np.cross(v2 - v1, v3 - v1), axis=1)


def gen_pointcloud(vertices: ndArray, faces: ndArray, resolution=0.05):
    """Generate a point cloud from a mesh.
    Parameters are vertices and faces of a mesh as numpy array.
    Returns a numpy array of shape (num_points, 3) with dtype=float32.
    """
    v1 = vertices[faces[:, 0]]
    v2 = vertices[faces[:, 1]]
    v3 = vertices[faces[:, 2]]

    areas = triangle_areas(v1, v2, v3)
    totalArea = np.sum(areas)
    probs = areas / totalArea
    num_points_per_m2 = int(1.0 / resolution + 1) ** 2
    num_points = totalArea.astype(int) * num_points_per_m2
    random_indices = np.random.choice(range(len(areas)), num_points, p=probs)
    v1 = v1[random_indices]
    v2 = v2[random_indices]
    v3 = v3[random_indices]
    u, v, w = barycentric(num_points)
    points = u * v1 + v * v2 + w * v3
    return points.astype(np.float32)


def count_elements_in_list(semantic_label, bridge_components_list):
    number = 0
    for bridge_component in bridge_components_list:
        if bridge_component.semantic_label == semantic_label:
            number = number + 1

    return number


def generate_pointclouds_from_ifc_element(ifc_element, settings, resolution):
    shape = ifcopenshell.geom.create_shape(settings, ifc_element)
    matrix = shape.transformation.matrix.data
    matrix = np.array(matrix).reshape(4, 3)
    origin = matrix[-1]
    transform = matrix[:-1]
    verts = shape.geometry.verts
    faces = shape.geometry.faces
    materials = shape.geometry.materials
    rgb = [int(255 * x) for x in materials[0].diffuse]

    verts = np.array([[verts[i], verts[i + 1], verts[i + 2]] for i in range(0, len(verts), 3)])
    faces = np.array([[faces[i], faces[i + 1], faces[i + 2]] for i in range(0, len(faces), 3)])
    verts = local_to_world(origin, transform, verts)
    points = gen_pointcloud(verts, faces, resolution=resolution)
    return points


if __name__ == '__main__':
    # Part 1: Read the ifc file and obtain the semantic list && RGB
    folder_path = '/home/feng/Code/catkin_ros/src/IIQC/data/prior_model/ArchBridgeSim'
    ifc_file_path = folder_path + '/arch_bridge_sim.ifc'
    model = ifcopenshell.open(ifc_file_path)
    # Check the types
    types = set(i.is_a() for i in model)
    print(types)
    bridge_entity = model.by_type('IfcBuildingElementProxy')
    print('A total of %d components are found in the IFC file. They are:' % bridge_entity.__len__())

    settings = ifcopenshell.geom.settings()
    settings.set(settings.APPLY_DEFAULT_MATERIALS, True)

    # Part 2: Create the component list and point clouds
    bridge = Bridge()
    index = 0
    resolution = 0.02
    for bridgeComponent in bridge_entity:
        component = BridgeComponent()
        component.global_id = bridgeComponent.Tag
        component.semantic_label = bridgeComponent.ObjectType.split(":")[0]
        component.sequence_number = count_elements_in_list(component.semantic_label, bridge.bridge_components) + 1
        component.scalar = index
        points = generate_pointclouds_from_ifc_element(bridgeComponent, settings, resolution=resolution)
        component.points_data = points
        component.print()
        bridge.addComponent(component)
        index += 1

    # Part 3: Save the bridge components and point clouds
    bridge.saveComponent(folder_path)
    bridge.savePointCloud(folder_path)
