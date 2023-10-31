import numpy as np
import random
import ifc_to_semantic_pc


class SemanticPoint:
    def __init__(self, x, y, z, r, g, b, semantic_label):
        self.x = x
        self.y = y
        self.z = z
        self.r = r
        self.g = g
        self.b = b
        self.semantic_label = semantic_label


def read_file(filename):
    data = []
    with open(filename, 'r') as file:
        for line in file:
            # Initialize an empty dictionary for this line's data
            entry = {}

            # Split the line into its attributes
            attributes = line.strip().split(', ')

            for attribute in attributes:
                key, value = attribute.split(': ')
                entry[key.strip()] = value.strip()

            data.append(entry)

    return data


def read_xyz_file(filename):
    point_cloud = []

    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()  # Remove leading and trailing whitespace

            # Skip comments or header lines
            if line.startswith("#"):
                continue

            # Convert the line to a list of numbers (floats)
            point = list(map(float, line.split()))

            # Append the point to the point cloud
            point_cloud.append(point)

    return point_cloud


def matchSemanticFromData(data, scalar):
    for entry in data:
        index = float(entry.get('Scalar'))
        if index == scalar:
            return entry.get('Semantic_Label')


def random_rgb():
    return (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))


def save_points_to_xyz(points, filename):
    with open(filename, 'w') as file:
        # Optional: Add a header to the file
        file.write("# x y z r g b semantic_label\n")

        for point in points:
            line = f"{point.x} {point.y} {point.z} {point.r} {point.g} {point.b} {point.semantic_label}\n"
            file.write(line)


if __name__ == '__main__':
    # Read the defined semantic label
    filename = "/data/prior_model/HastingsBridge/Bridge_semantic_label.txt"
    data = read_file(filename)

    # Read the point cloud:
    filename = "/data/prior_model/HastingsBridge/BridgePointCloud.xyz"
    point_cloud_data = read_xyz_file(filename)

    # To display the contents:
    semantic_list = {
        "Default": (0, 0, 0),
    }

    semantic_pc = []
    for point in point_cloud_data:
        scalar = point[3]
        semantic_label = matchSemanticFromData(data, scalar)
        color = (0, 0, 0)
        if semantic_label in semantic_list:
            color = semantic_list.get(semantic_label)
        else:
            color = random_rgb()
            semantic_list[semantic_label] = color
        semantic_point = SemanticPoint(point[0], point[1], point[2], color[0], color[1], color[2], semantic_label)
        semantic_pc.append(semantic_point)

    save_points_to_xyz(semantic_pc, "/data/prior_model/HastingsBridge/SemanticPC.xyz")
