import csv

import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt


class Pose:
    def __init__(self, position, quaternion):
        self.position = position
        self.quaternion = quaternion  # qw, qx, qy, qz

    def x(self):
        return self.position[0]

    def y(self):
        return self.position[1]

    def z(self):
        return self.position[2]

    def qw(self):
        return self.quaternion[0]

    def qx(self):
        return self.quaternion[1]

    def qy(self):
        return self.quaternion[2]

    def qz(self):
        return self.quaternion[3]


class Frame:
    def __init__(self, id, ground_truth_pose, GPS_pose, estimated_pose):
        self.id = id
        self.gt_pose = ground_truth_pose
        self.gps_pose = GPS_pose
        self.est_pose = estimated_pose


def euler2Quaternion(yaw, pitch, roll):
    # Convert Euler angles to a rotation matrix
    r = Rotation.from_euler('zyx', [yaw, pitch, roll], degrees=False)
    # Convert the rotation matrix to a quaternion
    quaternion = r.as_quat()
    return quaternion


def quaternion2Euler(quaternion):  # qx, qy, qz, qw
    # Create a Rotation object from the input quaternion
    r = Rotation.from_quat(quaternion)
    # Convert the rotation to Euler angles (Yaw, Pitch, Roll) in radians
    euler_angles = r.as_euler('zyx', degrees=False)
    return [euler_angles[0], euler_angles[1], euler_angles[2]]


def getGTArray(frames):
    gt_poses = []
    for frame in frames:
        ypr = quaternion2Euler(frame.gt_pose.quaternion)
        new_row = [frame.id, frame.gt_pose.x(), frame.gt_pose.y(), frame.gt_pose.z(), ypr[0], ypr[1], ypr[2]]
        gt_poses.append(new_row)
    gt_array = np.array(gt_poses, dtype=float)
    gt_ori_array = gt_array[:, 4:6]
    gt_ori_array = gt_ori_array / np.pi * 180
    # gt_ori_array[gt_ori_array < 0] += 360
    gt_array[:, 4:6] = gt_ori_array
    return gt_array


def getGPSArray(frames):
    gps_poses = []
    for frame in frames:
        ypr = quaternion2Euler(frame.gps_pose.quaternion)
        new_row = [frame.id, frame.gps_pose.x(), frame.gps_pose.y(), frame.gps_pose.z(), ypr[0], ypr[1], ypr[2]]
        gps_poses.append(new_row)
    gps_array = np.array(gps_poses, dtype=float)
    gps_ori_array = gps_array[:, 4:6]
    gps_ori_array = gps_ori_array / np.pi * 180
    # gps_ori_array[gps_ori_array < 0] += 360
    gps_array[:, 4:6] = gps_ori_array
    return gps_array


def getEstArray(frames):
    est_poses = []
    for frame in frames:
        ypr = quaternion2Euler(frame.est_pose.quaternion)
        new_row = [frame.id, frame.est_pose.x(), frame.est_pose.y(), frame.est_pose.z(), ypr[0], ypr[1], ypr[2]]
        est_poses.append(new_row)
    est_array = np.array(est_poses, dtype=float)
    est_ori_array = est_array[:, 4:6]
    est_ori_array = est_ori_array / np.pi * 180
    # est_ori_array[est_ori_array < 0] += 360
    est_array[:, 4:6] = est_ori_array
    return est_array


def calAPE(frame_id, pose1, pose2):
    pos1 = np.array(pose1.position, dtype=float)
    qua1 = np.array(pose1.quaternion, dtype=float)
    pos2 = np.array(pose2.position, dtype=float)
    qua2 = np.array(pose2.quaternion, dtype=float)
    # print(qua1)
    # print(qua2)

    position_error = pos2 - pos1
    ape_pos = np.linalg.norm(position_error)
    # print(position_error)
    # print(ape_pos)

    rot1 = Rotation.from_quat(qua1)
    rot2 = Rotation.from_quat(qua2)
    rotation_error = rot1.inv() * rot2
    quaternion_error = rotation_error.as_quat()
    quaternion_error /= np.linalg.norm(quaternion_error)
    orientation_error = quaternion2Euler(quaternion_error)
    orientation_error = np.array(orientation_error, dtype=float) * 180 / np.pi
    ape_ori = 2.0 * np.arccos(quaternion_error[3])
    if ape_ori > np.pi:
        ape_ori = 2.0 * np.pi - ape_ori
    else:
        ape_ori = ape_ori
    ape_ori = ape_ori * 180 / np.pi
    # print(quaternion_error)
    # print(ape_ori)

    return [frame_id, position_error[0], position_error[1], position_error[2], ape_pos, orientation_error[0],
            orientation_error[1], orientation_error[2], ape_ori]


def calGPSErrorArray(frames):
    GPS_errors = []
    for frame in frames:
        GPS_errors.append(calAPE(frame.id, frame.gt_pose, frame.gps_pose))
    GPS_errors_array = np.array(GPS_errors, dtype=float)
    return GPS_errors_array


def calEstErrorArray(frames):
    EST_errors = []
    for frame in frames:
        EST_errors.append(calAPE(frame.id, frame.gt_pose, frame.est_pose))
    EST_errors_array = np.array(EST_errors, dtype=float)
    return EST_errors_array


def plotFrames(frames):  # Plot poses
    # Figure setings
    label_string = ['Ground Truth', 'GPS', 'Optimised']
    colors = [(0 / 255.0, 0 / 255.0, 0 / 255.0), (157 / 255.0, 195 / 255.0, 230 / 255.0),
              (255 / 255.0, 147 / 255.0, 147 / 255.0)]
    line_styles = ['--', 'solid', 'solid']
    line_width = [1.0, 1.0, 1.0]

    fig, axes = plt.subplots(3, 2, sharex='col', figsize=(10, 4), dpi=144)

    # Obtain the data
    gt_array = getGTArray(frames)
    gps_array = getGPSArray(frames)
    est_array = getEstArray(frames)

    # Plot the data
    for y in range(2):
        for x in range(3):
            col_index = y * 3 + x + 1
            axes[x, y].plot(gt_array[:, 0], gt_array[:, col_index], color=colors[0], linewidth=1.0,
                            linestyle=line_styles[0], label=label_string[0])
            axes[x, y].plot(gps_array[:, 0], gps_array[:, col_index], color=colors[1], linewidth=1.0,
                            linestyle=line_styles[1], label=label_string[1])
            axes[x, y].plot(est_array[:, 0], est_array[:, col_index], color=colors[2], linewidth=1.0,
                            linestyle=line_styles[2], label=label_string[2])

    axes[0][0].set_ylabel('x (m)')
    axes[1][0].set_ylabel('y (m)')
    axes[2][0].set_ylabel('z (m)')
    axes[2][0].set_xlabel('image id')
    axes[2][0].set_xlim(0, 73)
    axes[0][1].set_ylabel('yaw (degree)')
    axes[1][1].set_ylabel('pitch (degree)')
    axes[2][1].set_ylabel('roll (degree)')
    axes[2][1].set_xlabel('image id')

    axes[0][0].legend(loc='upper right', fontsize='10')

    plt.tight_layout()
    plt.show()


def plotAPEs(frames):  # Box
    # Figure setings
    label_string = ['GPS', 'Optimised']
    colors = [(157 / 255.0, 195 / 255.0, 230 / 255.0),
              (255 / 255.0, 147 / 255.0, 147 / 255.0)]
    line_styles = ['solid', 'solid']
    line_width = [1.0, 1.0]

    fig, axes = plt.subplots(4, 2, sharex='col', figsize=(10, 6), dpi=144)

    # Obtain the data
    GPS_errors_array = calGPSErrorArray(frames)
    Est_errors_array = calEstErrorArray(frames)

    # Plot the data
    for y in range(2):
        for x in range(4):
            col_index = y * 4 + x + 1
            axes[x, y].plot(GPS_errors_array[:, 0], GPS_errors_array[:, col_index], color=colors[0], linewidth=1.0,
                            linestyle=line_styles[0], label=label_string[0])
            axes[x, y].plot(Est_errors_array[:, 0], Est_errors_array[:, col_index], color=colors[1], linewidth=1.0,
                            linestyle=line_styles[1], label=label_string[1])

    axes[0][0].set_ylabel('x (m)')
    axes[1][0].set_ylabel('y (m)')
    axes[2][0].set_ylabel('z (m)')
    axes[3][0].set_ylabel('APE (m)')
    axes[3][0].set_xlabel('image id')
    axes[3][0].set_xlim(0, 73)
    axes[0][1].set_ylabel('yaw (degree)')
    axes[1][1].set_ylabel('pitch (degree)')
    axes[2][1].set_ylabel('roll (degree)')
    axes[3][1].set_ylabel('AOE (degree)')
    axes[3][1].set_xlabel('image id')
    axes[3][1].set_xlim(0, 73)

    axes[0][0].legend(loc='upper right', fontsize='10')

    plt.tight_layout()
    plt.show()


def plotStatistics(frames):  # Statistic results
    # Figure setings
    labels = ['GPS', 'Optimised']
    colors = [(157 / 255.0, 195 / 255.0, 230 / 255.0),
              (255 / 255.0, 147 / 255.0, 147 / 255.0)]

    # Prepare the data
    GPS_error_array = calGPSErrorArray(frames)
    EST_error_array = calEstErrorArray(frames)

    APE_data = np.hstack(
        (np.array(GPS_error_array[:, 4]).reshape(-1, 1), np.array(EST_error_array[:, 4]).reshape(-1, 1)))
    AOE_data = np.hstack(
        (np.array(GPS_error_array[:, 8]).reshape(-1, 1), np.array(EST_error_array[:, 8]).reshape(-1, 1)))

    # Plot
    # fig, axes = plt.subplots(1, 2, figsize=(12, 6), dpi=144)
    fig = plt.figure(figsize=(10, 3), dpi=144)
    ax1 = fig.add_subplot(121)
    position = (0.8, 1.2)
    bplot = ax1.boxplot(APE_data, patch_artist=True, positions=position, labels=labels, widths=0.15, showfliers=True)
    for patch, color in zip(bplot['boxes'], colors):
        patch.set_facecolor(color)
    ax1.legend(bplot['boxes'], labels, loc='upper right')
    ax1.set_ylabel('APE (m)')

    ax2 = fig.add_subplot(122)
    position = (0.8, 1.2)
    bplot = ax2.boxplot(AOE_data, patch_artist=True, labels=labels, widths=0.15, showfliers=True)
    for patch, color in zip(bplot['boxes'], colors):
        patch.set_facecolor(color)
    ax2.set_ylabel('AOE (degree)')

    plt.tight_layout()
    plt.show()


file_path = '/data/CapturedData/Simulation/GirderBridge/girder_bridge_pier/final_images_info.txt'

frames = []

with open(file_path, newline='') as csvfile:
    csv_reader = csv.DictReader(csvfile)
    for row in csv_reader:
        gps_pose = Pose([row["gps_x"], row["gps_y"], row["gps_z"]],
                        euler2Quaternion(row["gps_yaw"], row["gps_pitch"], row["gps_roll"]))
        # print(gps_pose.quaternion)
        est_pose = Pose([row["est_x"], row["est_y"], row["est_z"]],
                        [row["est_qx"], row["est_qy"], row["est_qz"], row["est_qw"]])
        # print(est_pose.quaternion)
        gt_pose = Pose([row["gt_x"], row["gt_y"], row["gt_z"]],
                       [row["gt_qx"], row["gt_qy"], row["gt_qz"], row["gt_qw"]])
        # print(gt_pose.quaternion)
        frame = Frame(row["#ID"], gt_pose, gps_pose, est_pose)
        frames.append(frame)

# Plot poses
plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.size'] = 12


plotFrames(frames)
plotAPEs(frames)
plotStatistics(frames)
