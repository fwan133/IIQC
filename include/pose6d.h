#ifndef DATA_QUALITY_CHECK_POSE6D_H
#define DATA_QUALITY_CHECK_POSE6D_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

class Pose6d {
public:
    Pose6d() {};

    Pose6d(double x, double y, double z, double qx, double qy, double qz, double qw) {
        position = Eigen::Vector3d(x, y, z);
        orientation = Eigen::Quaterniond(qw, qx, qy, qz);
    }

    Pose6d(Eigen::Vector3d Position, Eigen::Quaterniond Orientation) {
        position = Position;
        orientation = Orientation;
    };

    Pose6d(Eigen::Isometry3d T){
        position = T.translation();
        orientation = Eigen::Quaterniond(T.linear());
    }

    Pose6d(const geometry_msgs::Pose &pose) {
        position = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
        orientation = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                         pose.orientation.z);
    };

    Pose6d(const nav_msgs::Odometry &odometry) {
        position = Eigen::Vector3d(odometry.pose.pose.position.x, odometry.pose.pose.position.y,
                                   odometry.pose.pose.position.z);
        orientation = Eigen::Quaterniond(odometry.pose.pose.orientation.w, odometry.pose.pose.orientation.x,
                                         odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z);
    };

    void setPosition(double x, double y, double z) {
        position = Eigen::Vector3d(x, y, z);
    }

    void setOrientation(double qw, double qx, double qy, double qz) {
        orientation = Eigen::Quaterniond(qw, qx, qy, qz);
    }

    const Eigen::Isometry3d cvt2TransformMatrix() const {
        Eigen::Isometry3d isometry3d = Eigen::Isometry3d::Identity();
        isometry3d.prerotate(orientation);
        isometry3d.pretranslate(position);
        return isometry3d;
    }

    const Eigen::Matrix3d getRotationMatrix() const {
        return orientation.normalized().toRotationMatrix();
    }

    ~Pose6d() {};


public:
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

#endif