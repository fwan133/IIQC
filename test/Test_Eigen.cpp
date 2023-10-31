#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <Eigen/Dense>

int main(int argc, char **argv) {
    // Define transformation parameters
    double x = 1.0;         // Translation along the x-axis
    double y = 2.0;         // Translation along the y-axis
    double z = 3.0;         // Translation along the z-axis
    double yaw = 112.0/180*M_PI ;       // Yaw rotation (around the z-axis)
    double pitch = 0.0;     // Pitch rotation (around the y-axis)
    double roll = 0.0;      // Roll rotation (around the x-axis)

    // Construct the transformation matrix
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

    // Translation part (top-right 3x1 submatrix)
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    // Yaw rotation (z-axis)
    Eigen::Quaterniond q = yawAngle*pitchAngle*rollAngle;
    std::cout << yaw << std::endl;
    // Copy the rotation matrix into the transformation matrix (top-left 3x3 submatrix)
    transformation_matrix.block<3, 3>(0, 0) = q.matrix();

    // Output the transformation matrix
    std::cout << "Transformation Matrix:\n" << transformation_matrix << std::endl;

    return 0;
}
