#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

typedef pcl::PointXYZ PointType;

int main() {




    /*
    // Create two point clouds (replace with your data)
    pcl::PointCloud<PointType>::Ptr cloud_source(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_target(new pcl::PointCloud<PointType>);

    // Fill in your point clouds with data
    cloud_source

    // Create ICP object
    pcl::IterativeClosestPoint<PointType, PointType> icp;

    // Set the maximum number of iterations and the transformation epsilon (convergence criteria)
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-4);

    // Enable scaling
    icp.setRANSACOutlierRejectionThreshold(0.1); // Adjust this threshold as needed

    // Set the source and target point clouds
    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);

    // Align the source to the target
    pcl::PointCloud<PointType> aligned_cloud;
    icp.align(aligned_cloud);

    // Get the estimated transformation matrix (rotation, translation, and scale)
    Eigen::Matrix4f transformation = icp.getFinalTransformation();

    // Extract the rotation, translation, and scale from the transformation matrix
    Eigen::Matrix3f rotation = transformation.block<3, 3>(0, 0);
    Eigen::Vector3f translation = transformation.block<3, 1>(0, 3);
    float scale = rotation.determinant();

    // Output the results
    std::cout << "Estimated Rotation Matrix:\n" << rotation << std::endl;
    std::cout << "Estimated Translation Vector:\n" << translation << std::endl;
    std::cout << "Estimated Scale Factor:\n" << scale << std::endl;
    */

    return 0;
}
