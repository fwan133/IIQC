#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>

// Function to estimate the scale and transformation matrix.
bool EstimateScaleAndTransform(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
        float& scale,
        Eigen::Matrix4f& transformation_matrix) {

    // Create an ICP object.
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);

    // Set ICP parameters (optional).
    icp.setMaxCorrespondenceDistance(0.05); // Set a maximum correspondence distance.
    icp.setMaximumIterations(50); // Set the maximum number of iterations.

    // Create a transformation matrix for the initial guess (optional).
    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity(); // Identity matrix for no initial guess.
    icp.align(*source_cloud, initial_guess);

    // Check if the ICP converged and obtain the transformation matrix.
    if (!icp.hasConverged()) {
        std::cerr << "ICP did not converge." << std::endl;
        return false;
    }

    // Calculate the scaling factor using the average distance between matched points.
    float total_scale = 0.0f;
    int num_matches = 0;

    for (size_t i = 0; i < source_cloud->size(); ++i) {
        int corresponding_index = icp.correspondences_[i];
        if (corresponding_index != -1) {
            float source_target_distance = pcl::euclideanDistance(
                    source_cloud->points[i], target_cloud->points[corresponding_index]);
            if (source_target_distance != 0) {
                total_scale += source_target_distance;
                num_matches++;
            }
        }
    }

    if (num_matches == 0) {
        std::cerr << "No valid correspondences found for scaling." << std::endl;
        return false;
    }

    scale = total_scale / num_matches;

    // Extract the transformation matrix.
    transformation_matrix = icp.getFinalTransformation();

    return true;
}

int main() {
    // Load your source and target point clouds (replace with your data).
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill source_cloud and target_cloud with your point data.

    // Estimate the scale and transformation matrix.
    float scale;
    Eigen::Matrix4f transformation_matrix;

    if (EstimateScaleAndTransform(source_cloud, target_cloud, scale, transformation_matrix)) {
        // Output results.
        std::cout << "Estimated Scale: " << scale << std::endl;
        std::cout << "Estimated Transformation Matrix:" << std::endl;
        std::cout << transformation_matrix << std::endl;
    }

    return 0;
}
