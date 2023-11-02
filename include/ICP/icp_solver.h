#ifndef IIQC_ICP_SOLVER_H
#define IIQC_ICP_SOLVER_H

#include <iostream>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ia_ransac.h>

#include "ICP/ICPScale.h"

class ICPSolver {
public:
    typedef std::shared_ptr<ICPSolver> Ptr;

    ICPSolver() {}

    /// Set ground truth cloud
    void setICPSolver(std::string gt_filename, bool visulised) {
        ground_truth_cloud = loadPLYFile(gt_filename);
        ICPSolver::visualised = visulised;
    }

    /// Rigid Alignment
    void registerWithGICP(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &registered_cloud, Eigen::Isometry3d &T,
                          double &scale_factor) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = filterWithStatistics(src_cloud);

        // Solve
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
        gicp.setInputSource(filtered_cloud);
        gicp.setInputTarget(ground_truth_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        gicp.align(*aligned_cloud);

        if (gicp.hasConverged()) {
//            std::cout << "GICP converged with score: " << gicp.getFitnessScore() << std::endl;
            double scale = gicp.getFinalTransformation().block<3, 1>(0, 0).norm();
//            std::cout << "Estimated scale factor: " << scale << std::endl;
//            std::cout << "Transformation matrix:\n" << gicp.getFinalTransformation() << std::endl;
            registered_cloud = aligned_cloud;
        } else {
            std::cerr << "GICP did not converge." << std::endl;
        }

        if (visualised) {
            /// Visualise the point cloud
            pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
            viewer.setBackgroundColor(0.0, 0.0, 0.0);
            viewer.addPointCloud(registered_cloud, "registered_cloud");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0,
                                                    "registered_cloud");
            viewer.addPointCloud(ground_truth_cloud, "target_cloud");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.9, 0.9,
                                                    "target_cloud");
            viewer.addPointCloud(filtered_cloud, "source_cloud");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0,
                                                    "source_cloud");

            while (!viewer.wasStopped()) {
                viewer.spinOnce();
            }

            viewer.close();
//            std::cout << "[Registration Visualisation]\n";
        }

        T.matrix() = gicp.getFinalTransformation().cast<double>();
        scale_factor = 1;
    }


    /// Rigid Alignment with considering scale
    void registerWithScaleGICP(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr &registered_cloud, Eigen::Isometry3d &T,
                               double &scale_factor) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_initial = filterWithStatistics(src_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = filterWithStatistics(filtered_cloud_initial);

        /// Step 1: Estimate the inital alignment using GICP
        Eigen::Matrix4d T_initial;
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
        gicp.setInputSource(filtered_cloud);
        gicp.setInputTarget(ground_truth_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        gicp.align(*aligned_cloud);

        if (gicp.hasConverged()) {
//            std::cout << "GICP converged with score: " << gicp.getFitnessScore() << std::endl;
            std::cout << "[ICP]: Initial alignment with GICP. The scale factor is 1. Transformation matrix is:\n" << gicp.getFinalTransformation() << std::endl;
            T_initial = gicp.getFinalTransformation().cast<double>();
        } else {
            std::cerr << "GICP did not converge." << std::endl;
            throw std::runtime_error("ICP failed");
        }

        /// Step 2: Estimate the final alignment using Scale-GICP
        Eigen::Matrix4d T_second;
        ScaleEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, double>::Ptr pestiS = std::make_shared<ScaleEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, double>>();
        pcl::registration::TransformationEstimation<pcl::PointXYZ, pcl::PointXYZ, double>::Ptr pS = pestiS;
        IterativeClosestPointScale<pcl::PointXYZ, pcl::PointXYZ, double>::Ptr pScale (new IterativeClosestPointScale<pcl::PointXYZ, pcl::PointXYZ, double>(pS));
        pScale->setInputSource(aligned_cloud);
        pScale->setInputTarget(ground_truth_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr psrc_pointcloud_final(new pcl::PointCloud<pcl::PointXYZ>());
        pScale->align(*psrc_pointcloud_final);

        T_second = pScale->getFinalTransformation();
        double refined_scale_factor = pScale->getFinalTransformation().block<3, 1>(0, 0).norm();
        std::cout << "[ICP]: Refine alignment with Scale-ICP. The scale factor is " << refined_scale_factor << ". The SIM3 transformation matrix is:\n" << pScale->getFinalTransformation() << std::endl;

        registered_cloud = psrc_pointcloud_final;

        /// Convert the Matrix4d to Isodometry
        Eigen::Matrix4d SIM3 = T_initial * T_second;
        Eigen::Matrix3d rotationMatrix = SIM3.topLeftCorner<3, 3>();
        Eigen::Quaterniond rotationQuaternion(rotationMatrix);
        Eigen::Vector3d translationVector = SIM3.topRightCorner<3, 1>();

        Eigen::Isometry3d isometryTransform = Eigen::Isometry3d::Identity();
        isometryTransform.linear() = rotationQuaternion.toRotationMatrix();
        isometryTransform.translation() = translationVector;

        if (visualised) {
            /// Visualise the point cloud
            pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
            viewer.setBackgroundColor(0.0, 0.0, 0.0);
            viewer.addPointCloud(registered_cloud, "registered_cloud");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0,
                                                    "registered_cloud");
            viewer.addPointCloud(ground_truth_cloud, "target_cloud");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.9, 0.9,
                                                    "target_cloud");
            viewer.addPointCloud(filtered_cloud, "source_cloud");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0,
                                                    "source_cloud");

            while (!viewer.wasStopped()) {
                viewer.spinOnce();
            }

            viewer.close();
        }

        T = isometryTransform;
        scale_factor = refined_scale_factor;
        std::cout << "[ICP]: Final scale factor is " << scale_factor << ". Final transformation matrix is\n" << T.matrix()
                  << std::endl;
    }


    /// Filter the disperse points
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterWithVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, float size) {
        // Create the Voxel Grid filter object
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(src_cloud); // Set the input point cloud
        sor.setLeafSize(size, size, size); // Adjust the leaf size as needed

        // Apply the filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        sor.filter(*filtered_cloud);

        // Now, 'filtered_cloud' contains the point cloud with disperse points removed

//        std::cout << "Original point cloud size: " << src_cloud->size() << std::endl;
//        std::cout << "Filtered point cloud size: " << filtered_cloud->size() << std::endl;

        return filtered_cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filterWithStatistics(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(src_cloud);
        sor.setMeanK(100);   // Number of neighbors to consider for mean and standard deviation estimation
        sor.setStddevMulThresh(0.005);  // Standard deviation threshold

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        sor.filter(*filtered_cloud);
//        std::cout << "Original point cloud size: " << src_cloud->size() << std::endl;
//        std::cout << "Filtered point cloud size: " << filtered_cloud->size() << std::endl;
        return filtered_cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filterWithCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud) {
        // Create a KD-Tree for spatial search
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
        kdtree->setInputCloud(src_cloud);

        // Create a Euclidean Cluster Extraction object
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.02);  // Set the spatial tolerance for clustering
        ec.setMinClusterSize(100);  // Set the minimum cluster size
        ec.setMaxClusterSize(25000);  // Set the maximum cluster size
        ec.setSearchMethod(kdtree);
        ec.setInputCloud(src_cloud);

        // Extract clusters
        std::vector<pcl::PointIndices> cluster_indices;
        ec.extract(cluster_indices);

        if (cluster_indices.empty()) {
            std::cerr << "No clusters found in the point cloud." << std::endl;
        }

        // Find the cluster with the maximum number of points
        std::vector<pcl::PointIndices>::const_iterator largest_cluster = std::max_element(
                cluster_indices.begin(),
                cluster_indices.end(),
                [](const pcl::PointIndices &a, const pcl::PointIndices &b) {
                    return a.indices.size() < b.indices.size();
                }
        );

        // Create a new point cloud to store the largest cluster
        pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Copy points from the largest cluster into the new point cloud
        for (int index: largest_cluster->indices) {
            largest_cluster_cloud->points.push_back(src_cloud->points[index]);
        }

        std::cout << "Original point cloud size: " << src_cloud->size() << std::endl;
        std::cout << "Filtered point cloud size: " << largest_cluster_cloud->size() << std::endl;
        return largest_cluster_cloud;
    }

    /// Registration with ICP
    Eigen::Isometry3d registrationRigid(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr &registered_cloud) {
        // Perform ICP registration
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(source_cloud);
        icp.setInputTarget(target_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        icp.align(*aligned_cloud);

        if (icp.hasConverged()) {
            std::cout << "ICP converged with score: " << icp.getFitnessScore() << std::endl;
            std::cout << "Transformation matrix:\n" << icp.getFinalTransformation() << std::endl;
            registered_cloud = aligned_cloud;

        } else {
            std::cerr << "ICP did not converge." << std::endl;
        }

        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.matrix() = icp.getFinalTransformation().cast<double>();

        return T;
    }

    /// Register with ICP-Scale
    Eigen::Matrix4d registrationRigidWithScale(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr &registered_cloud) {
        // Perform Generalized ICP (Scale-Invariant ICP)
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
        gicp.setInputSource(source_cloud);
        gicp.setInputTarget(target_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        gicp.align(*aligned_cloud);

        if (gicp.hasConverged()) {
            std::cout << "GICP converged with score: " << gicp.getFitnessScore() << std::endl;
            double scale = gicp.getFinalTransformation().block<3, 1>(0, 0).norm();
            std::cout << "Estimated scale factor: " << scale << std::endl;
            std::cout << "Transformation matrix:\n" << gicp.getFinalTransformation() << std::endl;
            registered_cloud = aligned_cloud;
        } else {
            std::cerr << "GICP did not converge." << std::endl;
        }


        Eigen::Matrix4d T = gicp.getFinalTransformation().cast<double>();

        return T;
    }

    /// Align two partially overlapped pointcloud with scale estimation
    void
    alignPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr &registered_cloud, Eigen::Matrix4d &T, double &scale) {
        /// Step 1: Estimate the initial alignment (rotation and translation)
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_source(new pcl::PointCloud<pcl::PointXYZ>);
        icp.setInputSource(source_cloud);
        icp.setInputTarget(target_cloud);
        icp.align(*aligned_source);

        if (icp.hasConverged()) {
            std::cout << "[Initial] ICP converged with score: " << icp.getFitnessScore() << std::endl;
            std::cout << "Initial Transformation matrix:\n" << icp.getFinalTransformation() << std::endl;
        } else {
            std::cerr << "ICP did not converge." << std::endl;
        }
        Eigen::Matrix4d T_initial;
        T_initial = icp.getFinalTransformation().cast<double>();


        // Step 2: Apply the estimated initial transformation to the source point cloud
        source_cloud = aligned_source;

        // Step 3: Estimate the scale factor using Generalized ICP (GICP)
        Eigen::Matrix4d T_sec;
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
        gicp.setInputSource(source_cloud);
        gicp.setInputTarget(target_cloud);

        // Optionally, set parameters for GICP
        gicp.setMaxCorrespondenceDistance(0.05);  // Maximum correspondence distance
        gicp.setMaximumIterations(50);             // Maximum iterations
        gicp.setTransformationEpsilon(1e-8);      // Transformation epsilon (convergence criteria)
        gicp.setEuclideanFitnessEpsilon(1);        // Fitness epsilon (convergence criteria)

        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        gicp.align(*aligned_cloud);

        if (gicp.hasConverged()) {
            // Step 5: Refine the transformation using the estimated scale factor
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(aligned_cloud);
            icp.setInputTarget(target_cloud);
            icp.align(*aligned_cloud);

            if (icp.hasConverged()) {
                std::cout << "[Refine] ICP converged with score: " << icp.getFitnessScore() << std::endl;
                T_sec = icp.getFinalTransformation().cast<double>();
                std::cout << "Second transformation matrix:\n" << icp.getFinalTransformation() << std::endl;
                scale = gicp.getFinalTransformation().block<3, 1>(0, 0).norm();
                std::cout << "The scale is " << scale << std::endl;
                registered_cloud = aligned_cloud;
            } else {
                std::cerr << "ICP did not converge." << std::endl;
            }
        } else {
            std::cerr << "GICP did not converge." << std::endl;
        }

        T = T_initial * T_sec;
        std::cout << "[Results]: Final transformation matrix is\n" << T << std::endl;
    }


    /// Load the point cloud data from the .ply file
    pcl::PointCloud<pcl::PointXYZ>::Ptr loadPLYFile(std::string filename) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud) == -1) {
            PCL_ERROR("Couldn't read file.\n");
        } else {
            std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << filename << std::endl;
        }
        return cloud;
    }

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_truth_cloud;
    bool visualised = true;
};

#endif //IIQC_ICP_SOLVER_H
