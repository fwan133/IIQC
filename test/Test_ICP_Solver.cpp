#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "ICP/icp_solver.h"


int main (int argc, char **argv){
    std::string sfm_pc_filename = "/home/feng/Code/catkin_ros/src/IIQC/data/sfm_pointcloud/girder_bridge_sim/girder_bridge_pier.ply";
    std::string gt_pc_filename = "/home/feng/Code/catkin_ros/src/IIQC/data/prior_model/GirderBridgeSim/PointCloud.ply";

    ICPSolver::Ptr icpSolver = ICPSolver::Ptr(new ICPSolver());
    icpSolver->setICPSolver(gt_pc_filename, true);

    pcl::PointCloud<pcl::PointXYZ>::Ptr src_pc=icpSolver->loadPLYFile(sfm_pc_filename);

    /// Registration
    pcl::PointCloud<pcl::PointXYZ>::Ptr registered_pc;
    Eigen::Isometry3d T;
    double scale;

    /// Point cloud alignment
    icpSolver->registerWithScaleGICP(src_pc,registered_pc, T, scale);



    return 0;
}