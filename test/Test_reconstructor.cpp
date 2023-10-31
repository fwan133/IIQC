#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

#include "flight_control/captured_frame_collection.h"
#include "config.h"
#include "SfM/frame.h"
#include "SfM/reconstructor.h"
#include "Viewer/viewer.h"

void render(sfm::Map::Ptr map) {
    Viewer::Ptr viewer = Viewer::Ptr(new Viewer());
    viewer->drawMap(map);
}

int main(int argc, char **argv) {
    int length = 100;
    std::string gt_cloud_path = "/home/feng/Code/catkin_ros/src/IIQC/data/prior_model/ArchBridgeSim/PointCloud.ply";
    std::string img_folder_path = "/home/feng/Code/catkin_ros/src/IIQC/data/CapturedData/Simulation/ArchBridge/arch_bridge";
    std::string registered_cloud_path = "/home/feng/Code/catkin_ros/src/IIQC/data/sfm_pointcloud/arch_bridge/arch_bridge.ply";

    /// Load image data and define parameters
    CapturedFrameCollection frame_collection("Nothing",img_folder_path, length, false);
    Eigen::Isometry3d T_body_c = Eigen::Isometry3d::Identity();
    T_body_c.matrix().topLeftCorner<3, 3>() << 0, 0, 1, -1, 0, 0, 0, -1, 0;

    /// Read parameters
    Config::instance()->setParameterFile("/home/feng/Code/catkin_ros/src/IIQC/config/config.yaml");

    int maxNumFeatures = Config::instance()->get<int>("max_num_features");
    int maxImageSize = Config::instance()->get<int>("max_image_size");

    int match_type = 0;
    int sequential_overlap = 2;
    int max_overlap = 3;
    float ratio_thresh = Config::instance()->get<float>("ratio_thresh");
    float distance_thresh = Config::instance()->get<float>("distance_thresh");
    int min_num_matches = Config::instance()->get<int>("min_num_matches");
    int cross_check = Config::instance()->get<int>("cross_check");

    /// Create a reconstructor
    sfm::Reconstructor::Ptr reconstructor = sfm::Reconstructor::Ptr(new sfm::Reconstructor);
    sfm::Map::Ptr map = reconstructor->getMap();
    // Create a viewer
    std::thread renderThread(&render, std::ref(map));

    // SetParameters
    reconstructor->setFeatureDetector(maxImageSize, maxNumFeatures, sfm::Frame::KeyPointType::SIFT);
    reconstructor->setMatcher(match_type, sequential_overlap, max_overlap, ratio_thresh, distance_thresh,
                              min_num_matches, cross_check, false);
    reconstructor->setICPSolver(gt_cloud_path, true);

    for (int index = 1; index <= frame_collection.GetSize(); index++) {
        Eigen::Isometry3d T_gps = frame_collection.getFrameByIndex(index).GetGeoCoordinate().getEigenPose() * T_body_c;
        Eigen::Isometry3d T_gt = frame_collection.getFrameByIndex(index).GetGroundTruthPose().cvt2TransformMatrix() * T_body_c;

        reconstructor->addNewImg(frame_collection.getFrameByIndex(index).GetImgPath(),T_gps);
    }

    reconstructor->finalRefine();

    renderThread.join();

    // Save the data
    reconstructor->getMap()->writePLYBinary(registered_cloud_path);

    // Save the estimated image poses
    frame_collection.SaveToFile(img_folder_path,false);

    return 0;
}