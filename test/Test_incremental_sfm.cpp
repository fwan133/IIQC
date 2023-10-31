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
    int length = 0;
    std::string gt_cloud_path = "/home/feng/Code/catkin_ros/src/IIQC/data/prior_model/GirderBridgeSim/PointCloud.ply";
    std::string img_folder_path = "/home/feng/Code/catkin_ros/src/IIQC/data/CapturedData/Simulation/GirderBridge/girder_bridge_pier";
    std::string registered_cloud_path = "/home/feng/Code/catkin_ros/src/IIQC/data/sfm_pointcloud/girder_bridge_sim/girder_bridge_pier.ply";

    /// Load image data and define parameters
    CapturedFrameCollection frame_collection("", img_folder_path, length, false);
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
    reconstructor->setICPSolver(gt_cloud_path, false);

    for (int index = 1; index <= frame_collection.GetSize(); index++) {
        // Obtain basic info of the incoming image
        CapturedFrame inputCapturedFrame = frame_collection.getFrameByIndex(index);
        std::string img_path = inputCapturedFrame.GetImgPath();
        cv::Mat inputImg = cv::imread(img_path);
        Eigen::Isometry3d T_gps = inputCapturedFrame.GetGeoCoordinate().getEigenPose() * T_body_c;
        Eigen::Isometry3d T_gt = inputCapturedFrame.GetGroundTruthPose().cvt2TransformMatrix() * T_body_c;

        // Add new image and do feature matching
        reconstructor->addImage(img_path, inputImg, T_gps);

        // Registration and Triangulation
        if (index == 1) {
            reconstructor->getCurrentFrame()->setTwc_est(T_gps);
            reconstructor->getCurrentFrame()->setRegistered(true);
        } else if (index == 2) {
            reconstructor->initialise(true);
        } else if (index >= 3) {
            reconstructor->registration();
            reconstructor->triangulate();

            if (index % 3 == 0) reconstructor->localOptimisation();
            if (index % 5 ==0) reconstructor->globalOptimisation();
            if (index == frame_collection.GetSize()){
                reconstructor->globalOptimisation();
            }
        }
    }

    /// Align2GPS
    reconstructor->align2GPS();
    /// ICP
    reconstructor->align2Cloud();

    std::cout << "[1]\n";
    // Save the data
    reconstructor->getMap()->writePLYBinary(registered_cloud_path);

    // Update the estimated poses and save the results
    for (uint index = 1; index <= frame_collection.GetSize(); index++){
        long id = index -1;
        Eigen::Isometry3d T_est_new = reconstructor->getMap()->getAllFrames().at(id)->getTwc_est();
        Eigen::Isometry3d T_final = T_est_new * (T_body_c.inverse());
        frame_collection.setEstByIndex(index, T_final);
    }
    frame_collection.SaveToFile(img_folder_path, false);

    /// Join the render thread
    renderThread.join();

    return 0;
}