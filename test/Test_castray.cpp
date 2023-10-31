#include <istream>
#include <string>

#include <ros/ros.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include "IQM_image/IQM_image.h"
#include "IQM_octomap/IQM_octree.h"
#include "SemanticPC.h"

int main(int argc, char **argv) {
    /// Load point cloud
    std::string filename = "/home/feng/Code/catkin_ros/src/IIQC/data/prior_model/SemanticPC.xyz";
    SemanticPC cloud(filename);
    std::cout << "point cloud loaded, piont size = " << cloud.points.size() << std::endl;
//    cloud.Visualise();

    /// Test the initialisation of IQMOcTree
    octomap::IQMOcTree IQM_tree_pier_1(0.1);
    IQM_tree_pier_1.setMetricsThreshold(0.8, 20, 235, 1, 0.015);
    for (auto p: cloud.points) {
        IQM_tree_pier_1.initialiseIQMNode(p.x,p.y,p.z, p.r,p.g, p.b,p.semantic_label);
    }
    std::cout << "\nPost Initialisation:" << std::endl;
    IQM_tree_pier_1.printNodeInfo(0,0,0);

    /// Test IQM update of a node
//    std::cout << "\nPost Node IQM update:" << std::endl;
//    IQM_tree_pier_1.updateNodeIQM(0,0,0,0.2,128,"test",2.5);
//    IQM_tree_pier_1.printNodeInfo(0,0,0);

    /// Test IQM_image
//    cv::Mat image = cv::imread("/home/feng/Code/catkin_ros/src/IIQC/data/CapturedData/Pier1/1_colour.jpg");
//    IQMImage IQM_image(image, "test");
//    IQM_image.evaluateBPM(17);
//    IQM_image.evaluateEIM(17);
//    IQM_image.setPose(1.0, 0.0, 0.0, -90,-90,0);
//    IQM_image.setIntrinsicParas(512, 384, 731.4, 731.4);

    /// Test IQM update from a IQM image
//    IQM_tree_pier_1.updateIQMfromImage(IQM_image, 5);
//    std::cout << "IQM update from Image" << std::endl;

    /// Test Leaf Iterator
//    std::cout << "\nColor Update" << std::endl;
//    IQM_tree_pier_1.refreshTreeColor(octomap::ColorType::TRAFFIC);

    /// Test Visualisation
    ros::init(argc, argv, "Octomap_Visualisation");
    ros::NodeHandle nh;

    ros::Publisher octomap_pub=nh.advertise<octomap_msgs::Octomap>("octomap_full",1,true);

    octomap_msgs::Octomap map_msg;
    map_msg.header.frame_id="map";
    map_msg.header.stamp=ros::Time::now();

    if (octomap_msgs::fullMapToMsg(IQM_tree_pier_1,map_msg)){
        map_msg.header.stamp=ros::Time::now();
        octomap_pub.publish(map_msg);
    };

    ros::spin();




//    octomap::OcTree octomap_pier_1(0.1);
//    for (auto p: cloud.points) {
//        octomap_pier_1.updateNode(octomap::point3d(p.x, p.y, p.z+6), true);
//    }
//
//    octomap_pier_1.updateInnerOccupancy();
//
//    octomap::ColorOcTree color_octomap_pier_1(0.1);
//    for (auto p: cloud.points) {
//        color_octomap_pier_1.updateNode(octomap::point3d(p.x, p.y, p.z), true);
//        color_octomap_pier_1.setNodeColor(p.x, p.y, p.z, 128, 128, 128);
//    }
//    color_octomap_pier_1.updateInnerOccupancy();
//
//
//    // RayCasting
//    cv::Mat image = cv::imread("/home/feng/Code/catkin_ros/src/IIQC/data/Lena.jpg");
//    IQMImage IQM_image(image, "test");
//
//
//    std::cout << "" << std::endl;
//    IQM_image.setPosition(1.0, 0.0, 0.0);
//    IQM_image.setOrientation(-0.5, -0.5, 0.5, 0.6);
//    IQM_image.setIntrinsicParas(128, 128, 128, 128);
//    Eigen::Vector3d point = IQM_image.backProjection(128,128);
//    std::cout << point << std::endl;
//
//    // Go through pixel
//    for(int u = 0; u < image.size[0]; ++u) {
//        for(int v = 0; v < image.size[1]; ++v) {
//            Eigen::Vector3d point = IQM_image.backProjection(u,v);
//            Eigen::Vector3d direction_ = point - IQM_image.getPose().position;
//
//            // RayCasting
//            octomap::point3d origin(IQM_image.getPose().position.x(), IQM_image.getPose().position.y(), IQM_image.getPose().position.z());
//            octomap::point3d direction(direction_.x(), direction_.y(), direction_.z());
//            octomap::point3d end;
//            if (color_octomap_pier_1.castRay(origin, direction, end, true, 5)){
//                color_octomap_pier_1.setNodeColor(end.x(),end.y(),end.z(),255,0,0);
//            }
//        }
//    }
//
//    // Publish the map_msg
//    ros::init(argc, argv, "Octomap_Visualisation");
//    ros::NodeHandle nh;
//
//    ros::Publisher octomap_pub=nh.advertise<octomap_msgs::Octomap>("octomap_full",1,true);
//
//    octomap_msgs::Octomap map_msg;
//    map_msg.header.frame_id="map";
//    map_msg.header.stamp=ros::Time::now();
//
//    if (octomap_msgs::fullMapToMsg(color_octomap_pier_1,map_msg)){
//        map_msg.header.stamp=ros::Time::now();
//        octomap_pub.publish(map_msg);
//    };
//
//    ros::spin();

    return 0;
}
