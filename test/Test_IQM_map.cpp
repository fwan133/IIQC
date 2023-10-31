#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include "IQM_octomap/IQM_octree.h"
#include "flight_control/captured_frame_collection.h"

void publishPoses(tf2_ros::TransformBroadcaster &tf_broadcaster, std::vector<Eigen::Isometry3d> &imagePoses){
    for (size_t i = 0; i < imagePoses.size(); i++)
    {
    // Create a TransformStamped message
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "map"; // Fixed frame
    transform.child_frame_id = "Image_" + std::to_string(i); // Unique frame for each image

    // Extract position and orientation from the image pose
    Eigen::Isometry3d T_w_c = imagePoses[i];
    transform.transform.translation.x = T_w_c.translation().x(); // X position
    transform.transform.translation.y = T_w_c.translation().y(); // Y position
    transform.transform.translation.z = T_w_c.translation().z(); // Z position

    // Extract orientation from the image pose (you need to convert from the rotation matrix)
    Eigen::Quaterniond q = Eigen::Quaterniond(T_w_c.linear());
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    // Broadcast the image pose transformation
    tf_broadcaster.sendTransform(transform);
    }
}

void publishPoseAsArrowArray(ros::Publisher &marker_array_pub, visualization_msgs::MarkerArray &marker_array, Eigen::Isometry3d &imagePose, int id){
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp
    marker.header.frame_id = "map";  // Set the frame you want to use
    marker.header.stamp = ros::Time::now();
    marker.ns = "arrow_markers";
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the position and orientation
    double angle = -90.0 * M_PI / 180.0;
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY());
    imagePose.rotate(rotationMatrix);
    Eigen::Vector3d img_position = imagePose.translation();
    marker.pose.position.x = img_position.x();
    marker.pose.position.y = img_position.y();
    marker.pose.position.z = img_position.z();
    Eigen::Quaterniond img_orientation(imagePose.linear());
    marker.pose.orientation.x = img_orientation.x();
    marker.pose.orientation.y = img_orientation.y();
    marker.pose.orientation.z = img_orientation.z();
    marker.pose.orientation.w = img_orientation.w();

    // Set the scale of the arrow
    marker.scale.x = 0.5;
    marker.scale.y = 0.1;  // Width of the arrow
    marker.scale.z = 0.1;  // Height of the arrow
    // Set the color (RGBA)
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;  // Fully opaque

    marker.lifetime = ros::Duration(1000);

    marker_array.markers.push_back(marker);
    marker_array_pub.publish(marker_array);
}

void publishPoseArray(){

}

int main(int argc, char **argv) {
    /// Basic variables
    std::string cloud_filename = "/home/feng/Code/catkin_ros/src/IIQC/data/prior_model/GirderBridgeSim/PointCloud.ply";
    std::string imgs_folder = "/home/feng/Code/catkin_ros/src/IIQC/data/CapturedData/Simulation/GirderBridge/girder_bridge_pier";
    double fx = 3648, fy = 3648, cx = 2736, cy = 1824;
    double size = 0.1;
    double blur_threshold = 0.8;
    double exposure_min = 20;
    double exposure_max = 232;
    uint times_threshold = 3;
    double ssd_threshold = 1;
    int patch_size = 17;

    /// Load images
    CapturedFrameCollection frame_collection(imgs_folder, "final_images_info.txt", "Pier",0, false);
    Eigen::Isometry3d T_body_c = Eigen::Isometry3d::Identity();
    T_body_c.matrix().topLeftCorner<3, 3>() << 0, 0, 1, -1, 0, 0, 0, -1, 0;

    /// Load the point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::io::loadPLYFile<pcl::PointXYZ>(cloud_filename, cloud);
    std::cout << "point cloud loaded, cloud size = " << cloud.points.size() << std::endl;

    /// Initialise the IQM_Octomap from gt cloud
    octomap::IQMOcTree IQM_octree(size);
    IQM_octree.setMetricsThreshold(blur_threshold,exposure_min,exposure_max, times_threshold, ssd_threshold);
    std::cout << IQM_octree.getTreeType() << std::endl;

    for (auto p: cloud.points) {
        IQM_octree.initialiseIQMNode(p.x, p.y, p.z, 128, 128, 128, "pier");
    }

    /// Initialise ROS node and Publish the initial map
    ros::init(argc, argv, "Octomap_Visualisation");
    ros::NodeHandle nh;

    // Octomap
    ros::Publisher octomap_pub=nh.advertise<octomap_msgs::Octomap>("octomap_full",1,true);
    octomap_msgs::Octomap map_msg;
    map_msg.header.frame_id="map";
    map_msg.header.stamp=ros::Time::now();
    if (octomap_msgs::fullMapToMsg(IQM_octree,map_msg)){
        map_msg.header.stamp=ros::Time::now();
        octomap_pub.publish(map_msg);
    };
    // Image Poses using arrow marker
    ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("marker_array", 1);
    visualization_msgs::MarkerArray marker_array;

    /// Update the Image
    for (int index = 1; index <= frame_collection.GetSize(); index++){
        /// Generate a IQM image object
        Eigen::Isometry3d T_w_c = frame_collection.getFrameByIndex(index).GetEstimatedPose().cvt2TransformMatrix() * T_body_c;
        IQMImage IQM_img("/home/feng/Code/catkin_ros/src/IIQC/data/CapturedData/Simulation/GirderBridge/girder_bridge_pier/1_colour.jpg");
        IQM_img.setPose(Pose6d(T_w_c));
        IQM_img.setIntrinsicParas(cx, cy, fx, fy);
        IQM_img.evaluateBPM(patch_size);
        IQM_img.evaluateEIM(patch_size);
        IQM_octree.updateIQMfromImage(IQM_img, 10);
        IQM_octree.refreshTreeColor(octomap::ColorType::MULTIPLE);

        /// Publish camera pose
        publishPoseAsArrowArray(marker_array_pub, marker_array, T_w_c, index);
        std::cout << "[Image]: " << index << std::endl;

        /// Publish the map
        map_msg.header.stamp=ros::Time::now();
        if (octomap_msgs::fullMapToMsg(IQM_octree,map_msg)){
            map_msg.header.stamp=ros::Time::now();
            octomap_pub.publish(map_msg);
        };
    }

    ros::spin();

    return 0;
}
