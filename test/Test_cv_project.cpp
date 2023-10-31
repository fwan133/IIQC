#include <opencv2/opencv.hpp>
#include "Utils.h"

int main() {
    Eigen::Isometry3d T_w_c = Eigen::Isometry3d::Identity();
    T_w_c.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));
    Eigen::Vector3d pos3d (0,-1,0);
    T_w_c.pretranslate(pos3d);

    // Example intrinsic matrix (3x3)
    cv::Mat intrinsic_matrix = (cv::Mat_<double>(3, 3) << 1000, 0, 500, 0, 1000, 500, 0, 0, 1);


    // Example 3D world point (x, y, z)
    cv::Mat world_point = (cv::Mat_<double>(3, 1) << 0, 1, 1);

    // Project the 3D point to the image plane using OpenCV
    cv::Mat image_point;
    cv::projectPoints(world_point, Utils::IsometryToMatR(T_w_c.inverse()), Utils::IsometryToMatt(T_w_c.inverse()), intrinsic_matrix, cv::Mat(), image_point);

    // Extract the 2D image coordinates
    cv::Point2d projected_point = image_point.at<cv::Point2d>(0, 0);

    // Print the projected image point
    std::cout << "Projected image point: (" << projected_point.x << ", " << projected_point.y << ")" << std::endl;

    return 0;
}
