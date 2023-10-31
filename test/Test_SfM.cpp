#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

// Read images from a directory
std::vector<cv::Mat> readImagesFromDirectory(const std::string& directory) {
    std::vector<cv::Mat> images;

    // Open the directory
    cv::String directory_path(directory);
    std::vector<cv::String> file_paths;
    cv::glob(directory_path, file_paths);

    // Read images and store them in a vector
    for (const cv::String& file_path : file_paths) {
        cv::Mat image = cv::imread(file_path);
        if (!image.empty()) {
            images.push_back(image);
        } else {
            std::cerr << "Failed to read image: " << file_path << std::endl;
        }
    }

    return images;
}


// Function to perform incremental SfM
void IncrementalSfM(const std::vector<cv::Mat>& images, std::vector<cv::Mat>& poses, std::vector<cv::Point3f>& points) {
    cv::Ptr<cv::Feature2D> feature_detector = cv::ORB::create();
    cv::BFMatcher matcher(cv::NORM_HAMMING);

    std::vector<cv::KeyPoint> prev_keypoints, curr_keypoints;
    cv::Mat prev_descriptors, curr_descriptors;

    // Initialize the first image
    cv::Mat prev_image = images[0];
    std::vector<cv::Point2f> prev_points;
    feature_detector->detect(prev_image, prev_keypoints);
    feature_detector->compute(prev_image, prev_keypoints, prev_descriptors);
    cv::KeyPoint::convert(prev_keypoints, prev_points);

    poses.push_back(cv::Mat::eye(4, 4, CV_64F)); // Initial pose as identity matrix

    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F); // Assuming a simple identity camera matrix

    for (int i = 1; i < images.size(); ++i) {
        curr_keypoints.clear();
        curr_descriptors.release();

        // Detect keypoints and compute descriptors for the current image
        cv::Mat curr_image = images[i];
        feature_detector->detectAndCompute(curr_image, cv::noArray(), curr_keypoints, curr_descriptors);

        std::vector<cv::DMatch> matches;
        matcher.match(prev_descriptors, curr_descriptors, matches);

        // Essential matrix or fundamental matrix estimation
        std::vector<cv::Point2f> prev_matched_points, curr_matched_points;
        for (const cv::DMatch& match : matches) {
            prev_matched_points.push_back(prev_points[match.queryIdx]);
            curr_matched_points.push_back(curr_keypoints[match.trainIdx].pt);
        }

        cv::Mat essential_matrix = cv::findEssentialMat(
                curr_matched_points, prev_matched_points, camera_matrix, cv::RANSAC, 0.999, 1.0);

        // Recover relative pose
        cv::Mat rotation, translation;
        cv::recoverPose(essential_matrix, curr_matched_points, prev_matched_points, rotation, translation);

        // Triangulate 3D points
        std::vector<cv::Point3f> new_points;
        cv::triangulatePoints(
                cv::Mat::eye(3, 4, CV_64F), cv::Mat::eye(3, 4, CV_64F),
                prev_matched_points, curr_matched_points, new_points
        );

        // Transform the new points to the global coordinate system
        cv::Mat Rt = cv::Mat::eye(4, 4, CV_64F);
        rotation.copyTo(Rt(cv::Rect(0, 0, 3, 3)));
        translation.copyTo(Rt(cv::Rect(3, 0, 1, 3)));
        poses.push_back(poses.back() * Rt);

        // Update 'points' vector
        points.insert(points.end(), new_points.begin(), new_points.end());

        // Update previous keypoints and descriptors
        prev_keypoints = curr_keypoints;
        prev_descriptors = curr_descriptors;
        prev_points = curr_matched_points;
    }
}

int main() {
    std::string image_directory = "/home/feng/Code/catkin_ros/src/IIQC/data/dji_data/"; // Replace with your image directory

    // Read images
    std::vector<cv::Mat> images = readImagesFromDirectory(image_directory);

    // Initialize data structures for camera poses and 3D points
    std::vector<cv::Mat> poses; // Represent camera poses as matrices (4x4)
    std::vector<cv::Point3f> points; // Represent 3D points as Point3f

    // Perform incremental SfM
    IncrementalSfM(images, poses, points);

    // Save camera poses and point cloud to files
    cv::FileStorage fs("camera_poses.yml", cv::FileStorage::WRITE);
    for (int i = 0; i < poses.size(); ++i) {
        fs << "Pose" << poses[i];
    }
    fs.release();

    fs.open("point_cloud.yml", cv::FileStorage::WRITE);
    for (int i = 0; i < points.size(); ++i) {
        fs << "Point" << points[i];
    }
    fs.release();

    std::cout << "Camera poses and point cloud saved to 'camera_poses.yml' and 'point_cloud.yml'." << std::endl;

    return 0;
}
