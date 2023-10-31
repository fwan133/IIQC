#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {
    cv::Mat img_1 = cv::imread("/home/feng/Code/catkin_ros/src/IIQC/data/dji_data/DJI_0338.JPG", cv::IMREAD_COLOR);
    cv::Mat img_2 = cv::imread("/home/feng/Code/catkin_ros/src/IIQC/data/dji_data/DJI_0339.JPG", cv::IMREAD_COLOR);

    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;

    // ORB
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
//    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
//    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce_Hamming");

    // Step 1 Detect FAST corner
    std::chrono::steady_clock::time_point t1= std::chrono::steady_clock::now();
    detector->detectAndCompute(img_1, cv::noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(img_2, cv::noArray(), keypoints2, descriptors2);
    std::chrono::steady_clock::time_point t2= std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
    std::cout << "Extract ORB feature cost: " << time_used.count() << " seconds" << std::endl;

    // Step 2 Match
    cv::BFMatcher bf(cv::NORM_HAMMING, true);
    std::vector<cv::DMatch> matches;
    bf.match(descriptors1, descriptors2, matches);
    std::sort(matches.begin(), matches.end());

    cv::Mat result_image;
    cv::drawMatches(img_1, keypoints1, img_2, keypoints2, matches, result_image, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    // Display the result
    cv::Mat resized_img;
    cv::resize(result_image,resized_img,cv::Size(0,0), 0.125,0.125);
    cv::imshow("ORB Matches", resized_img);
    cv::waitKey(0);
    cv::destroyAllWindows();

}