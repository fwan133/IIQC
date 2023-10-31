#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <thread>

cv::Mat img;

void showImageThread(cv::Mat img){
    cv::namedWindow("Image", cv::WINDOW_NORMAL);
    while (true){
        if(!img.empty()){
            cv::imshow("Image", img);
        }
    }

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // Convert the ROS image message to an OpenCV image
        img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Error converting image: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_subscriber_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/typhoon_h480_0/camera/image_raw", 1, imageCallback);

    std::thread imageThread(showImageThread, img);

    ros::spin();
    return 0;
}
